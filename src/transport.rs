//! Additional Modbus transports not provided by `tokio-modbus`: Modbus/UDP
//! and Modbus ASCII. Both are implemented as `tokio_modbus::client::Client`s so
//! the rest of the program can use the standard `Reader`/`Writer` helpers.
//!
//! Modbus RTU-over-TCP ("enc") does not need a custom client: it is just the
//! RTU codec attached to a `TcpStream` (`tokio_modbus::client::rtu::attach_slave`).

use std::io;
use std::pin::Pin;
use std::task::{Context, Poll};
use std::time::Duration;

use async_trait::async_trait;
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt, ReadBuf};
use tokio::time::Sleep;
use tokio_serial::SerialPort;

use tokio_modbus::Error;
use tokio_modbus::bytes::Bytes;
use tokio_modbus::client::Client;
use tokio_modbus::prelude::{
    ConformityLevel, DeviceIdObject, ExceptionCode, ReadCode, ReadDeviceIdentificationResponse,
    Request, Response, Slave, SlaveContext,
};

/// Append a big-endian `u16` to a byte buffer.
pub(crate) fn push_u16(buf: &mut Vec<u8>, value: u16) {
    buf.push((value >> 8) as u8);
    buf.push(value as u8);
}

fn protocol_err(msg: impl Into<String>) -> Error {
    Error::Transport(io::Error::other(msg.into()))
}

// ---------------------------------------------------------------------------
// Modbus PDU encode/decode
// ---------------------------------------------------------------------------

fn encode_request_pdu(req: &Request<'_>) -> Result<Vec<u8>, String> {
    let mut p = Vec::new();
    match req {
        Request::ReadCoils(addr, cnt) => {
            p.push(0x01);
            push_u16(&mut p, *addr);
            push_u16(&mut p, *cnt);
        }
        Request::ReadDiscreteInputs(addr, cnt) => {
            p.push(0x02);
            push_u16(&mut p, *addr);
            push_u16(&mut p, *cnt);
        }
        Request::ReadHoldingRegisters(addr, cnt) => {
            p.push(0x03);
            push_u16(&mut p, *addr);
            push_u16(&mut p, *cnt);
        }
        Request::ReadInputRegisters(addr, cnt) => {
            p.push(0x04);
            push_u16(&mut p, *addr);
            push_u16(&mut p, *cnt);
        }
        Request::WriteSingleCoil(addr, coil) => {
            p.push(0x05);
            push_u16(&mut p, *addr);
            p.push(if *coil { 0xFF } else { 0x00 });
            p.push(0x00);
        }
        Request::WriteSingleRegister(addr, word) => {
            p.push(0x06);
            push_u16(&mut p, *addr);
            push_u16(&mut p, *word);
        }
        Request::WriteMultipleCoils(addr, coils) => {
            let count = coils.len();
            let byte_count = count.div_ceil(8);
            p.push(0x0F);
            push_u16(&mut p, *addr);
            push_u16(&mut p, count as u16);
            p.push(byte_count as u8);
            let mut bytes = vec![0u8; byte_count];
            for (i, coil) in coils.iter().enumerate() {
                if *coil {
                    bytes[i / 8] |= 1 << (i % 8);
                }
            }
            p.extend_from_slice(&bytes);
        }
        Request::WriteMultipleRegisters(addr, words) => {
            p.push(0x10);
            push_u16(&mut p, *addr);
            push_u16(&mut p, words.len() as u16);
            p.push((words.len() * 2) as u8);
            for word in words.iter() {
                push_u16(&mut p, *word);
            }
        }
        Request::ReadDeviceIdentification(read_code, object_id) => {
            p.push(0x2B);
            p.push(0x0E);
            p.push(read_code.value());
            p.push(*object_id);
        }
        Request::Custom(func, data) => {
            p.push(*func);
            p.extend_from_slice(data);
        }
        _ => return Err(format!("unsupported request {req:?}")),
    }
    Ok(p)
}

fn decode_response_pdu(func: u8, data: &[u8]) -> Result<Response, String> {
    match func {
        0x01 | 0x02 => {
            let byte_count = *data.first().ok_or("missing byte count")? as usize;
            let bytes = data.get(1..).ok_or("missing coil data")?;
            if bytes.len() < byte_count {
                return Err("short coil data".into());
            }
            let mut bits = Vec::with_capacity(byte_count * 8);
            for byte in bytes.iter().take(byte_count) {
                for i in 0..8 {
                    bits.push((byte >> i) & 1 == 1);
                }
            }
            if func == 0x01 {
                Ok(Response::ReadCoils(bits))
            } else {
                Ok(Response::ReadDiscreteInputs(bits))
            }
        }
        0x03 | 0x04 => {
            let byte_count = *data.first().ok_or("missing byte count")? as usize;
            let bytes = data.get(1..).ok_or("missing register data")?;
            if bytes.len() < byte_count {
                return Err("short register data".into());
            }
            let mut words = Vec::with_capacity(byte_count / 2);
            for i in 0..(byte_count / 2) {
                words.push(u16::from_be_bytes([bytes[2 * i], bytes[2 * i + 1]]));
            }
            if func == 0x03 {
                Ok(Response::ReadHoldingRegisters(words))
            } else {
                Ok(Response::ReadInputRegisters(words))
            }
        }
        0x05 => {
            let addr = be_u16(data, 0)?;
            let value = be_u16(data, 2)?;
            Ok(Response::WriteSingleCoil(addr, value == 0xFF00))
        }
        0x06 => {
            let addr = be_u16(data, 0)?;
            let value = be_u16(data, 2)?;
            Ok(Response::WriteSingleRegister(addr, value))
        }
        0x0F => {
            let addr = be_u16(data, 0)?;
            let qty = be_u16(data, 2)?;
            Ok(Response::WriteMultipleCoils(addr, qty))
        }
        0x10 => {
            let addr = be_u16(data, 0)?;
            let qty = be_u16(data, 2)?;
            Ok(Response::WriteMultipleRegisters(addr, qty))
        }
        0x2B => decode_device_id(data),
        other => Ok(Response::Custom(other, Bytes::from(data.to_vec()))),
    }
}

fn be_u16(data: &[u8], at: usize) -> Result<u16, String> {
    let hi = *data.get(at).ok_or("truncated response")?;
    let lo = *data.get(at + 1).ok_or("truncated response")?;
    Ok(u16::from_be_bytes([hi, lo]))
}

fn decode_device_id(data: &[u8]) -> Result<Response, String> {
    if data.first().copied() != Some(0x0E) {
        return Err("unexpected MEI type in device identification response".into());
    }
    let read_code = *data.get(1).ok_or("missing read code")?;
    let conformity = *data.get(2).ok_or("missing conformity level")?;
    let more_follows = *data.get(3).ok_or("missing more-follows flag")?;
    let next_id = *data.get(4).ok_or("missing next object id")?;
    let object_count = *data.get(5).ok_or("missing object count")? as usize;

    let mut objects = Vec::with_capacity(object_count);
    let mut idx = 6;
    for _ in 0..object_count {
        let id = *data.get(idx).ok_or("short object id")?;
        let len = *data.get(idx + 1).ok_or("short object length")? as usize;
        let value = data
            .get(idx + 2..idx + 2 + len)
            .ok_or("short object value")?;
        objects.push(DeviceIdObject {
            id,
            value: Bytes::from(value.to_vec()),
        });
        idx += 2 + len;
    }

    Ok(Response::ReadDeviceIdentification(
        ReadDeviceIdentificationResponse {
            read_code: ReadCode::try_from_value(read_code).ok_or("invalid read code")?,
            conformity_level: ConformityLevel::try_from_value(conformity)
                .ok_or("invalid conformity level")?,
            more_follows: more_follows == 0xFF,
            next_object_id: next_id,
            device_id_objects: objects,
        },
    ))
}

// ---------------------------------------------------------------------------
// Modbus UDP client
// ---------------------------------------------------------------------------

/// Abstraction over the datagram send/receive that [`UdpClient`] needs.
/// Implemented by `tokio::net::UdpSocket` in production and by a mock in tests.
#[async_trait]
trait UdpTransport: Send {
    async fn send(&mut self, data: &[u8]) -> io::Result<usize>;
    async fn recv(&mut self, buf: &mut [u8]) -> io::Result<usize>;
}

#[async_trait]
impl UdpTransport for tokio::net::UdpSocket {
    async fn send(&mut self, data: &[u8]) -> io::Result<usize> {
        self.send(data).await
    }

    async fn recv(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.recv(buf).await
    }
}

/// A Modbus-over-UDP client using the same MBAP header as Modbus TCP.
pub struct UdpClient<T> {
    transport: T,
    slave: Slave,
    transaction: u16,
}

impl UdpClient<tokio::net::UdpSocket> {
    /// Connects to a Modbus UDP server.
    pub async fn connect(addr: std::net::SocketAddr, slave: Slave) -> io::Result<Self> {
        let socket = tokio::net::UdpSocket::bind("0.0.0.0:0").await?;
        socket.connect(addr).await?;
        Ok(Self::new(socket, slave))
    }
}

impl<T> UdpClient<T> {
    pub fn new(transport: T, slave: Slave) -> Self {
        Self {
            transport,
            slave,
            transaction: 1,
        }
    }
}

#[async_trait]
impl<T> Client for UdpClient<T>
where
    T: UdpTransport + 'static,
{
    async fn call(&mut self, req: Request<'_>) -> tokio_modbus::Result<Response> {
        let pdu = encode_request_pdu(&req).map_err(protocol_err)?;

        let transaction = self.transaction;
        self.transaction = self.transaction.wrapping_add(1);

        // MBAP header: transaction id (2), protocol id (2, =0), length (2),
        // unit id (1), then the PDU.
        let mut frame = Vec::with_capacity(7 + pdu.len());
        push_u16(&mut frame, transaction);
        push_u16(&mut frame, 0);
        push_u16(&mut frame, (1 + pdu.len()) as u16);
        frame.push(self.slave.0);
        frame.extend_from_slice(&pdu);

        self.transport.send(&frame).await?;

        let mut buf = [0u8; 260];
        let n = self.transport.recv(&mut buf).await?;
        if n < 8 {
            return Err(protocol_err("short UDP response"));
        }
        let resp_txn = u16::from_be_bytes([buf[0], buf[1]]);
        if resp_txn != transaction {
            return Err(protocol_err(format!(
                "mismatching UDP transaction id (expected {transaction}, got {resp_txn})"
            )));
        }
        let pdu_bytes = buf.get(7..n).ok_or_else(|| protocol_err("empty UDP PDU"))?;
        let (resp_func, payload) = pdu_bytes
            .split_first()
            .ok_or_else(|| protocol_err("empty UDP PDU"))?;

        if resp_func & 0x80 != 0 {
            let code = payload.first().copied().unwrap_or(0);
            return Ok(Err(ExceptionCode::new(code)));
        }
        decode_response_pdu(*resp_func, payload)
            .map(Ok)
            .map_err(protocol_err)
    }

    async fn disconnect(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl<T> SlaveContext for UdpClient<T> {
    fn set_slave(&mut self, slave: Slave) {
        self.slave = slave;
    }
}

// ---------------------------------------------------------------------------
// RS-485 aware serial transport
// ---------------------------------------------------------------------------

/// Polling interval while waiting for the serial driver to finish sending
/// pending bytes (`bytes_to_write() == 0`).
const DRAIN_POLL_INTERVAL: Duration = Duration::from_millis(1);

enum FlushState {
    Idle,
    Flushing,
    Draining(Pin<Box<Sleep>>),
    Waiting(Pin<Box<Sleep>>),
}

/// Abstraction over the RTS control and transmit-queue inspection that
/// [`RtsStream`] needs. Implemented by `tokio_serial::SerialStream` in
/// production and by a mock in tests.
trait RtsControl {
    fn set_rts(&mut self, level: bool) -> io::Result<()>;
    fn pending_bytes(&mut self) -> io::Result<u32>;
}

impl RtsControl for tokio_serial::SerialStream {
    fn set_rts(&mut self, level: bool) -> io::Result<()> {
        self.write_request_to_send(level).map_err(io::Error::other)
    }

    fn pending_bytes(&mut self) -> io::Result<u32> {
        self.bytes_to_write().map_err(io::Error::other)
    }
}

/// Wraps a serial transport and, when an RTS delay is configured, asserts RTS
/// before writing and deasserts it after the data has been flushed plus the
/// requested delay. This is the classic half-duplex RS-485 turnaround control.
pub struct RtsStream<T> {
    inner: T,
    delay: Option<Duration>,
    state: FlushState,
}

impl<T> RtsStream<T> {
    pub fn new(inner: T, delay: Option<Duration>) -> Self {
        Self {
            inner,
            delay,
            state: FlushState::Idle,
        }
    }

    /// Advances the flush state machine through the drain stage: once the
    /// driver has no pending bytes, schedule the RTS-off delay. If the driver
    /// cannot report pending bytes, fall back to just the delay.
    fn drain_check(&mut self)
    where
        T: RtsControl,
    {
        let delay = self.delay.expect("drain only runs with an RTS delay");
        match self.inner.pending_bytes() {
            Ok(0) | Err(_) => {
                self.state = FlushState::Waiting(Box::pin(tokio::time::sleep(delay)));
            }
            Ok(_) => {
                self.state =
                    FlushState::Draining(Box::pin(tokio::time::sleep(DRAIN_POLL_INTERVAL)));
            }
        }
    }
}

impl<T> AsyncRead for RtsStream<T>
where
    T: AsyncRead + Unpin,
{
    fn poll_read(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut ReadBuf<'_>,
    ) -> Poll<io::Result<()>> {
        Pin::new(&mut self.get_mut().inner).poll_read(cx, buf)
    }
}

impl<T> AsyncWrite for RtsStream<T>
where
    T: AsyncWrite + RtsControl + Unpin,
{
    fn poll_write(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<io::Result<usize>> {
        let this = self.get_mut();
        if this.delay.is_some()
            && let Err(e) = this.inner.set_rts(true)
        {
            return Poll::Ready(Err(e));
        }
        Pin::new(&mut this.inner).poll_write(cx, buf)
    }

    fn poll_flush(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        let this = self.get_mut();
        if this.delay.is_none() {
            return Pin::new(&mut this.inner).poll_flush(cx);
        }

        loop {
            match std::mem::replace(&mut this.state, FlushState::Idle) {
                FlushState::Idle => {
                    this.state = FlushState::Flushing;
                }
                FlushState::Flushing => {
                    match Pin::new(&mut this.inner).poll_flush(cx) {
                        Poll::Ready(Ok(())) => {}
                        Poll::Ready(Err(e)) => return Poll::Ready(Err(e)),
                        Poll::Pending => return Poll::Pending,
                    }
                    this.drain_check();
                }
                FlushState::Draining(mut sleep) => {
                    if sleep.as_mut().poll(cx).is_pending() {
                        this.state = FlushState::Draining(sleep);
                        return Poll::Pending;
                    }
                    this.drain_check();
                }
                FlushState::Waiting(mut sleep) => {
                    if sleep.as_mut().poll(cx).is_pending() {
                        this.state = FlushState::Waiting(sleep);
                        return Poll::Pending;
                    }
                    if let Err(e) = this.inner.set_rts(false) {
                        return Poll::Ready(Err(e));
                    }
                    this.state = FlushState::Idle;
                    return Poll::Ready(Ok(()));
                }
            }
        }
    }

    fn poll_shutdown(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        let this = self.get_mut();
        if this.delay.is_some() {
            let _ = this.inner.set_rts(false);
        }
        Pin::new(&mut this.inner).poll_shutdown(cx)
    }
}

/// `RtsStream` over a real serial port.
pub type RtsSerialStream = RtsStream<tokio_serial::SerialStream>;

// ---------------------------------------------------------------------------
// Modbus ASCII client
// ---------------------------------------------------------------------------

/// A Modbus ASCII client. The PDU is framed with `:` / CRLF and an LRC checksum
/// and encoded as hexadecimal characters over a serial line.
pub struct AsciiClient<T> {
    port: T,
    slave: Slave,
}

impl<T> AsciiClient<T> {
    pub fn new(port: T, slave: Slave) -> Self {
        Self { port, slave }
    }
}

fn lrc(data: &[u8]) -> u8 {
    data.iter()
        .fold(0u8, |acc, b| acc.wrapping_add(*b))
        .wrapping_neg()
}

fn hex_byte(b: u8) -> String {
    format!("{b:02X}")
}

#[async_trait]
impl<T> Client for AsciiClient<T>
where
    T: AsyncRead + AsyncWrite + Unpin + Send + 'static,
{
    async fn call(&mut self, req: Request<'_>) -> tokio_modbus::Result<Response> {
        let pdu = encode_request_pdu(&req).map_err(protocol_err)?;

        // ADU = slave address + PDU + LRC.
        let mut adu = Vec::with_capacity(pdu.len() + 2);
        adu.push(self.slave.0);
        adu.extend_from_slice(&pdu);
        let checksum = lrc(&adu);
        adu.push(checksum);

        let mut frame = Vec::with_capacity(1 + adu.len() * 2 + 2);
        frame.push(b':');
        for b in &adu {
            frame.extend_from_slice(hex_byte(*b).as_bytes());
        }
        frame.push(b'\r');
        frame.push(b'\n');

        self.port.write_all(&frame).await?;
        self.port.flush().await?;

        // Read the response frame, terminated by '\n'.
        let mut response = Vec::new();
        loop {
            let mut ch = [0u8; 1];
            let n = self.port.read(&mut ch).await?;
            if n == 0 {
                return Err(protocol_err(
                    "serial port closed while reading ASCII response",
                ));
            }
            response.push(ch[0]);
            if ch[0] == b'\n' {
                break;
            }
            if response.len() > 1024 {
                return Err(protocol_err("oversized ASCII response frame"));
            }
        }

        let adu = parse_ascii_frame(&response).map_err(protocol_err)?;
        if adu.len() < 3 {
            return Err(protocol_err("short ASCII response"));
        }

        let resp_slave = adu[0];
        let resp_func = adu[1];
        let payload = &adu[2..adu.len() - 1];
        let received_lrc = adu[adu.len() - 1];

        if lrc(&adu[..adu.len() - 1]) != received_lrc {
            return Err(protocol_err("ASCII LRC checksum mismatch"));
        }
        if resp_slave != self.slave.0 {
            return Err(protocol_err(format!(
                "mismatching slave address (expected {}, got {resp_slave})",
                self.slave.0
            )));
        }
        if resp_func & 0x80 != 0 {
            let code = payload.first().copied().unwrap_or(0);
            return Ok(Err(ExceptionCode::new(code)));
        }
        decode_response_pdu(resp_func, payload)
            .map(Ok)
            .map_err(protocol_err)
    }

    async fn disconnect(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl<T> SlaveContext for AsciiClient<T> {
    fn set_slave(&mut self, slave: Slave) {
        self.slave = slave;
    }
}

fn parse_ascii_frame(frame: &[u8]) -> Result<Vec<u8>, String> {
    let start = frame
        .iter()
        .position(|&b| b == b':')
        .ok_or("no start ':' in ASCII frame")?;
    let mut hex = &frame[start + 1..];
    while hex.last() == Some(&b'\r') || hex.last() == Some(&b'\n') {
        hex = &hex[..hex.len() - 1];
    }
    if hex.is_empty() || !hex.len().is_multiple_of(2) {
        return Err("malformed ASCII frame".into());
    }

    let mut out = Vec::with_capacity(hex.len() / 2);
    for i in (0..hex.len()).step_by(2) {
        let hi = (hex[i] as char).to_digit(16).ok_or("invalid hex digit")? as u8;
        let lo = (hex[i + 1] as char)
            .to_digit(16)
            .ok_or("invalid hex digit")? as u8;
        out.push((hi << 4) | lo);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::borrow::Cow;

    #[test]
    fn read_holding_registers_round_trip() {
        let req = Request::ReadHoldingRegisters(0x006B, 3);
        assert_eq!(
            encode_request_pdu(&req).unwrap(),
            vec![0x03, 0x00, 0x6B, 0x00, 0x03]
        );
        let resp = decode_response_pdu(0x03, &[0x06, 0xAE, 0x41, 0x56, 0x52, 0x43, 0x40]).unwrap();
        assert_eq!(
            resp,
            Response::ReadHoldingRegisters(vec![0xAE41, 0x5652, 0x4340])
        );
    }

    #[test]
    fn write_multiple_registers_round_trip() {
        let req = Request::WriteMultipleRegisters(0x006B, Cow::Borrowed(&[0x00FF, 0x1234][..]));
        assert_eq!(
            encode_request_pdu(&req).unwrap(),
            vec![0x10, 0x00, 0x6B, 0x00, 0x02, 0x04, 0x00, 0xFF, 0x12, 0x34]
        );
        let resp = decode_response_pdu(0x10, &[0x00, 0x6B, 0x00, 0x02]).unwrap();
        assert_eq!(resp, Response::WriteMultipleRegisters(0x006B, 2));
    }

    #[test]
    fn read_coils_round_trip() {
        let req = Request::ReadCoils(0x0013, 19);
        assert_eq!(
            encode_request_pdu(&req).unwrap(),
            vec![0x01, 0x00, 0x13, 0x00, 0x13]
        );
        let resp = decode_response_pdu(0x01, &[0x03, 0xCD, 0x6B, 0x05]).unwrap();
        match resp {
            Response::ReadCoils(bits) => {
                assert_eq!(bits.len(), 24);
                assert!(bits[0] && !bits[1] && bits[2]);
            }
            other => panic!("unexpected {other:?}"),
        }
    }

    #[test]
    fn device_id_decode() {
        let resp = decode_response_pdu(
            0x2B,
            &[
                0x0E, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x03, 0x41, 0x42, 0x43,
            ],
        )
        .unwrap();
        match resp {
            Response::ReadDeviceIdentification(r) => {
                assert_eq!(r.read_code, ReadCode::Basic);
                assert!(!r.more_follows);
                assert_eq!(r.device_id_objects.len(), 1);
                assert_eq!(r.device_id_objects[0].value_as_str(), Some("ABC"));
            }
            other => panic!("unexpected {other:?}"),
        }
    }

    #[test]
    fn encode_additional_requests() {
        use std::borrow::Cow;

        // FC2 / FC4
        assert_eq!(
            encode_request_pdu(&Request::ReadDiscreteInputs(0x006B, 3)).unwrap(),
            vec![0x02, 0x00, 0x6B, 0x00, 0x03]
        );
        assert_eq!(
            encode_request_pdu(&Request::ReadInputRegisters(0x006B, 3)).unwrap(),
            vec![0x04, 0x00, 0x6B, 0x00, 0x03]
        );

        // FC5 (ON / OFF)
        assert_eq!(
            encode_request_pdu(&Request::WriteSingleCoil(0x00AC, true)).unwrap(),
            vec![0x05, 0x00, 0xAC, 0xFF, 0x00]
        );
        assert_eq!(
            encode_request_pdu(&Request::WriteSingleCoil(0x00AC, false)).unwrap(),
            vec![0x05, 0x00, 0xAC, 0x00, 0x00]
        );

        // FC6
        assert_eq!(
            encode_request_pdu(&Request::WriteSingleRegister(0x0001, 0x0003)).unwrap(),
            vec![0x06, 0x00, 0x01, 0x00, 0x03]
        );

        // FC15 (3 coils: on, off, on)
        assert_eq!(
            encode_request_pdu(&Request::WriteMultipleCoils(
                0x0013,
                Cow::Borrowed(&[true, false, true][..])
            ))
            .unwrap(),
            vec![0x0F, 0x00, 0x13, 0x00, 0x03, 0x01, 0x05]
        );

        // FC43 (read device identification)
        assert_eq!(
            encode_request_pdu(&Request::ReadDeviceIdentification(ReadCode::Basic, 0)).unwrap(),
            vec![0x2B, 0x0E, 0x01, 0x00]
        );

        // Custom
        assert_eq!(
            encode_request_pdu(&Request::Custom(0x14, Cow::Borrowed(&[0x07, 0x06][..]))).unwrap(),
            vec![0x14, 0x07, 0x06]
        );
    }

    #[test]
    fn decode_additional_responses() {
        // FC2
        assert_eq!(
            decode_response_pdu(0x02, &[0x01, 0x05]).unwrap(),
            Response::ReadDiscreteInputs(vec![
                true, false, true, false, false, false, false, false
            ])
        );

        // FC4
        assert_eq!(
            decode_response_pdu(0x04, &[0x04, 0x00, 0x0A, 0x00, 0x14]).unwrap(),
            Response::ReadInputRegisters(vec![10, 20])
        );

        // FC5
        assert_eq!(
            decode_response_pdu(0x05, &[0x00, 0xAC, 0xFF, 0x00]).unwrap(),
            Response::WriteSingleCoil(0xAC, true)
        );

        // FC6
        assert_eq!(
            decode_response_pdu(0x06, &[0x00, 0x01, 0x00, 0x03]).unwrap(),
            Response::WriteSingleRegister(0x01, 0x03)
        );

        // FC15
        assert_eq!(
            decode_response_pdu(0x0F, &[0x00, 0x13, 0x00, 0x03]).unwrap(),
            Response::WriteMultipleCoils(0x13, 3)
        );

        // Custom
        assert_eq!(
            decode_response_pdu(0x14, &[0xAA, 0xBB]).unwrap(),
            Response::Custom(0x14, Bytes::from(vec![0xAA, 0xBB]))
        );
    }

    #[test]
    fn encode_unsupported_request_rejected() {
        assert!(encode_request_pdu(&Request::ReportServerId).is_err());
        assert!(encode_request_pdu(&Request::MaskWriteRegister(0, 0, 0)).is_err());
        assert!(
            encode_request_pdu(&Request::ReadWriteMultipleRegisters(
                0,
                1,
                0,
                Cow::Borrowed(&[0x0001][..])
            ))
            .is_err()
        );
    }

    #[test]
    fn decode_response_error_paths() {
        // FC1/2: missing byte count
        assert!(decode_response_pdu(0x01, &[]).is_err());
        // FC1/2: byte count exceeds available data
        assert!(decode_response_pdu(0x01, &[0x05, 0x01]).is_err());
        // FC3/4: missing byte count
        assert!(decode_response_pdu(0x03, &[]).is_err());
        // FC3/4: byte count exceeds available data
        assert!(decode_response_pdu(0x03, &[0x04, 0x00, 0x0A]).is_err());
        // FC5/6/15/16: truncated address/value
        assert!(decode_response_pdu(0x05, &[0x00]).is_err());
        assert!(decode_response_pdu(0x06, &[]).is_err());
        assert!(decode_response_pdu(0x10, &[0x00, 0x01, 0x00]).is_err());
    }

    #[test]
    fn device_id_decode_error_paths() {
        // Unexpected MEI type
        assert!(decode_device_id(&[0x00]).is_err());
        // Missing read code
        assert!(decode_device_id(&[0x0E]).is_err());
        // Invalid read code
        assert!(decode_device_id(&[0x0E, 0xFF, 0x01, 0x00, 0x00, 0x00]).is_err());
        // Invalid conformity level
        assert!(decode_device_id(&[0x0E, 0x01, 0xFF, 0x00, 0x00, 0x00]).is_err());
        // Missing object count
        assert!(decode_device_id(&[0x0E, 0x01, 0x01, 0x00, 0x00]).is_err());
        // Short object id
        assert!(decode_device_id(&[0x0E, 0x01, 0x01, 0x00, 0x00, 0x01]).is_err());
        // Short object length
        assert!(decode_device_id(&[0x0E, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00]).is_err());
        // Short object value
        assert!(decode_device_id(&[0x0E, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x03, 0x41]).is_err());
        // more_follows == 0xFF
        match decode_device_id(&[0x0E, 0x01, 0x01, 0xFF, 0x00, 0x00]).unwrap() {
            Response::ReadDeviceIdentification(r) => assert!(r.more_follows),
            other => panic!("unexpected {other:?}"),
        }
    }

    #[test]
    fn ascii_frame_round_trip() {
        let adu = vec![0x01u8, 0x03, 0x00, 0x00, 0x00, 0x01];
        let checksum = lrc(&adu);
        assert_eq!(checksum, 0xFB);

        let mut frame = vec![b':'];
        for b in &adu {
            frame.extend_from_slice(hex_byte(*b).as_bytes());
        }
        frame.extend_from_slice(hex_byte(checksum).as_bytes());
        frame.extend_from_slice(b"\r\n");

        let mut expected = adu.clone();
        expected.push(checksum);
        assert_eq!(parse_ascii_frame(&frame).unwrap(), expected);
        assert_eq!(hex_byte(0x0F), "0F");
    }

    // ------------------------------------------------------------------
    // RtsStream state machine
    // ------------------------------------------------------------------

    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::{Arc, Mutex};

    #[derive(Clone)]
    struct MockSerial {
        rts: Arc<Mutex<bool>>,
        rts_calls: Arc<Mutex<Vec<bool>>>,
        initial_pending: u32,
        pending_calls: Arc<AtomicUsize>,
        fail_rts: bool,
        fail_pending: bool,
        fail_flush: bool,
    }

    impl MockSerial {
        fn new(initial_pending: u32) -> Self {
            Self {
                rts: Arc::new(Mutex::new(false)),
                rts_calls: Arc::new(Mutex::new(Vec::new())),
                initial_pending,
                pending_calls: Arc::new(AtomicUsize::new(0)),
                fail_rts: false,
                fail_pending: false,
                fail_flush: false,
            }
        }

        fn fail_rts(mut self) -> Self {
            self.fail_rts = true;
            self
        }

        fn fail_pending(mut self) -> Self {
            self.fail_pending = true;
            self
        }

        fn fail_flush(mut self) -> Self {
            self.fail_flush = true;
            self
        }

        fn rts(&self) -> bool {
            *self.rts.lock().unwrap()
        }

        fn rts_calls(&self) -> Vec<bool> {
            self.rts_calls.lock().unwrap().clone()
        }
    }

    impl RtsControl for MockSerial {
        fn set_rts(&mut self, level: bool) -> io::Result<()> {
            if self.fail_rts {
                return Err(io::Error::other("rts failed"));
            }
            *self.rts.lock().unwrap() = level;
            self.rts_calls.lock().unwrap().push(level);
            Ok(())
        }

        fn pending_bytes(&mut self) -> io::Result<u32> {
            if self.fail_pending {
                return Err(io::Error::other("pending failed"));
            }
            // Return `initial_pending` on the first call and 0 afterwards, so
            // a pending byte "drains" after the first poll.
            let call = self.pending_calls.fetch_add(1, Ordering::SeqCst);
            Ok(if call == 0 { self.initial_pending } else { 0 })
        }
    }

    impl AsyncRead for MockSerial {
        fn poll_read(
            self: Pin<&mut Self>,
            _cx: &mut Context<'_>,
            _buf: &mut ReadBuf<'_>,
        ) -> Poll<io::Result<()>> {
            Poll::Pending
        }
    }

    impl AsyncWrite for MockSerial {
        fn poll_write(
            self: Pin<&mut Self>,
            _cx: &mut Context<'_>,
            buf: &[u8],
        ) -> Poll<io::Result<usize>> {
            Poll::Ready(Ok(buf.len()))
        }

        fn poll_flush(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
            if self.fail_flush {
                return Poll::Ready(Err(io::Error::other("flush failed")));
            }
            Poll::Ready(Ok(()))
        }

        fn poll_shutdown(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
            Poll::Ready(Ok(()))
        }
    }

    #[tokio::test]
    async fn rts_asserted_on_write_deasserted_on_flush() {
        let mock = MockSerial::new(0);
        let observer = mock.clone();
        let mut stream = RtsStream::new(mock, Some(Duration::from_millis(1)));

        stream.write_all(b"abc").await.unwrap();
        assert!(observer.rts(), "RTS should be asserted while writing");

        stream.flush().await.unwrap();
        assert!(!observer.rts(), "RTS should be deasserted after flush");
        assert_eq!(observer.rts_calls(), vec![true, false]);
    }

    #[tokio::test]
    async fn rts_untouched_without_delay() {
        let mock = MockSerial::new(0);
        let observer = mock.clone();
        let mut stream = RtsStream::new(mock, None);

        stream.write_all(b"abc").await.unwrap();
        stream.flush().await.unwrap();
        assert!(observer.rts_calls().is_empty(), "no RTS control without -4");
    }

    #[tokio::test]
    async fn rts_deasserted_only_after_drain() {
        let mock = MockSerial::new(1); // one pending byte to drain
        let observer = mock.clone();
        let mut stream = RtsStream::new(mock, Some(Duration::from_millis(1)));

        stream.write_all(b"abc").await.unwrap();
        stream.flush().await.unwrap();

        assert!(!observer.rts(), "RTS should be deasserted after draining");
        assert_eq!(observer.rts_calls(), vec![true, false]);
    }

    #[tokio::test]
    async fn rts_flush_propagates_inner_error() {
        let mock = MockSerial::new(0).fail_flush();
        let mut stream = RtsStream::new(mock, Some(Duration::from_millis(1)));

        stream.write_all(b"abc").await.unwrap();
        assert!(stream.flush().await.is_err());
    }

    #[tokio::test]
    async fn rts_drain_falls_back_on_pending_error() {
        let mock = MockSerial::new(0).fail_pending();
        let observer = mock.clone();
        let mut stream = RtsStream::new(mock, Some(Duration::from_millis(1)));

        stream.write_all(b"abc").await.unwrap();
        stream.flush().await.unwrap();

        assert!(
            !observer.rts(),
            "RTS should be deasserted after the fallback delay"
        );
        assert_eq!(observer.rts_calls(), vec![true, false]);
    }

    #[tokio::test]
    async fn rts_write_propagates_rts_error() {
        let mock = MockSerial::new(0).fail_rts();
        let mut stream = RtsStream::new(mock, Some(Duration::from_millis(1)));

        assert!(stream.write_all(b"abc").await.is_err());
    }

    #[tokio::test]
    async fn rts_shutdown_deasserts_and_shuts_down() {
        let mock = MockSerial::new(0);
        let observer = mock.clone();
        let mut stream = RtsStream::new(mock, Some(Duration::from_millis(1)));

        stream.shutdown().await.unwrap();
        assert!(!observer.rts(), "shutdown should deassert RTS");
    }

    #[tokio::test]
    async fn rts_shutdown_without_delay() {
        let mock = MockSerial::new(0);
        let observer = mock.clone();
        let mut stream = RtsStream::new(mock, None);

        stream.shutdown().await.unwrap();
        assert!(observer.rts_calls().is_empty(), "no RTS control without -4");
    }

    // ------------------------------------------------------------------
    // UDP / ASCII client `call` with a mocked transport
    // ------------------------------------------------------------------

    use std::collections::VecDeque;

    #[derive(Clone)]
    struct MockIo {
        written: Arc<Mutex<Vec<u8>>>,
        response: Arc<Mutex<VecDeque<u8>>>,
    }

    impl MockIo {
        fn new(response: Vec<u8>) -> Self {
            Self {
                written: Arc::new(Mutex::new(Vec::new())),
                response: Arc::new(Mutex::new(response.into())),
            }
        }

        fn written(&self) -> Vec<u8> {
            self.written.lock().unwrap().clone()
        }
    }

    impl AsyncRead for MockIo {
        fn poll_read(
            self: Pin<&mut Self>,
            _cx: &mut Context<'_>,
            buf: &mut ReadBuf<'_>,
        ) -> Poll<io::Result<()>> {
            let mut response = self.response.lock().unwrap();
            let n = response.len().min(buf.remaining());
            let bytes: Vec<u8> = response.drain(..n).collect();
            buf.put_slice(&bytes);
            Poll::Ready(Ok(()))
        }
    }

    impl AsyncWrite for MockIo {
        fn poll_write(
            self: Pin<&mut Self>,
            _cx: &mut Context<'_>,
            buf: &[u8],
        ) -> Poll<io::Result<usize>> {
            self.written.lock().unwrap().extend_from_slice(buf);
            Poll::Ready(Ok(buf.len()))
        }

        fn poll_flush(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
            Poll::Ready(Ok(()))
        }

        fn poll_shutdown(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<io::Result<()>> {
            Poll::Ready(Ok(()))
        }
    }

    #[derive(Clone)]
    struct MockUdp {
        sent: Arc<Mutex<Vec<Vec<u8>>>>,
        response: Arc<Mutex<VecDeque<Vec<u8>>>>,
    }

    impl MockUdp {
        fn new(responses: Vec<Vec<u8>>) -> Self {
            Self {
                sent: Arc::new(Mutex::new(Vec::new())),
                response: Arc::new(Mutex::new(responses.into())),
            }
        }

        fn sent_frames(&self) -> Vec<Vec<u8>> {
            self.sent.lock().unwrap().clone()
        }
    }

    #[async_trait]
    impl UdpTransport for MockUdp {
        async fn send(&mut self, data: &[u8]) -> io::Result<usize> {
            self.sent.lock().unwrap().push(data.to_vec());
            Ok(data.len())
        }

        async fn recv(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            let resp = self.response.lock().unwrap().pop_front().unwrap();
            let n = resp.len().min(buf.len());
            buf[..n].copy_from_slice(&resp[..n]);
            Ok(n)
        }
    }

    #[tokio::test]
    async fn udp_client_call_round_trip() {
        // MBAP response: txn=1, proto=0, len=5, unit=1, FC3 + 1 register (0x1234).
        let response = vec![
            0x00, 0x01, 0x00, 0x00, 0x00, 0x05, 0x01, 0x03, 0x02, 0x12, 0x34,
        ];
        let mock = MockUdp::new(vec![response]);
        let observer = mock.clone();
        let mut client = UdpClient::new(mock, Slave::from(1));

        let resp = client
            .call(Request::ReadHoldingRegisters(0, 1))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(resp, Response::ReadHoldingRegisters(vec![0x1234]));

        let sent = observer.sent_frames();
        assert_eq!(sent.len(), 1);
        assert_eq!(
            sent[0],
            vec![
                0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01
            ]
        );
    }

    #[tokio::test]
    async fn udp_client_rejects_mismatched_transaction() {
        let response = vec![
            0x00, 0x02, 0x00, 0x00, 0x00, 0x05, 0x01, 0x03, 0x02, 0x12, 0x34,
        ];
        let mock = MockUdp::new(vec![response]);
        let mut client = UdpClient::new(mock, Slave::from(1));

        let result = client.call(Request::ReadHoldingRegisters(0, 1)).await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn ascii_client_call_round_trip() {
        // Response frame: slave 1, FC3, 1 register (0x1234), LRC 0xB4.
        let mock = MockIo::new(b":0103021234B4\r\n".to_vec());
        let observer = mock.clone();
        let mut client = AsciiClient::new(mock, Slave::from(1));

        let resp = client
            .call(Request::ReadHoldingRegisters(0, 1))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(resp, Response::ReadHoldingRegisters(vec![0x1234]));

        assert_eq!(observer.written(), b":010300000001FB\r\n".to_vec());
    }

    #[tokio::test]
    async fn udp_client_rejects_short_response() {
        let response = vec![0x00, 0x01, 0x00, 0x00]; // fewer than 8 bytes
        let mock = MockUdp::new(vec![response]);
        let mut client = UdpClient::new(mock, Slave::from(1));

        assert!(
            client
                .call(Request::ReadHoldingRegisters(0, 1))
                .await
                .is_err()
        );
    }

    #[tokio::test]
    async fn ascii_client_closed_port() {
        // An empty response stream makes the first read return 0 (EOF).
        let mock = MockIo::new(vec![]);
        let mut client = AsciiClient::new(mock, Slave::from(1));

        assert!(
            client
                .call(Request::ReadHoldingRegisters(0, 1))
                .await
                .is_err()
        );
    }

    #[tokio::test]
    async fn ascii_client_short_response() {
        let mock = MockIo::new(b":01\r\n".to_vec());
        let mut client = AsciiClient::new(mock, Slave::from(1));

        assert!(
            client
                .call(Request::ReadHoldingRegisters(0, 1))
                .await
                .is_err()
        );
    }

    #[tokio::test]
    async fn ascii_client_lrc_mismatch() {
        let mock = MockIo::new(b":0103021234FF\r\n".to_vec());
        let mut client = AsciiClient::new(mock, Slave::from(1));

        assert!(
            client
                .call(Request::ReadHoldingRegisters(0, 1))
                .await
                .is_err()
        );
    }

    #[tokio::test]
    async fn ascii_client_slave_mismatch() {
        // Valid LRC (0xB3) for slave 2, but the client expects slave 1.
        let mock = MockIo::new(b":0203021234B3\r\n".to_vec());
        let mut client = AsciiClient::new(mock, Slave::from(1));

        assert!(
            client
                .call(Request::ReadHoldingRegisters(0, 1))
                .await
                .is_err()
        );
    }

    #[tokio::test]
    async fn ascii_client_exception_response() {
        // Slave 1, FC3 | 0x80 = 0x83, exception code 1 (illegal function).
        let mock = MockIo::new(b":0183017B\r\n".to_vec());
        let mut client = AsciiClient::new(mock, Slave::from(1));

        let result = client.call(Request::ReadHoldingRegisters(0, 1)).await;
        assert!(result.unwrap().is_err());
    }
}
