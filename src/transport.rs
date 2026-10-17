//! Additional Modbus transports not provided by `tokio-modbus`: Modbus/UDP
//! and Modbus ASCII. Both are implemented as `tokio_modbus::client::Client`s so
//! the rest of the program can use the standard `Reader`/`Writer` helpers.
//!
//! Modbus RTU-over-TCP ("enc") does not need a custom client: it is just the
//! RTU codec attached to a `TcpStream` (`tokio_modbus::client::rtu::attach_slave`).

use std::io;

use async_trait::async_trait;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

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
    Error::Transport(io::Error::new(io::ErrorKind::Other, msg.into()))
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
            let byte_count = (count + 7) / 8;
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

/// A Modbus-over-UDP client using the same MBAP header as Modbus TCP.
pub struct UdpClient {
    socket: tokio::net::UdpSocket,
    slave: Slave,
    transaction: u16,
}

impl UdpClient {
    pub async fn new(addr: std::net::SocketAddr, slave: Slave) -> io::Result<Self> {
        let socket = tokio::net::UdpSocket::bind("0.0.0.0:0").await?;
        socket.connect(addr).await?;
        Ok(Self {
            socket,
            slave,
            transaction: 1,
        })
    }
}

#[async_trait]
impl Client for UdpClient {
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

        self.socket.send(&frame).await?;

        let mut buf = [0u8; 260];
        let n = self.socket.recv(&mut buf).await?;
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

impl SlaveContext for UdpClient {
    fn set_slave(&mut self, slave: Slave) {
        self.slave = slave;
    }
}

// ---------------------------------------------------------------------------
// Modbus ASCII client
// ---------------------------------------------------------------------------

/// A Modbus ASCII client. The PDU is framed with `:` / CRLF and an LRC checksum
/// and encoded as hexadecimal characters over a serial line.
pub struct AsciiClient {
    port: tokio_serial::SerialStream,
    slave: Slave,
}

impl AsciiClient {
    pub fn new(port: tokio_serial::SerialStream, slave: Slave) -> Self {
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
impl Client for AsciiClient {
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

impl SlaveContext for AsciiClient {
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
    if hex.is_empty() || hex.len() % 2 != 0 {
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
}
