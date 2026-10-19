mod datatype;
mod transport;

use std::future::Future;
use std::net::SocketAddr;
use std::time::Duration;

use anyhow::{Result, anyhow, bail};
use clap::{Parser, ValueEnum};
use tokio_modbus::client::{Context, rtu, tcp};
use tokio_modbus::prelude::{
    Client, ReadCode, ReadDeviceIdentificationResponse, Reader, Request, Response, Slave, Writer,
};

use crate::datatype::{DataType, ValueOptions, encode_write_words, format_registers, parse_bits};
use crate::transport::{AsciiClient, RtsSerialStream, UdpClient, push_u16};

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum Protocol {
    Ascii,
    Rtu,
    Tcp,
    Udp,
    Enc,
}

impl Protocol {
    fn is_serial(self) -> bool {
        matches!(self, Protocol::Rtu | Protocol::Ascii)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ParityArg {
    None,
    Even,
    Odd,
}

#[derive(Parser)]
#[command(
    name = "rmodpoll",
    version,
    about = "Modbus master poller for testing device communication"
)]
struct Cli {
    /// Serial port (Modbus ASCII/RTU) or host name / dotted IP address (TCP/UDP/enc)
    #[arg(value_name = "SERIALPORT|HOST")]
    target: String,

    /// List of values to write; if omitted, data is read instead
    #[arg(value_name = "WRITEVALUES")]
    write_values: Vec<String>,

    /// Protocol: ascii, rtu, tcp, udp or enc (auto-detected if omitted)
    #[arg(short = 'm', value_name = "PROTOCOL")]
    protocol: Option<Protocol>,

    /// Slave address (1-247 serial, 0-255 TCP, default 1)
    #[arg(short = 'a', value_name = "ADDR", default_value_t = 1)]
    slave: u8,

    /// Start reference (1-65536, default 1; 0-based with -0)
    #[arg(short = 'r', value_name = "REF", default_value_t = 1)]
    start_ref: u32,

    /// Number of values to read (1-125, default 1); for writing use -c 1 to force FC5/FC6
    #[arg(short = 'c', value_name = "COUNT")]
    count: Option<u16>,

    /// Data type: 0, 1, 3, 3:hex, 3:i32, 3:i64, 3:mod, 3:f32, 3:f64,
    /// 4, 4:hex, 4:i32, 4:i64, 4:mod, 4:f32, 4:f64, id or file
    #[arg(short = 't', value_name = "TYPE", default_value = "4")]
    data_type: String,

    /// File number for file records (default 4)
    #[arg(short = 'n', value_name = "FILE", default_value_t = 4)]
    file_number: u16,

    /// Display data as unsigned integers
    #[arg(short = 'u')]
    unsigned: bool,

    /// Slave operates on big-endian 32/64-bit integers
    #[arg(short = 'i')]
    big_int: bool,

    /// Slave operates on big-endian 32/64-bit floats
    #[arg(short = 'f')]
    big_float: bool,

    /// First reference is 0 (PDU addressing) instead of 1
    #[arg(short = '0')]
    zero_based: bool,

    /// Poll only once
    #[arg(short = '1')]
    poll_once: bool,

    /// Poll rate in ms (default 1000)
    #[arg(short = 'l', value_name = "MS", default_value_t = 1000)]
    poll_rate_ms: u64,

    /// Timeout in seconds (0.01 - 10.0, default 1.0)
    #[arg(short = 'o', value_name = "SEC", default_value_t = 1.0)]
    timeout: f64,

    /// IP port (TCP/UDP/enc) or parity (serial: none, even, odd)
    #[arg(short = 'p', value_name = "PORT|PARITY")]
    port_parity: Option<String>,

    /// Baudrate (default 19200)
    #[arg(short = 'b', value_name = "BAUD", default_value_t = 19200)]
    baud: u32,

    /// Data bits (7 or 8 for ASCII, 8 for RTU)
    #[arg(short = 'd', value_name = "BITS")]
    data_bits: Option<u8>,

    /// Stop bits (1 or 2, default 1)
    #[arg(short = 's', value_name = "BITS", default_value_t = 1)]
    stop_bits: u8,

    /// RS-485 mode: RTS on while transmitting and N ms after
    #[arg(short = '4', value_name = "MS")]
    rts_delay_ms: Option<u32>,
}

struct Config {
    protocol: Protocol,
    target: String,
    write_values: Vec<String>,
    slave: u8,
    start_ref: u32,
    zero_based: bool,
    count: Option<u16>,
    data_type: DataType,
    file_number: u16,
    value_opts: ValueOptions,
    poll_once: bool,
    poll_rate: Duration,
    timeout: Duration,
    port: u16,
    baud: u32,
    data_bits: Option<u8>,
    stop_bits: u8,
    parity: ParityArg,
    rts_delay_ms: Option<u32>,
}

impl Config {
    fn build(cli: Cli) -> Result<Self> {
        let protocol = cli.protocol.unwrap_or_else(|| detect_protocol(&cli.target));
        let data_type = DataType::parse(&cli.data_type)?;

        // The -p flag is overloaded: it is a parity for serial protocols and a
        // port number for the network protocols.
        let (port, parity) = match protocol {
            Protocol::Rtu | Protocol::Ascii => {
                let pplow = cli.port_parity.as_deref().map(str::to_ascii_lowercase);
                let parity = match pplow.as_deref() {
                    None | Some("even") => ParityArg::Even,
                    Some("none") => ParityArg::None,
                    Some("odd") => ParityArg::Odd,
                    Some(other) => bail!("invalid parity '{other}' (expected none, even or odd)"),
                };
                (502, parity)
            }
            _ => {
                let port = match cli.port_parity.as_deref() {
                    None => 502,
                    Some(s) => s
                        .parse::<u16>()
                        .map_err(|_| anyhow!("invalid IP port '{s}'"))?,
                };
                (port, ParityArg::Even)
            }
        };

        // Slave address range depends on the transport.
        match protocol {
            Protocol::Rtu | Protocol::Ascii | Protocol::Enc => {
                if !(1..=247).contains(&cli.slave) {
                    bail!("slave address must be 1-247 for serial protocols");
                }
            }
            Protocol::Tcp | Protocol::Udp => {}
        }

        // Start reference / record number validation.
        if data_type == DataType::File {
            if cli.start_ref > 65_535 {
                bail!("record number must be 0-65535 for file records");
            }
        } else if cli.zero_based {
            if cli.start_ref > 65_535 {
                bail!("start reference must be 0-65535 with 0-based addressing (-0)");
            }
        } else if !(1..=65_536).contains(&cli.start_ref) {
            bail!("start reference must be 1-65536");
        }

        if let Some(c) = cli.count
            && !(1..=125).contains(&c)
        {
            bail!("count must be 1-125");
        }

        if data_type.is_register() {
            let value_count = usize::from(cli.count.unwrap_or(1));
            let register_count = value_count * data_type.words_per_value();
            if register_count > 125 {
                bail!(
                    "{} value(s) of a {}-word type require {} registers, exceeding the limit of 125",
                    value_count,
                    data_type.words_per_value(),
                    register_count
                );
            }
        }

        if !(0.01..=10.0).contains(&cli.timeout) {
            bail!("timeout must be 0.01-10.0 seconds");
        }

        if let Some(d) = cli.data_bits {
            if d != 7 && d != 8 {
                bail!("data bits must be 7 or 8");
            }
            if protocol == Protocol::Rtu && d != 8 {
                bail!("Modbus RTU requires 8 data bits");
            }
        }
        if cli.stop_bits != 1 && cli.stop_bits != 2 {
            bail!("stop bits must be 1 or 2");
        }

        if cli.rts_delay_ms.is_some() && !protocol.is_serial() {
            bail!("RS-485 mode (-4) is only valid with Modbus ASCII or RTU");
        }

        if !cli.write_values.is_empty() && !data_type.is_writable() {
            bail!("data type {} cannot be written", data_type.describe());
        }

        let big_int = cli.big_int;
        let big_float = cli.big_float;

        Ok(Self {
            protocol,
            target: cli.target,
            write_values: cli.write_values,
            slave: cli.slave,
            start_ref: cli.start_ref,
            zero_based: cli.zero_based,
            count: cli.count,
            data_type,
            file_number: cli.file_number,
            value_opts: ValueOptions {
                unsigned: cli.unsigned,
                big_endian_int: big_int,
                big_endian_float: big_float,
            },
            poll_once: cli.poll_once,
            poll_rate: Duration::from_millis(cli.poll_rate_ms),
            timeout: Duration::from_secs_f64(cli.timeout),
            port,
            baud: cli.baud,
            data_bits: cli.data_bits,
            stop_bits: cli.stop_bits,
            parity,
            rts_delay_ms: cli.rts_delay_ms,
        })
    }

    fn pdu_addr(&self) -> u16 {
        if self.zero_based {
            self.start_ref as u16
        } else {
            (self.start_ref - 1) as u16
        }
    }

    fn record_number(&self) -> u16 {
        self.start_ref as u16
    }

    fn value_count(&self) -> u16 {
        self.count.unwrap_or(1)
    }

    /// `-c 1` forces the single-write function codes (FC5/FC6).
    fn force_single(&self) -> bool {
        self.count == Some(1)
    }

    fn register_count(&self) -> u16 {
        self.value_count() * self.data_type.words_per_value() as u16
    }
}

fn detect_protocol(target: &str) -> Protocol {
    let upper = target.to_ascii_uppercase();
    if target.contains('\\') || upper.contains("COM") || target.starts_with("/dev/") {
        Protocol::Rtu
    } else {
        Protocol::Tcp
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();
    let cfg = Config::build(cli)?;
    print_header(&cfg);
    run(&cfg).await
}

fn print_header(cfg: &Config) {
    let version = env!("CARGO_PKG_VERSION");
    println!("rmodpoll {version} - Modbus master poller");
    println!("Protocol: {}", protocol_name(cfg.protocol));

    let target = match cfg.protocol {
        Protocol::Tcp | Protocol::Udp | Protocol::Enc => format!("{}:{}", cfg.target, cfg.port),
        _ => cfg.target.clone(),
    };
    println!(
        "Target: {target}, slave {}, t/o {:.2} s, poll rate {} ms",
        cfg.slave,
        cfg.timeout.as_secs_f64(),
        cfg.poll_rate.as_millis()
    );
    println!("Data type: {}", cfg.data_type.describe());
    println!(
        "Reference: {} ({}-based), count {}",
        cfg.start_ref,
        if cfg.zero_based { "0" } else { "1" },
        cfg.value_count()
    );

    if let Some(ms) = cfg.rts_delay_ms {
        println!("RS-485 mode: RTS during transmit +{ms} ms after");
    }

    println!();
}

fn protocol_name(p: Protocol) -> &'static str {
    match p {
        Protocol::Ascii => "Modbus ASCII",
        Protocol::Rtu => "Modbus RTU",
        Protocol::Tcp => "Modbus TCP",
        Protocol::Udp => "Modbus UDP",
        Protocol::Enc => "Modbus RTU over TCP (encapsulated)",
    }
}

async fn connect(cfg: &Config) -> Result<Context> {
    let slave = Slave::from(cfg.slave);
    Ok(match cfg.protocol {
        Protocol::Tcp => {
            let addr = resolve(&cfg.target, cfg.port).await?;
            tcp::connect_slave(addr, slave).await?
        }
        Protocol::Udp => {
            let addr = resolve(&cfg.target, cfg.port).await?;
            Context::from(Box::new(UdpClient::connect(addr, slave).await?) as Box<dyn Client>)
        }
        Protocol::Enc => {
            let addr = resolve(&cfg.target, cfg.port).await?;
            let stream = tokio::net::TcpStream::connect(addr).await?;
            rtu::attach_slave(stream, slave)
        }
        Protocol::Rtu => {
            let port = open_serial(&cfg.target, cfg, 8)?;
            rtu::attach_slave(port, slave)
        }
        Protocol::Ascii => {
            let port = open_serial(&cfg.target, cfg, 7)?;
            Context::from(Box::new(AsciiClient::new(port, slave)) as Box<dyn Client>)
        }
    })
}

async fn resolve(host: &str, port: u16) -> Result<SocketAddr> {
    let mut addrs = tokio::net::lookup_host((host, port)).await?;
    addrs
        .next()
        .ok_or_else(|| anyhow!("could not resolve host '{host}'"))
}

fn open_serial(path: &str, cfg: &Config, default_data_bits: u8) -> Result<RtsSerialStream> {
    use tokio_serial::{DataBits, FlowControl, Parity, SerialStream, StopBits};

    let data_bits = match cfg.data_bits.unwrap_or(default_data_bits) {
        7 => DataBits::Seven,
        8 => DataBits::Eight,
        _ => unreachable!("data bits validated during argument parsing"),
    };
    let stop_bits = if cfg.stop_bits == 2 {
        StopBits::Two
    } else {
        StopBits::One
    };
    let parity = match cfg.parity {
        ParityArg::None => Parity::None,
        ParityArg::Even => Parity::Even,
        ParityArg::Odd => Parity::Odd,
    };

    let builder = tokio_serial::new(path, cfg.baud)
        .data_bits(data_bits)
        .stop_bits(stop_bits)
        .parity(parity)
        .flow_control(FlowControl::None);

    let port = SerialStream::open(&builder)?;
    let rts_delay = cfg
        .rts_delay_ms
        .map(|ms| Duration::from_millis(u64::from(ms)));
    Ok(RtsSerialStream::new(port, rts_delay))
}

/// Wraps a Modbus operation with the configured timeout and flattens the
/// nested `Result<Result<T, ExceptionCode>, Error>` into an `anyhow::Result`.
async fn timed<F, T>(timeout: Duration, fut: F) -> Result<T>
where
    F: Future<Output = tokio_modbus::Result<T>>,
{
    match tokio::time::timeout(timeout, fut).await {
        Err(_) => Err(anyhow!(
            "operation timed out after {:.2} s",
            timeout.as_secs_f64()
        )),
        Ok(Err(err)) => Err(anyhow!("transport/protocol error: {err}")),
        Ok(Ok(Err(exc))) => Err(anyhow!("modbus exception: {exc}")),
        Ok(Ok(Ok(val))) => Ok(val),
    }
}

async fn run(cfg: &Config) -> Result<()> {
    if cfg.write_values.is_empty() {
        run_read(cfg).await
    } else {
        run_write(cfg).await
    }
}

/// Write mode mirrors `modpoll`: perform a single write and exit.
async fn run_write(cfg: &Config) -> Result<()> {
    let mut ctx = connect(cfg).await?;
    let result = do_write(&mut ctx, cfg).await;
    disconnect(ctx).await;
    result
}

async fn run_read(cfg: &Config) -> Result<()> {
    let is_serial = cfg.protocol.is_serial();
    let mut ctx: Option<Context> = None;

    loop {
        // Establish a connection when none is open. Serial ports are opened
        // once and kept for the whole run; a network connection is re-opened
        // only after a failed transaction.
        if ctx.is_none() {
            match connect(cfg).await {
                Ok(connection) => ctx = Some(connection),
                Err(e) => {
                    let msg = format!("connection failed: {e:#}");
                    if cfg.poll_once || is_serial {
                        return Err(anyhow!(msg));
                    }
                    eprintln!("Error: {msg}");
                    tokio::time::sleep(cfg.poll_rate).await;
                    continue;
                }
            }
        }

        match do_read(ctx.as_mut().expect("connection established above"), cfg).await {
            Ok(()) => {}
            Err(e) => {
                eprintln!("Error: {e:#}");
                if cfg.poll_once {
                    return Err(e);
                }
                // A failed transaction may have left a network connection in a
                // broken state, so drop it and reconnect on the next cycle. A
                // serial port stays open (a timeout does not close it).
                if !is_serial && let Some(connection) = ctx.take() {
                    disconnect(connection).await;
                }
            }
        }

        if cfg.poll_once {
            break;
        }
        println!();
        tokio::time::sleep(cfg.poll_rate).await;
    }

    if let Some(connection) = ctx {
        disconnect(connection).await;
    }
    Ok(())
}

/// Gracefully shuts down a client context, ignoring any error.
async fn disconnect(mut connection: Context) {
    let _ = connection.disconnect().await;
}

async fn do_read(ctx: &mut Context, cfg: &Config) -> Result<()> {
    match cfg.data_type {
        DataType::Coil => {
            let bits = timed(
                cfg.timeout,
                ctx.read_coils(cfg.pdu_addr(), cfg.value_count()),
            )
            .await?;
            print_bits(cfg.pdu_addr(), cfg.zero_based, &bits);
        }
        DataType::DiscreteInput => {
            let bits = timed(
                cfg.timeout,
                ctx.read_discrete_inputs(cfg.pdu_addr(), cfg.value_count()),
            )
            .await?;
            print_bits(cfg.pdu_addr(), cfg.zero_based, &bits);
        }
        dt if dt.is_input_register() => {
            let words = timed(
                cfg.timeout,
                ctx.read_input_registers(cfg.pdu_addr(), cfg.register_count()),
            )
            .await?;
            print_registers(cfg.pdu_addr(), cfg.zero_based, dt, &cfg.value_opts, &words)?;
        }
        dt if dt.is_holding_register() => {
            let words = timed(
                cfg.timeout,
                ctx.read_holding_registers(cfg.pdu_addr(), cfg.register_count()),
            )
            .await?;
            print_registers(cfg.pdu_addr(), cfg.zero_based, dt, &cfg.value_opts, &words)?;
        }
        DataType::DeviceId => {
            let resp = timed(
                cfg.timeout,
                ctx.read_device_identification(ReadCode::Extended, 0),
            )
            .await?;
            print_device_id(&resp);
        }
        DataType::File => {
            let words = read_file(ctx, cfg).await?;
            print_file_words(&words);
        }
        // The guard arms above cover every register variant; this is only
        // here to satisfy exhaustiveness checking.
        _ => unreachable!("all data types handled"),
    }
    Ok(())
}

async fn do_write(ctx: &mut Context, cfg: &Config) -> Result<()> {
    match cfg.data_type {
        DataType::Coil => {
            let bits = parse_bits(&cfg.write_values)?;
            if bits.len() == 1 && cfg.force_single() {
                timed(cfg.timeout, ctx.write_single_coil(cfg.pdu_addr(), bits[0])).await?;
            } else {
                timed(cfg.timeout, ctx.write_multiple_coils(cfg.pdu_addr(), &bits)).await?;
            }
            println!(
                "Wrote {} coil(s) at reference {}",
                bits.len(),
                cfg.start_ref
            );
        }
        dt if dt.is_holding_register() => {
            let words = encode_write_words(dt, &cfg.write_values, &cfg.value_opts)?;
            if words.len() == 1 && cfg.force_single() {
                timed(
                    cfg.timeout,
                    ctx.write_single_register(cfg.pdu_addr(), words[0]),
                )
                .await?;
            } else {
                timed(
                    cfg.timeout,
                    ctx.write_multiple_registers(cfg.pdu_addr(), &words),
                )
                .await?;
            }
            println!(
                "Wrote {} register(s) at reference {}",
                words.len(),
                cfg.start_ref
            );
        }
        DataType::File => {
            write_file(ctx, cfg).await?;
            println!(
                "Wrote file record (file {}, record {})",
                cfg.file_number,
                cfg.record_number()
            );
        }
        _ => bail!("data type {:?} cannot be written", cfg.data_type),
    }
    Ok(())
}

fn print_bits(addr: u16, zero_based: bool, bits: &[bool]) {
    for (i, bit) in bits.iter().enumerate() {
        let reference = if zero_based {
            u32::from(addr) + i as u32
        } else {
            u32::from(addr) + i as u32 + 1
        };
        println!("[{reference}]: {}", u8::from(*bit));
    }
}

fn print_registers(
    addr: u16,
    zero_based: bool,
    dt: DataType,
    opts: &ValueOptions,
    words: &[u16],
) -> Result<()> {
    let values = format_registers(words, dt, opts)?;
    let wpv = dt.words_per_value();
    for (i, value) in values.iter().enumerate() {
        let reference = if zero_based {
            u32::from(addr) + (i * wpv) as u32
        } else {
            u32::from(addr) + (i * wpv) as u32 + 1
        };
        println!("[{reference}]: {value}");
    }
    Ok(())
}

fn print_device_id(resp: &ReadDeviceIdentificationResponse) {
    for obj in &resp.device_id_objects {
        let raw = obj.value.as_ref();
        // `modpoll` treats the value as a C string: it stops displaying at the
        // first NUL byte, while still reporting the full byte length.
        let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
        let value = String::from_utf8_lossy(&raw[..end]);
        println!("[{}]: '{value}', len = {}", obj.id, obj.value.len());
    }
}

fn print_file_words(words: &[u16]) {
    for (i, word) in words.iter().enumerate() {
        println!("[{}]: {word}", i + 1);
    }
}

/// Builds the FC20 (Read File Record) request PDU payload, preceded by its
/// byte count: reference type (1) + file number (2) + record number (2) +
/// record length (2) = 7 bytes.
fn file_read_request(file_number: u16, record_number: u16, record_length: u16) -> Vec<u8> {
    let mut data = vec![0x07, 0x06];
    push_u16(&mut data, file_number);
    push_u16(&mut data, record_number);
    push_u16(&mut data, record_length);
    data
}

async fn read_file(ctx: &mut Context, cfg: &Config) -> Result<Vec<u16>> {
    let data = file_read_request(cfg.file_number, cfg.record_number(), cfg.value_count());
    let req = Request::Custom(0x14, std::borrow::Cow::Borrowed(&data));
    let resp = timed(cfg.timeout, ctx.call(req)).await?;

    match resp {
        Response::Custom(0x14, bytes) => parse_file_read_response(bytes.as_ref()),
        Response::Custom(fc, _) => Err(anyhow!(
            "unexpected function code 0x{fc:02X} in file read response"
        )),
        _ => Err(anyhow!("unexpected response to file read")),
    }
}

/// Builds the FC21 (Write File Record) request PDU payload, preceded by its
/// byte count: reference type (1) + file number (2) + record number (2) +
/// record length (2) + record data (2N).
fn file_write_request(file_number: u16, record_number: u16, words: &[u16]) -> Vec<u8> {
    let mut data = Vec::with_capacity(8 + 2 * words.len());
    data.push((7 + 2 * words.len()) as u8);
    data.push(0x06);
    push_u16(&mut data, file_number);
    push_u16(&mut data, record_number);
    push_u16(&mut data, words.len() as u16);
    for word in words {
        push_u16(&mut data, *word);
    }
    data
}

async fn write_file(ctx: &mut Context, cfg: &Config) -> Result<()> {
    let words = encode_write_words(DataType::HoldingReg16, &cfg.write_values, &cfg.value_opts)?;
    let data = file_write_request(cfg.file_number, cfg.record_number(), &words);
    let req = Request::Custom(0x15, std::borrow::Cow::Borrowed(&data));
    let resp = timed(cfg.timeout, ctx.call(req)).await?;

    match resp {
        Response::Custom(0x15, _) => Ok(()),
        Response::Custom(fc, _) => Err(anyhow!(
            "unexpected function code 0x{fc:02X} in file write response"
        )),
        _ => Err(anyhow!("unexpected response to file write")),
    }
}

fn parse_file_read_response(bytes: &[u8]) -> Result<Vec<u16>> {
    if bytes.len() < 3 {
        bail!("short file read response");
    }
    let byte_count = usize::from(bytes[0]);
    if bytes.len() < 1 + byte_count {
        bail!("truncated file read response");
    }
    let sub_len = usize::from(bytes[1]);
    if sub_len < 1 || bytes.len() < 2 + sub_len {
        bail!("malformed file read sub-response");
    }
    let reference_type = bytes[2];
    if reference_type != 0x06 {
        bail!("unexpected reference type 0x{reference_type:02X}");
    }

    let data = &bytes[3..2 + sub_len];
    // File record data is a sequence of 16-bit words, so it must have an even
    // byte length. Reject odd-length data instead of panicking on the pair
    // access below when the peer sends a malformed frame.
    if !data.len().is_multiple_of(2) {
        bail!("malformed file read sub-response: odd data length");
    }
    let mut words = Vec::with_capacity(data.len() / 2);
    for i in (0..data.len()).step_by(2) {
        words.push(u16::from_be_bytes([data[i], data[i + 1]]));
    }
    Ok(words)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cli(target: &str) -> Cli {
        Cli {
            target: target.to_string(),
            write_values: vec![],
            protocol: None,
            slave: 1,
            start_ref: 1,
            count: None,
            data_type: "4".to_string(),
            file_number: 4,
            unsigned: false,
            big_int: false,
            big_float: false,
            zero_based: false,
            poll_once: false,
            poll_rate_ms: 1000,
            timeout: 1.0,
            port_parity: None,
            baud: 19200,
            data_bits: None,
            stop_bits: 1,
            rts_delay_ms: None,
        }
    }

    #[test]
    fn detect_protocol_defaults() {
        assert_eq!(detect_protocol("COM1"), Protocol::Rtu);
        assert_eq!(detect_protocol(r"\\.\COM5"), Protocol::Rtu);
        assert_eq!(detect_protocol("/dev/ttyUSB0"), Protocol::Rtu);
        assert_eq!(detect_protocol("192.168.1.10"), Protocol::Tcp);
        assert_eq!(detect_protocol("localhost"), Protocol::Tcp);
    }

    #[test]
    fn valid_default_config() {
        let cfg = Config::build(cli("192.168.1.10")).unwrap();
        assert_eq!(cfg.protocol, Protocol::Tcp);
        assert_eq!(cfg.slave, 1);
        assert_eq!(cfg.data_type, DataType::HoldingReg16);
        assert_eq!(cfg.pdu_addr(), 0);
        assert_eq!(cfg.port, 502);
    }

    #[test]
    fn invalid_parity_rejected() {
        let mut c = cli("COM1");
        c.protocol = Some(Protocol::Rtu);
        c.port_parity = Some("foo".to_string());
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn invalid_slave_for_serial_rejected() {
        let mut c = cli("COM1");
        c.protocol = Some(Protocol::Rtu);
        c.slave = 0;
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn invalid_start_ref_rejected() {
        let mut c = cli("192.168.1.10");
        c.start_ref = 0; // requires -0
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn zero_based_start_ref() {
        let mut c = cli("192.168.1.10");
        c.start_ref = 0;
        c.zero_based = true;
        let cfg = Config::build(c).unwrap();
        assert_eq!(cfg.pdu_addr(), 0);
    }

    #[test]
    fn invalid_count_rejected() {
        let mut c = cli("192.168.1.10");
        c.count = Some(126);
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn register_limit_enforced() {
        let mut c = cli("192.168.1.10");
        c.data_type = "4:i64".to_string();
        c.count = Some(32); // 32 * 4 = 128 registers > 125
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn invalid_timeout_rejected() {
        let mut c = cli("192.168.1.10");
        c.timeout = 11.0;
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn rtu_requires_8_data_bits() {
        let mut c = cli("COM1");
        c.protocol = Some(Protocol::Rtu);
        c.data_bits = Some(7);
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn rs485_only_for_serial() {
        let mut c = cli("192.168.1.10");
        c.rts_delay_ms = Some(10);
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn readonly_type_rejects_write() {
        let mut c = cli("192.168.1.10");
        c.data_type = "3".to_string();
        c.write_values = vec!["1".to_string()];
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn file_read_request_encoding() {
        assert_eq!(
            file_read_request(4, 0, 2),
            vec![0x07, 0x06, 0x00, 0x04, 0x00, 0x00, 0x00, 0x02]
        );
    }

    #[test]
    fn file_write_request_encoding() {
        assert_eq!(
            file_write_request(4, 0, &[100, 200]),
            vec![
                0x0B, 0x06, 0x00, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x64, 0x00, 0xC8
            ]
        );
    }

    #[test]
    fn file_read_response_parsing() {
        let words = parse_file_read_response(&[0x06, 0x05, 0x06, 0x00, 0x64, 0x00, 0xC8]).unwrap();
        assert_eq!(words, vec![100, 200]);
    }

    // ------------------------------------------------------------------
    // Integration tests with an in-process Modbus TCP server
    // ------------------------------------------------------------------

    use std::io;
    use std::net::SocketAddr;
    use std::sync::{Arc, Mutex};
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    use tokio::net::TcpListener;

    #[derive(Clone)]
    struct Store(Arc<Mutex<Vec<u16>>>);

    impl Store {
        fn get(&self, addr: usize) -> u16 {
            self.0.lock().unwrap().get(addr).copied().unwrap_or(0)
        }

        fn set(&self, addr: usize, value: u16) {
            let mut regs = self.0.lock().unwrap();
            if regs.len() <= addr {
                regs.resize(addr + 1, 0);
            }
            regs[addr] = value;
        }
    }

    /// Starts a Modbus TCP server serving the given initial registers and
    /// returns its bound address together with the shared register store.
    async fn spawn_server(initial: Vec<u16>) -> (SocketAddr, Store) {
        let store = Store(Arc::new(Mutex::new(initial)));
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();

        let server_store = store.clone();
        tokio::spawn(async move {
            loop {
                let (socket, _) = match listener.accept().await {
                    Ok(conn) => conn,
                    Err(_) => break,
                };
                let store = server_store.clone();
                tokio::spawn(async move {
                    let _ = serve_connection(socket, &store).await;
                });
            }
        });

        (addr, store)
    }

    /// Starts a server that accepts connections but never responds.
    async fn spawn_silent_server() -> SocketAddr {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            loop {
                let (_socket, _) = match listener.accept().await {
                    Ok(conn) => conn,
                    Err(_) => break,
                };
                tokio::spawn(std::future::pending::<()>());
            }
        });
        addr
    }

    async fn serve_connection(mut socket: tokio::net::TcpStream, store: &Store) -> io::Result<()> {
        loop {
            let mut header = [0u8; 7];
            if socket.read_exact(&mut header).await.is_err() {
                return Ok(()); // client closed the connection
            }
            let txn = [header[0], header[1]];
            let unit = header[6];
            let len = u16::from_be_bytes([header[4], header[5]]) as usize;
            if len == 0 {
                continue;
            }
            let mut pdu = vec![0u8; len - 1];
            if socket.read_exact(&mut pdu).await.is_err() {
                return Ok(());
            }

            let Some(resp) = build_response(&pdu, store) else {
                return Ok(());
            };

            let mut out = Vec::with_capacity(7 + resp.len());
            out.extend_from_slice(&txn);
            out.extend_from_slice(&[0, 0]);
            out.extend_from_slice(&((resp.len() + 1) as u16).to_be_bytes());
            out.push(unit);
            out.extend_from_slice(&resp);
            if socket.write_all(&out).await.is_err() {
                return Ok(());
            }
        }
    }

    fn build_response(pdu: &[u8], store: &Store) -> Option<Vec<u8>> {
        let func = *pdu.first()?;
        match func {
            0x03 | 0x04 => {
                if pdu.len() < 5 {
                    return None;
                }
                let addr = u16::from_be_bytes([pdu[1], pdu[2]]) as usize;
                let qty = u16::from_be_bytes([pdu[3], pdu[4]]) as usize;
                let mut r = vec![func, (qty * 2) as u8];
                for i in 0..qty {
                    let v = store.get(addr + i);
                    r.push((v >> 8) as u8);
                    r.push(v as u8);
                }
                Some(r)
            }
            0x06 => {
                if pdu.len() < 5 {
                    return None;
                }
                let addr = u16::from_be_bytes([pdu[1], pdu[2]]) as usize;
                let value = u16::from_be_bytes([pdu[3], pdu[4]]);
                store.set(addr, value);
                Some(vec![0x06, pdu[1], pdu[2], pdu[3], pdu[4]])
            }
            0x10 => {
                if pdu.len() < 6 {
                    return None;
                }
                let addr = u16::from_be_bytes([pdu[1], pdu[2]]) as usize;
                let qty = u16::from_be_bytes([pdu[3], pdu[4]]) as usize;
                for i in 0..qty {
                    let value = u16::from_be_bytes([pdu[6 + 2 * i], pdu[7 + 2 * i]]);
                    store.set(addr + i, value);
                }
                Some(vec![0x10, pdu[1], pdu[2], pdu[3], pdu[4]])
            }
            0x01 | 0x02 => {
                if pdu.len() < 5 {
                    return None;
                }
                let qty = u16::from_be_bytes([pdu[3], pdu[4]]) as usize;
                let byte_count = qty.div_ceil(8);
                let mut r = vec![func, byte_count as u8];
                r.extend(std::iter::repeat_n(0u8, byte_count));
                Some(r)
            }
            0x05 => Some(vec![0x05, pdu[1], pdu[2], pdu[3], pdu[4]]),
            0x0F => Some(vec![0x0F, pdu[1], pdu[2], pdu[3], pdu[4]]),
            0x2B => {
                // FC43 read device identification: [0x2B, 0x0E, read_code, object_id].
                // Respond with one object: vendor name (id 0) = "ACME".
                Some(vec![
                    0x2B, 0x0E, 0x03, 0x03, 0x00, 0x00, 0x01, 0x00, 0x04, b'A', b'C', b'M', b'E',
                ])
            }
            0x14 => {
                // FC20 read file record. Return two fixed words [0x1234, 0x5678].
                Some(vec![0x14, 0x06, 0x05, 0x06, 0x12, 0x34, 0x56, 0x78])
            }
            0x15 => {
                // FC21 write file record: echo the request header.
                let mut resp = vec![0x15];
                if pdu.len() >= 8 {
                    resp.extend_from_slice(&pdu[1..8]);
                }
                Some(resp)
            }
            _ => Some(vec![func | 0x80, 0x01]), // illegal function
        }
    }

    #[tokio::test]
    async fn read_holding_registers_end_to_end() {
        let (addr, _store) = spawn_server(vec![0x1234, 0x5678, 0xABCD]).await;

        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.count = Some(3);
        let cfg = Config::build(c).unwrap();

        let mut ctx = connect(&cfg).await.unwrap();
        let words = ctx.read_holding_registers(0, 3).await.unwrap().unwrap();
        assert_eq!(words, vec![0x1234, 0x5678, 0xABCD]);
    }

    #[tokio::test]
    async fn write_single_register_end_to_end() {
        let (addr, store) = spawn_server(vec![0, 0, 0]).await;

        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.write_values = vec!["100".to_string()];
        c.count = Some(1); // force FC6
        let cfg = Config::build(c).unwrap();

        let mut ctx = connect(&cfg).await.unwrap();
        do_write(&mut ctx, &cfg).await.unwrap();

        assert_eq!(store.get(0), 100);
    }

    #[tokio::test]
    async fn write_multiple_registers_end_to_end() {
        let (addr, store) = spawn_server(vec![0, 0, 0]).await;

        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.write_values = vec!["100".to_string(), "200".to_string()];
        let cfg = Config::build(c).unwrap();

        let mut ctx = connect(&cfg).await.unwrap();
        do_write(&mut ctx, &cfg).await.unwrap();

        assert_eq!(store.get(0), 100);
        assert_eq!(store.get(1), 200);
    }

    #[tokio::test]
    async fn do_read_exercises_full_path() {
        let (addr, _store) = spawn_server(vec![0x1234, 0x5678]).await;

        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.count = Some(2);
        let cfg = Config::build(c).unwrap();

        let mut ctx = connect(&cfg).await.unwrap();
        do_read(&mut ctx, &cfg).await.unwrap();
    }

    #[tokio::test]
    async fn run_polls_once() {
        let (addr, _store) = spawn_server(vec![1, 2]).await;

        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.count = Some(2);
        c.poll_once = true;
        let cfg = Config::build(c).unwrap();

        run(&cfg).await.unwrap();
    }

    #[tokio::test]
    async fn read_times_out() {
        let addr = spawn_silent_server().await;

        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.timeout = 0.1;
        c.count = Some(1);
        let cfg = Config::build(c).unwrap();

        let mut ctx = connect(&cfg).await.unwrap();
        let result = do_read(&mut ctx, &cfg).await;
        assert!(result.is_err(), "read should time out");
    }

    // ------------------------------------------------------------------
    // `timed` timeout / error mapping
    // ------------------------------------------------------------------

    #[tokio::test]
    async fn timed_returns_value_on_success() {
        let result: Result<u16> = timed(Duration::from_secs(1), async { Ok(Ok(42u16)) }).await;
        assert_eq!(result.unwrap(), 42);
    }

    #[tokio::test]
    async fn timed_maps_modbus_exception() {
        let result: Result<u16> = timed(Duration::from_secs(1), async {
            Ok(Err(tokio_modbus::prelude::ExceptionCode::IllegalFunction))
        })
        .await;
        assert!(result.is_err());
        assert!(format!("{:#}", result.unwrap_err()).contains("exception"));
    }

    #[tokio::test]
    async fn timed_maps_transport_error() {
        let result: Result<u16> = timed(Duration::from_secs(1), async {
            Err(tokio_modbus::Error::Transport(io::Error::other("boom")))
        })
        .await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn timed_times_out() {
        let result: Result<u16> = timed(
            Duration::from_millis(1),
            std::future::pending::<tokio_modbus::Result<u16>>(),
        )
        .await;
        assert!(result.is_err());
        assert!(format!("{:#}", result.unwrap_err()).contains("timed out"));
    }

    // ------------------------------------------------------------------
    // CLI parsing
    // ------------------------------------------------------------------

    #[test]
    fn cli_parses_read_invocation() {
        let cli = Cli::try_parse_from([
            "rmodpoll",
            "-m",
            "tcp",
            "-a",
            "2",
            "-r",
            "10",
            "-c",
            "5",
            "-t",
            "4:hex",
            "192.168.1.10",
        ])
        .unwrap();
        assert_eq!(cli.target, "192.168.1.10");
        assert_eq!(cli.protocol, Some(Protocol::Tcp));
        assert_eq!(cli.slave, 2);
        assert_eq!(cli.start_ref, 10);
        assert_eq!(cli.count, Some(5));
        assert_eq!(cli.data_type, "4:hex");
        assert!(cli.write_values.is_empty());
    }

    #[test]
    fn cli_parses_write_invocation() {
        let cli = Cli::try_parse_from(["rmodpoll", "192.168.1.10", "100", "200"]).unwrap();
        assert_eq!(cli.write_values, vec!["100", "200"]);
    }

    #[test]
    fn cli_parses_flags() {
        let cli = Cli::try_parse_from([
            "rmodpoll",
            "-0",
            "-1",
            "-u",
            "-i",
            "-f",
            "-4",
            "10",
            "192.168.1.10",
        ])
        .unwrap();
        assert!(cli.zero_based);
        assert!(cli.poll_once);
        assert!(cli.unsigned);
        assert!(cli.big_int);
        assert!(cli.big_float);
        assert_eq!(cli.rts_delay_ms, Some(10));
    }

    #[test]
    fn cli_rejects_unknown_flag() {
        assert!(Cli::try_parse_from(["rmodpoll", "-z", "192.168.1.10"]).is_err());
    }

    // ------------------------------------------------------------------
    // Remaining pure-function / validation coverage
    // ------------------------------------------------------------------

    use tokio_modbus::bytes::Bytes;
    use tokio_modbus::prelude::{ConformityLevel, DeviceIdObject};

    #[test]
    fn protocol_name_maps_all_variants() {
        assert_eq!(protocol_name(Protocol::Ascii), "Modbus ASCII");
        assert_eq!(protocol_name(Protocol::Rtu), "Modbus RTU");
        assert_eq!(protocol_name(Protocol::Tcp), "Modbus TCP");
        assert_eq!(protocol_name(Protocol::Udp), "Modbus UDP");
        assert_eq!(
            protocol_name(Protocol::Enc),
            "Modbus RTU over TCP (encapsulated)"
        );
    }

    #[test]
    fn invalid_ip_port_rejected() {
        let mut c = cli("192.168.1.10");
        c.port_parity = Some("abc".to_string());
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn file_record_number_out_of_range() {
        let mut c = cli("192.168.1.10");
        c.data_type = "file".to_string();
        c.start_ref = 65_536;
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn zero_based_start_ref_out_of_range() {
        let mut c = cli("192.168.1.10");
        c.zero_based = true;
        c.start_ref = 65_536;
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn invalid_data_bits_rejected() {
        let mut c = cli("COM1");
        c.protocol = Some(Protocol::Ascii);
        c.data_bits = Some(6);
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn invalid_stop_bits_rejected() {
        let mut c = cli("192.168.1.10");
        c.stop_bits = 3;
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn invalid_data_type_rejected() {
        let mut c = cli("192.168.1.10");
        c.data_type = "bogus".to_string();
        assert!(Config::build(c).is_err());
    }

    #[test]
    fn file_read_response_error_paths() {
        assert!(parse_file_read_response(&[]).is_err());
        // fewer than 3 bytes
        assert!(parse_file_read_response(&[0x06, 0x05]).is_err());
        // byte_count claims 6 bytes but only 3 follow
        assert!(parse_file_read_response(&[0x06, 0x05, 0x06, 0x00, 0x64]).is_err());
        // sub_len of 0 is malformed
        assert!(parse_file_read_response(&[0x02, 0x00, 0x06]).is_err());
        // unexpected reference type 0x07
        assert!(parse_file_read_response(&[0x04, 0x03, 0x07, 0x00, 0x64]).is_err());
        // odd data length would previously panic on the word-pair access
        assert!(parse_file_read_response(&[0x03, 0x02, 0x06, 0x64]).is_err());
    }

    #[test]
    fn print_bits_formats_references() {
        print_bits(0, false, &[true, false]);
        print_bits(0, true, &[true]);
    }

    #[test]
    fn print_file_words_formats_entries() {
        print_file_words(&[0x1234, 0x5678]);
    }

    #[test]
    fn print_device_id_prints_objects() {
        let resp = ReadDeviceIdentificationResponse {
            read_code: ReadCode::Extended,
            conformity_level: ConformityLevel::ExtendedIdentification,
            more_follows: false,
            next_object_id: 0,
            device_id_objects: vec![DeviceIdObject {
                id: 0,
                value: Bytes::from_static(b"ACME"),
            }],
        };
        print_device_id(&resp);
    }

    #[test]
    fn print_header_covers_network_and_serial() {
        let cfg = Config::build(cli("192.168.1.10")).unwrap();
        print_header(&cfg);

        let mut c = cli("COM1");
        c.protocol = Some(Protocol::Rtu);
        c.rts_delay_ms = Some(10);
        let cfg = Config::build(c).unwrap();
        print_header(&cfg);
    }

    // ------------------------------------------------------------------
    // Additional in-process integration coverage
    // ------------------------------------------------------------------

    #[tokio::test]
    async fn do_read_coils() {
        let (addr, _store) = spawn_server(vec![1]).await;
        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.data_type = "0".to_string();
        c.count = Some(2);
        let cfg = Config::build(c).unwrap();
        let mut ctx = connect(&cfg).await.unwrap();
        do_read(&mut ctx, &cfg).await.unwrap();
    }

    #[tokio::test]
    async fn do_read_discrete_inputs() {
        let (addr, _store) = spawn_server(vec![1]).await;
        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.data_type = "1".to_string();
        c.count = Some(2);
        let cfg = Config::build(c).unwrap();
        let mut ctx = connect(&cfg).await.unwrap();
        do_read(&mut ctx, &cfg).await.unwrap();
    }

    #[tokio::test]
    async fn do_read_input_registers() {
        let (addr, _store) = spawn_server(vec![0x1111, 0x2222]).await;
        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.data_type = "3".to_string();
        c.count = Some(2);
        let cfg = Config::build(c).unwrap();
        let mut ctx = connect(&cfg).await.unwrap();
        do_read(&mut ctx, &cfg).await.unwrap();
    }

    #[tokio::test]
    async fn do_write_coils() {
        let (addr, _store) = spawn_server(vec![1]).await;
        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.data_type = "0".to_string();
        c.write_values = vec!["1".to_string(), "0".to_string()];
        let cfg = Config::build(c).unwrap();
        let mut ctx = connect(&cfg).await.unwrap();
        do_write(&mut ctx, &cfg).await.unwrap();
    }

    #[tokio::test]
    async fn do_write_single_coil() {
        let (addr, _store) = spawn_server(vec![1]).await;
        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.data_type = "0".to_string();
        c.write_values = vec!["1".to_string()];
        c.count = Some(1); // force FC5
        let cfg = Config::build(c).unwrap();
        let mut ctx = connect(&cfg).await.unwrap();
        do_write(&mut ctx, &cfg).await.unwrap();
    }

    #[tokio::test]
    async fn do_read_device_id() {
        let (addr, _store) = spawn_server(vec![0]).await;
        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.data_type = "id".to_string();
        let cfg = Config::build(c).unwrap();
        let mut ctx = connect(&cfg).await.unwrap();
        do_read(&mut ctx, &cfg).await.unwrap();
    }

    #[tokio::test]
    async fn read_file_returns_words() {
        let (addr, _store) = spawn_server(vec![0]).await;
        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.data_type = "file".to_string();
        c.count = Some(2);
        let cfg = Config::build(c).unwrap();
        let mut ctx = connect(&cfg).await.unwrap();
        let words = read_file(&mut ctx, &cfg).await.unwrap();
        assert_eq!(words, vec![0x1234, 0x5678]);
    }

    #[tokio::test]
    async fn write_file_succeeds() {
        let (addr, _store) = spawn_server(vec![0]).await;
        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.data_type = "file".to_string();
        c.write_values = vec!["100".to_string(), "200".to_string()];
        let cfg = Config::build(c).unwrap();
        let mut ctx = connect(&cfg).await.unwrap();
        write_file(&mut ctx, &cfg).await.unwrap();
    }

    #[tokio::test]
    async fn do_read_file() {
        let (addr, _store) = spawn_server(vec![0]).await;
        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.data_type = "file".to_string();
        c.count = Some(2);
        let cfg = Config::build(c).unwrap();
        let mut ctx = connect(&cfg).await.unwrap();
        do_read(&mut ctx, &cfg).await.unwrap();
    }

    #[tokio::test]
    async fn do_write_file() {
        let (addr, _store) = spawn_server(vec![0]).await;
        let mut c = cli("127.0.0.1");
        c.port_parity = Some(addr.port().to_string());
        c.data_type = "file".to_string();
        c.write_values = vec!["100".to_string(), "200".to_string()];
        let cfg = Config::build(c).unwrap();
        let mut ctx = connect(&cfg).await.unwrap();
        do_write(&mut ctx, &cfg).await.unwrap();
    }
}
