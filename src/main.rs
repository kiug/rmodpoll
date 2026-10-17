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
use crate::transport::{AsciiClient, RtsStream, UdpClient, push_u16};

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

        if let Some(c) = cli.count {
            if !(1..=125).contains(&c) {
                bail!("count must be 1-125");
            }
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
            Context::from(Box::new(UdpClient::new(addr, slave).await?) as Box<dyn Client>)
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

fn open_serial(path: &str, cfg: &Config, default_data_bits: u8) -> Result<RtsStream> {
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
    Ok(RtsStream::new(port, rts_delay))
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
    // Serial transports keep a single open port for the whole run, while
    // network transports are re-established on every poll cycle so that a
    // dropped TCP/UDP connection recovers automatically.
    let persistent = cfg.protocol.is_serial();
    let mut serial = if persistent {
        Some(connect(cfg).await?)
    } else {
        None
    };

    let mut final_error = None;

    loop {
        let result = match serial.as_mut() {
            Some(ctx) => poll(ctx, cfg).await,
            None => match connect(cfg).await {
                Ok(mut ctx) => {
                    let result = poll(&mut ctx, cfg).await;
                    let _ = ctx.disconnect().await;
                    result
                }
                Err(e) => Err(anyhow!("connection failed: {e:#}")),
            },
        };

        match result {
            Ok(()) => {}
            Err(e) => {
                eprintln!("Error: {e:#}");
                final_error = Some(e);
            }
        }

        if cfg.poll_once {
            break;
        }
        println!();
        tokio::time::sleep(cfg.poll_rate).await;
    }

    if let Some(ctx) = serial.as_mut() {
        let _ = ctx.disconnect().await;
    }

    match final_error {
        Some(e) => Err(e),
        None => Ok(()),
    }
}

/// Performs a single read or write operation against an open connection.
async fn poll(ctx: &mut Context, cfg: &Config) -> Result<()> {
    if cfg.write_values.is_empty() {
        do_read(ctx, cfg).await
    } else {
        do_write(ctx, cfg).await
    }
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
                ctx.read_device_identification(ReadCode::Basic, 0),
            )
            .await?;
            print_device_id(&resp);
        }
        DataType::File => {
            let words = read_file(ctx, cfg).await?;
            print_file_words(cfg.record_number(), &words);
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
        let value = obj.value_as_str().map(str::to_owned).unwrap_or_else(|| {
            obj.value
                .iter()
                .map(|b| format!("{b:02X}"))
                .collect::<String>()
        });
        println!("  {} (0x{:02X}): {value}", object_name(obj.id), obj.id);
    }
}

fn object_name(id: u8) -> &'static str {
    match id {
        0x00 => "VendorName",
        0x01 => "ProductCode",
        0x02 => "MajorMinorRevision",
        0x03 => "VendorUrl",
        0x04 => "ProductName",
        0x05 => "ModelName",
        0x06 => "UserApplicationName",
        0x07..=0x7F => "Reserved",
        0x80..=0xFF => "Private",
    }
}

fn print_file_words(record_number: u16, words: &[u16]) {
    for (i, word) in words.iter().enumerate() {
        println!(
            "[{}]: 0x{word:04X} ({word})",
            u32::from(record_number) + i as u32
        );
    }
}

async fn read_file(ctx: &mut Context, cfg: &Config) -> Result<Vec<u16>> {
    let mut data = vec![0x06u8];
    push_u16(&mut data, cfg.file_number);
    push_u16(&mut data, cfg.record_number());
    push_u16(&mut data, cfg.value_count());

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

async fn write_file(ctx: &mut Context, cfg: &Config) -> Result<()> {
    let words = encode_write_words(DataType::HoldingReg16, &cfg.write_values, &cfg.value_opts)?;

    let mut data = vec![0x06u8];
    push_u16(&mut data, cfg.file_number);
    push_u16(&mut data, cfg.record_number());
    push_u16(&mut data, words.len() as u16);
    for word in &words {
        push_u16(&mut data, *word);
    }

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
    let mut words = Vec::with_capacity(data.len() / 2);
    for i in (0..data.len()).step_by(2) {
        words.push(u16::from_be_bytes([data[i], data[i + 1]]));
    }
    Ok(words)
}
