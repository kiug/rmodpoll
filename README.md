# rmodpoll

A Rust command-line **Modbus master** for testing communication with devices.
It replicates the interface of the well-known [`modpoll`](https://www.modbusdriver.com/modpoll.html)
tool and supports all of its protocols, data types and function codes.

## Features

- **Protocols**: Modbus TCP, UDP, RTU, ASCII and encapsulated RTU over TCP (`enc`)
- **Data types**: discrete outputs (coils), discrete inputs, input registers and
  holding registers — as 16-bit, hex, 32/64-bit integers, modulo-10000, and
  32/64-bit floats
- **Function codes**: FC 1/2/3/4 (read), FC 5/6/15/16 (write), FC 20/21
  (file records) and FC 43/14 (device identification)
- **Write support**: single/multiple coil and register writes
- **Endianness control**: big-endian 32/64-bit integers (`-i`) and floats (`-f`)
- **0-based or 1-based references** (`-0`)
- **RS-485** half-duplex RTS control with hardware drain (`-4`)
- **Auto-reconnect** for network transports after a failed transaction

## Requirements

- Rust 1.88+ (edition 2024)

## Building

```sh
cargo build --release
```

The binary is written to `target/release/rmodpoll`.

## Usage

```
Usage: rmodpoll [OPTIONS] <SERIALPORT|HOST> [WRITEVALUES...]
```

If no `WRITEVALUES` are given, `rmodpoll` reads data (polling at the configured
rate). Otherwise it writes the given values once and exits.

### Options

| Option | Description |
|---|---|
| `-m <PROTOCOL>` | Protocol: `ascii`, `rtu`, `tcp`, `udp` or `enc` (auto-detected if omitted) |
| `-a <ADDR>` | Slave address (1-247 serial, 0-255 TCP, default 1) |
| `-r <REF>` | Start reference (1-65536, default 1; 0-based with `-0`) |
| `-c <COUNT>` | Number of values (1-125, default 1); for writes use `-c 1` to force FC5/FC6 |
| `-t <TYPE>` | Data type (see below) |
| `-n <FILE>` | File number for file records (default 4) |
| `-u` | Display data as unsigned integers |
| `-i` | Slave operates on big-endian 32/64-bit integers |
| `-f` | Slave operates on big-endian 32/64-bit floats |
| `-0` | First reference is 0 (PDU addressing) instead of 1 |
| `-1` | Poll only once |
| `-l <MS>` | Poll rate in ms (default 1000) |
| `-o <SEC>` | Timeout in seconds (0.01 - 10.0, default 1.0) |
| `-p <PORT\|PARITY>` | IP port (TCP/UDP/enc) or parity (serial: `none`, `even`, `odd`) |
| `-b <BAUD>` | Baudrate (default 19200) |
| `-d <BITS>` | Data bits (7 or 8 for ASCII, 8 for RTU) |
| `-s <BITS>` | Stop bits (1 or 2, default 1) |
| `-4 <MS>` | RS-485 mode: RTS on while transmitting and N ms after |

### Data types (`-t`)

| Type | Description | Function code |
|---|---|---|
| `0` | Discrete output (coil) | FC 1 / 5 / 15 |
| `1` | Discrete input | FC 2 |
| `3` | 16-bit input register | FC 4 |
| `3:hex` | 16-bit input register, hex display | FC 4 |
| `3:i32` / `3:i64` | 32/64-bit integer input register | FC 4 |
| `3:mod` | 32-bit modulo-10000 input register | FC 4 |
| `3:f32` / `3:f64` | 32/64-bit float input register | FC 4 |
| `4` | 16-bit holding register (default) | FC 3 / 6 / 16 |
| `4:hex` | 16-bit holding register, hex display | FC 3 / 6 / 16 |
| `4:i32` / `4:i64` | 32/64-bit integer holding register | FC 3 / 16 |
| `4:mod` | 32-bit modulo-10000 holding register | FC 3 / 16 |
| `4:f32` / `4:f64` | 32/64-bit float holding register | FC 3 / 16 |
| `id` | Device identification objects | FC 43 / 14 |
| `file` | File record reference type 6 | FC 20 / 21 |

### Protocol auto-detection

When `-m` is omitted, the protocol is guessed from the target:

- contains `\` or `COM` → `rtu` (Windows serial port)
- starts with `/dev/` → `rtu` (Linux serial port)
- otherwise → `tcp`

## Examples

Read 10 holding registers from a Modbus TCP device:

```sh
rmodpoll -m tcp -a 1 -r 1 -c 10 192.168.1.10
```

Read 4 input registers (FC 4):

```sh
rmodpoll -m tcp -t 3 -r 1 -c 4 192.168.1.10
```

Read coils (FC 1):

```sh
rmodpoll -m tcp -t 0 -r 1 -c 8 192.168.1.10
```

Read two 32-bit floats (holding registers):

```sh
rmodpoll -m tcp -t 4:f32 -r 1 -c 2 192.168.1.10
```

Write a single holding register (FC 6):

```sh
rmodpoll -m tcp -t 4 -r 1 -c 1 192.168.1.10 100
```

Write multiple holding registers (FC 16):

```sh
rmodpoll -m tcp -t 4 -r 1 -c 2 192.168.1.10 100 200
```

Poll only once (instead of continuously):

```sh
rmodpoll -1 -t 4 -r 1 -c 10 192.168.1.10
```

Modbus RTU over a serial port (8 data bits, no parity):

```sh
rmodpoll -m rtu -b 9600 -p none -t 4 -r 1 -c 4 /dev/ttyUSB0
```

Modbus ASCII over a serial port (7 data bits):

```sh
rmodpoll -m ascii -b 9600 -d 7 -t 4 -r 1 -c 4 /dev/ttyUSB0
```

Read device identification objects (FC 43/14):

```sh
rmodpoll -m tcp -t id 192.168.1.10
```

Modbus UDP (default port 502, or specify one with `-p`):

```sh
rmodpoll -m udp -t 4 -r 1 -c 4 192.168.1.10
rmodpoll -m udp -p 1502 -t 4 -r 1 -c 4 192.168.1.10
```

Encapsulated Modbus RTU over TCP (`enc`):

```sh
rmodpoll -m enc -t 4 -r 1 -c 4 192.168.1.10
```

Read a file record (FC 20):

```sh
rmodpoll -m tcp -t file -n 4 -r 1 -c 2 192.168.1.10
```

## Testing

The test suite uses an in-process Modbus TCP server and in-memory mock
serial/UDP transports, so no hardware or network is required:

```sh
cargo test
```

To measure line coverage with `cargo-llvm-cov`:

```sh
cargo llvm-cov
```

Coverage is currently around 89% across the crate (82 tests). CI runs
`cargo fmt --check`, `cargo clippy -D warnings`, `cargo test` and
`cargo llvm-cov` on every push and pull request.

## Notes

- For multi-word types (`i32`, `i64`, `f32`, `f64`, `mod`), the reference
  advances by the number of registers per value (2 for 32-bit, 4 for 64-bit).
- By default 32/64-bit values use little-endian word order. Use `-i`/`-f` for
  big-endian slaves.
- Writing a single 16-bit value without `-c` uses FC16; add `-c 1` to force the
  single-write function (FC5/FC6).

## License

`rmodpoll` is licensed under the [GNU General Public License, version 2](LICENSE)
(`GPL-2.0-only`).
