//! Modbus data-type parsing, value encoding/decoding and display formatting.

use anyhow::{Result, anyhow, bail};

/// How a single logical value is represented on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataType {
    /// `-t 0` — discrete output (coil), FC 1/5/15.
    Coil,
    /// `-t 1` — discrete input, FC 2.
    DiscreteInput,
    /// `-t 3` — 16-bit input register, FC 4.
    InputReg16,
    /// `-t 3:hex`
    InputRegHex,
    /// `-t 3:i32`
    InputRegI32,
    /// `-t 3:i64`
    InputRegI64,
    /// `-t 3:mod`
    InputRegMod,
    /// `-t 3:f32`
    InputRegF32,
    /// `-t 3:f64`
    InputRegF64,
    /// `-t 4` — 16-bit holding register, FC 3/6/16 (default).
    HoldingReg16,
    /// `-t 4:hex`
    HoldingRegHex,
    /// `-t 4:i32`
    HoldingRegI32,
    /// `-t 4:i64`
    HoldingRegI64,
    /// `-t 4:mod`
    HoldingRegMod,
    /// `-t 4:f32`
    HoldingRegF32,
    /// `-t 4:f64`
    HoldingRegF64,
    /// `-t id` — device identification objects, FC 43/14.
    DeviceId,
    /// `-t file` — file record reference type 6, FC 20/21.
    File,
}

impl DataType {
    pub fn parse(s: &str) -> Result<Self> {
        let lower = s.trim().to_ascii_lowercase();
        Ok(match lower.as_str() {
            "0" => Self::Coil,
            "1" => Self::DiscreteInput,
            "3" => Self::InputReg16,
            "3:hex" => Self::InputRegHex,
            "3:i32" => Self::InputRegI32,
            "3:i64" => Self::InputRegI64,
            "3:mod" => Self::InputRegMod,
            "3:f32" => Self::InputRegF32,
            "3:f64" => Self::InputRegF64,
            "4" => Self::HoldingReg16,
            "4:hex" => Self::HoldingRegHex,
            "4:i32" => Self::HoldingRegI32,
            "4:i64" => Self::HoldingRegI64,
            "4:mod" => Self::HoldingRegMod,
            "4:f32" => Self::HoldingRegF32,
            "4:f64" => Self::HoldingRegF64,
            "id" => Self::DeviceId,
            "file" => Self::File,
            _ => bail!(
                "unknown data type '{s}' (expected 0, 1, 3, 3:hex, 3:i32, 3:i64, 3:mod, \
                 3:f32, 3:f64, 4, 4:hex, 4:i32, 4:i64, 4:mod, 4:f32, 4:f64, id or file)"
            ),
        })
    }

    pub fn is_coil(self) -> bool {
        matches!(self, Self::Coil)
    }

    pub fn is_input_register(self) -> bool {
        matches!(
            self,
            Self::InputReg16
                | Self::InputRegHex
                | Self::InputRegI32
                | Self::InputRegI64
                | Self::InputRegMod
                | Self::InputRegF32
                | Self::InputRegF64
        )
    }

    pub fn is_holding_register(self) -> bool {
        matches!(
            self,
            Self::HoldingReg16
                | Self::HoldingRegHex
                | Self::HoldingRegI32
                | Self::HoldingRegI64
                | Self::HoldingRegMod
                | Self::HoldingRegF32
                | Self::HoldingRegF64
        )
    }

    pub fn is_register(self) -> bool {
        self.is_input_register() || self.is_holding_register()
    }

    pub fn is_writable(self) -> bool {
        self.is_coil() || self.is_holding_register() || matches!(self, Self::File)
    }

    /// Number of 16-bit registers needed to encode a single logical value.
    pub fn words_per_value(self) -> usize {
        match self {
            Self::InputReg16 | Self::InputRegHex | Self::HoldingReg16 | Self::HoldingRegHex => 1,
            Self::InputRegI32
            | Self::InputRegMod
            | Self::InputRegF32
            | Self::HoldingRegI32
            | Self::HoldingRegMod
            | Self::HoldingRegF32 => 2,
            Self::InputRegI64 | Self::InputRegF64 | Self::HoldingRegI64 | Self::HoldingRegF64 => 4,
            _ => 0,
        }
    }

    /// Short human-readable description used in the startup header.
    pub fn describe(self) -> &'static str {
        match self {
            Self::Coil => "discrete output (coil), FC 1/5/15",
            Self::DiscreteInput => "discrete input, FC 2",
            Self::InputReg16 => "16-bit input register, FC 4",
            Self::InputRegHex => "16-bit input register (hex display), FC 4",
            Self::InputRegI32 => "32-bit integer input register, FC 4",
            Self::InputRegI64 => "64-bit integer input register, FC 4",
            Self::InputRegMod => "32-bit modulo-10000 input register, FC 4",
            Self::InputRegF32 => "32-bit float input register, FC 4",
            Self::InputRegF64 => "64-bit double input register, FC 4",
            Self::HoldingReg16 => "16-bit holding register, FC 3/6/16",
            Self::HoldingRegHex => "16-bit holding register (hex display), FC 3/6/16",
            Self::HoldingRegI32 => "32-bit integer holding register, FC 3/16",
            Self::HoldingRegI64 => "64-bit integer holding register, FC 3/16",
            Self::HoldingRegMod => "32-bit modulo-10000 holding register, FC 3/16",
            Self::HoldingRegF32 => "32-bit float holding register, FC 3/16",
            Self::HoldingRegF64 => "64-bit double holding register, FC 3/16",
            Self::DeviceId => "device identification objects, FC 43/14",
            Self::File => "file record reference type 6, FC 20/21",
        }
    }
}

/// Flags controlling how multi-register values are interpreted.
#[derive(Debug, Clone, Copy, Default)]
pub struct ValueOptions {
    pub unsigned: bool,
    pub big_endian_int: bool,
    pub big_endian_float: bool,
}

/// Format a list of raw registers according to a data type.
pub fn format_registers(words: &[u16], dt: DataType, opts: &ValueOptions) -> Result<Vec<String>> {
    let wpv = dt.words_per_value();
    if wpv == 0 {
        bail!("data type {dt:?} is not register-based");
    }
    if !words.len().is_multiple_of(wpv) {
        bail!(
            "received {} registers, not a multiple of {wpv}",
            words.len()
        );
    }

    let mut out = Vec::with_capacity(words.len() / wpv);
    for chunk in words.chunks_exact(wpv) {
        out.push(format_one(chunk, dt, opts));
    }
    Ok(out)
}

fn format_one(words: &[u16], dt: DataType, opts: &ValueOptions) -> String {
    match dt {
        DataType::InputReg16 | DataType::HoldingReg16 => {
            let v = words[0];
            if opts.unsigned {
                format!("{v}")
            } else {
                format!("{}", v as i16)
            }
        }
        DataType::InputRegHex | DataType::HoldingRegHex => format!("0x{:04X}", words[0]),
        DataType::InputRegI32 | DataType::HoldingRegI32 => {
            let v = if opts.big_endian_int {
                u32_from_words_be(words)
            } else {
                u32_from_words_le(words)
            };
            if opts.unsigned {
                format!("{v}")
            } else {
                format!("{}", v as i32)
            }
        }
        DataType::InputRegI64 | DataType::HoldingRegI64 => {
            let v = if opts.big_endian_int {
                u64_from_words_be(words)
            } else {
                u64_from_words_le(words)
            };
            if opts.unsigned {
                format!("{v}")
            } else {
                format!("{}", v as i64)
            }
        }
        DataType::InputRegMod | DataType::HoldingRegMod => {
            // Two base-10000 "digits" combined into a 32-bit value. As in
            // `modpoll`, the first register holds the most significant digit
            // and the order is fixed (not affected by -i).
            let v = u32::from(words[0]) * 10_000 + u32::from(words[1]);
            format!("{v}")
        }
        DataType::InputRegF32 | DataType::HoldingRegF32 => {
            let f = if opts.big_endian_float {
                f32_from_words_be(words)
            } else {
                f32_from_words_le(words)
            };
            format!("{f}")
        }
        DataType::InputRegF64 | DataType::HoldingRegF64 => {
            let f = if opts.big_endian_float {
                f64_from_words_be(words)
            } else {
                f64_from_words_le(words)
            };
            format!("{f}")
        }
        _ => String::new(),
    }
}

/// Parse a list of textual write values into coils.
pub fn parse_bits(values: &[String]) -> Result<Vec<bool>> {
    values.iter().map(|s| parse_bit(s)).collect()
}

fn parse_bit(s: &str) -> Result<bool> {
    match s.trim().to_ascii_lowercase().as_str() {
        "1" | "true" | "on" | "0xff00" => Ok(true),
        "0" | "false" | "off" | "0x0000" => Ok(false),
        _ => bail!("invalid coil value '{s}' (expected 0 or 1)"),
    }
}

/// Parse textual write values into register words for the given data type.
pub fn encode_write_words(
    dt: DataType,
    values: &[String],
    opts: &ValueOptions,
) -> Result<Vec<u16>> {
    let mut out = Vec::new();
    for s in values {
        match dt {
            DataType::HoldingReg16 | DataType::HoldingRegHex => out.push(parse_u16(s)?),
            DataType::HoldingRegI32 => push_u32_words(&mut out, parse_u32(s)?, opts.big_endian_int),
            DataType::HoldingRegI64 => push_u64_words(&mut out, parse_u64(s)?, opts.big_endian_int),
            DataType::HoldingRegMod => push_mod_words(&mut out, parse_u32(s)?),
            DataType::HoldingRegF32 => push_f32_words(
                &mut out,
                s.parse::<f32>()
                    .map_err(|_| anyhow!("invalid float '{s}'"))?,
                opts.big_endian_float,
            ),
            DataType::HoldingRegF64 => push_f64_words(
                &mut out,
                s.parse::<f64>()
                    .map_err(|_| anyhow!("invalid double '{s}'"))?,
                opts.big_endian_float,
            ),
            _ => bail!("data type {dt:?} is not writable"),
        }
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// Word/byte assembly helpers
// ---------------------------------------------------------------------------

fn u32_from_words_be(w: &[u16]) -> u32 {
    (u32::from(w[0]) << 16) | u32::from(w[1])
}

fn u32_from_words_le(w: &[u16]) -> u32 {
    (u32::from(w[1]) << 16) | u32::from(w[0])
}

fn u64_from_words_be(w: &[u16]) -> u64 {
    (u64::from(w[0]) << 48) | (u64::from(w[1]) << 32) | (u64::from(w[2]) << 16) | u64::from(w[3])
}

fn u64_from_words_le(w: &[u16]) -> u64 {
    (u64::from(w[3]) << 48) | (u64::from(w[2]) << 32) | (u64::from(w[1]) << 16) | u64::from(w[0])
}

fn f32_from_words_be(w: &[u16]) -> f32 {
    f32::from_be_bytes([(w[0] >> 8) as u8, w[0] as u8, (w[1] >> 8) as u8, w[1] as u8])
}

fn f32_from_words_le(w: &[u16]) -> f32 {
    f32::from_le_bytes([w[0] as u8, (w[0] >> 8) as u8, w[1] as u8, (w[1] >> 8) as u8])
}

fn f64_from_words_be(w: &[u16]) -> f64 {
    let mut b = [0u8; 8];
    for (i, word) in w.iter().enumerate() {
        b[2 * i] = (word >> 8) as u8;
        b[2 * i + 1] = *word as u8;
    }
    f64::from_be_bytes(b)
}

fn f64_from_words_le(w: &[u16]) -> f64 {
    let mut b = [0u8; 8];
    for (i, word) in w.iter().enumerate() {
        b[2 * i] = *word as u8;
        b[2 * i + 1] = (word >> 8) as u8;
    }
    f64::from_le_bytes(b)
}

fn push_u32_words(out: &mut Vec<u16>, v: u32, big_endian: bool) {
    let hi = (v >> 16) as u16;
    let lo = v as u16;
    if big_endian {
        out.push(hi);
        out.push(lo);
    } else {
        out.push(lo);
        out.push(hi);
    }
}

fn push_u64_words(out: &mut Vec<u16>, v: u64, big_endian: bool) {
    for i in 0..4 {
        let shift = if big_endian { (3 - i) * 16 } else { i * 16 };
        out.push(((v >> shift) & 0xFFFF) as u16);
    }
}

fn push_mod_words(out: &mut Vec<u16>, v: u32) {
    // Most significant base-10000 digit first, matching `modpoll`.
    out.push((v / 10_000) as u16);
    out.push((v % 10_000) as u16);
}

fn push_f32_words(out: &mut Vec<u16>, f: f32, big_endian: bool) {
    let b = if big_endian {
        f.to_be_bytes()
    } else {
        f.to_le_bytes()
    };
    if big_endian {
        out.push(u16::from_be_bytes([b[0], b[1]]));
        out.push(u16::from_be_bytes([b[2], b[3]]));
    } else {
        out.push(u16::from_le_bytes([b[0], b[1]]));
        out.push(u16::from_le_bytes([b[2], b[3]]));
    }
}

fn push_f64_words(out: &mut Vec<u16>, f: f64, big_endian: bool) {
    let b = if big_endian {
        f.to_be_bytes()
    } else {
        f.to_le_bytes()
    };
    if big_endian {
        for i in 0..4 {
            out.push(u16::from_be_bytes([b[2 * i], b[2 * i + 1]]));
        }
    } else {
        for i in 0..4 {
            out.push(u16::from_le_bytes([b[2 * i], b[2 * i + 1]]));
        }
    }
}

// ---------------------------------------------------------------------------
// Integer parsing helpers
// ---------------------------------------------------------------------------

fn strip_hex_prefix(s: &str) -> Option<&str> {
    s.strip_prefix("0x").or_else(|| s.strip_prefix("0X"))
}

fn parse_u16(s: &str) -> Result<u16> {
    if let Some(h) = strip_hex_prefix(s) {
        u16::from_str_radix(h, 16).map_err(|_| anyhow!("invalid hex value '{s}'"))
    } else {
        let v: i64 = s.parse().map_err(|_| anyhow!("invalid integer '{s}'"))?;
        if !(-32_768..=65_535).contains(&v) {
            bail!("value '{s}' out of range for a 16-bit register");
        }
        Ok(v as u16)
    }
}

fn parse_u32(s: &str) -> Result<u32> {
    if let Some(h) = strip_hex_prefix(s) {
        u32::from_str_radix(h, 16).map_err(|_| anyhow!("invalid hex value '{s}'"))
    } else {
        let v: i64 = s.parse().map_err(|_| anyhow!("invalid integer '{s}'"))?;
        if !(-(1i64 << 31)..=((1i64 << 32) - 1)).contains(&v) {
            bail!("value '{s}' out of range for a 32-bit value");
        }
        Ok(v as u32)
    }
}

fn parse_u64(s: &str) -> Result<u64> {
    if let Some(h) = strip_hex_prefix(s) {
        u64::from_str_radix(h, 16).map_err(|_| anyhow!("invalid hex value '{s}'"))
    } else {
        let v: i64 = s.parse().map_err(|_| anyhow!("invalid integer '{s}'"))?;
        // A 64-bit word can hold both signed and unsigned values via two's
        // complement, so a plain i64 parse covers the whole 64-bit range.
        Ok(v as u64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_types() {
        assert_eq!(DataType::parse("4").unwrap(), DataType::HoldingReg16);
        assert_eq!(DataType::parse("4:hex").unwrap(), DataType::HoldingRegHex);
        assert_eq!(DataType::parse("3:f32").unwrap(), DataType::InputRegF32);
        assert_eq!(DataType::parse("id").unwrap(), DataType::DeviceId);
        assert_eq!(DataType::parse("file").unwrap(), DataType::File);
        assert!(DataType::parse("bogus").is_err());
    }

    #[test]
    fn format_16bit() {
        let opts = ValueOptions::default();
        let values = format_registers(&[0xFFFF], DataType::HoldingReg16, &opts).unwrap();
        assert_eq!(values, vec!["-1".to_string()]);

        let unsigned = ValueOptions {
            unsigned: true,
            ..ValueOptions::default()
        };
        let values = format_registers(&[0xFFFF], DataType::HoldingReg16, &unsigned).unwrap();
        assert_eq!(values, vec!["65535".to_string()]);
    }

    #[test]
    fn format_hex() {
        let opts = ValueOptions::default();
        let values = format_registers(&[0x12AB], DataType::HoldingRegHex, &opts).unwrap();
        assert_eq!(values, vec!["0x12AB".to_string()]);
    }

    #[test]
    fn hex_write_parses_decimal() {
        // `modpoll` parses `4:hex` write values as decimal (the `:hex` suffix
        // only affects display), with an optional `0x` hex prefix.
        let opts = ValueOptions::default();
        let words =
            encode_write_words(DataType::HoldingRegHex, &["1234".to_string()], &opts).unwrap();
        assert_eq!(words, vec![1234]); // 0x04D2
        let words =
            encode_write_words(DataType::HoldingRegHex, &["0x1234".to_string()], &opts).unwrap();
        assert_eq!(words, vec![0x1234]);
    }

    #[test]
    fn i32_endianness() {
        let little = ValueOptions::default();
        let values = format_registers(&[0x0001, 0x0002], DataType::HoldingRegI32, &little).unwrap();
        assert_eq!(values, vec!["131073".to_string()]); // 0x00020001

        let big = ValueOptions {
            big_endian_int: true,
            ..ValueOptions::default()
        };
        let values = format_registers(&[0x0001, 0x0002], DataType::HoldingRegI32, &big).unwrap();
        assert_eq!(values, vec!["65538".to_string()]); // 0x00010002
    }

    #[test]
    fn i32_negative() {
        let opts = ValueOptions::default();
        let values = format_registers(&[0xFFFF, 0xFFFF], DataType::HoldingRegI32, &opts).unwrap();
        assert_eq!(values, vec!["-1".to_string()]);
    }

    #[test]
    fn f32_round_trip() {
        let big = ValueOptions {
            big_endian_float: true,
            ..ValueOptions::default()
        };
        let words =
            encode_write_words(DataType::HoldingRegF32, &["1.5".to_string()], &big).unwrap();
        assert_eq!(words, vec![0x3FC0, 0x0000]);
        assert_eq!(
            format_registers(&words, DataType::HoldingRegF32, &big).unwrap(),
            vec!["1.5".to_string()]
        );

        let little = ValueOptions::default();
        let words =
            encode_write_words(DataType::HoldingRegF32, &["1.5".to_string()], &little).unwrap();
        assert_eq!(words, vec![0x0000, 0x3FC0]);
        assert_eq!(
            format_registers(&words, DataType::HoldingRegF32, &little).unwrap(),
            vec!["1.5".to_string()]
        );
    }

    #[test]
    fn mod_word_order() {
        // `modpoll` stores the most significant base-10000 digit in the first
        // register, and this order is fixed (not affected by -i).
        let opts = ValueOptions::default();
        let words =
            encode_write_words(DataType::HoldingRegMod, &["12345678".to_string()], &opts).unwrap();
        assert_eq!(words, vec![1234, 5678]);
        assert_eq!(
            format_registers(&[1234, 5678], DataType::HoldingRegMod, &opts).unwrap(),
            vec!["12345678".to_string()]
        );

        let big = ValueOptions {
            big_endian_int: true,
            ..ValueOptions::default()
        };
        let words =
            encode_write_words(DataType::HoldingRegMod, &["12345678".to_string()], &big).unwrap();
        assert_eq!(words, vec![1234, 5678]);
    }

    #[test]
    fn i64_round_trip() {
        // 0x0123456789ABCDEF
        let little = ValueOptions::default();
        let words = encode_write_words(
            DataType::HoldingRegI64,
            &["81985529216486895".to_string()],
            &little,
        )
        .unwrap();
        assert_eq!(words, vec![0xCDEF, 0x89AB, 0x4567, 0x0123]);
        assert_eq!(
            format_registers(&words, DataType::HoldingRegI64, &little).unwrap(),
            vec!["81985529216486895".to_string()]
        );

        let big = ValueOptions {
            big_endian_int: true,
            ..ValueOptions::default()
        };
        let words = encode_write_words(
            DataType::HoldingRegI64,
            &["81985529216486895".to_string()],
            &big,
        )
        .unwrap();
        assert_eq!(words, vec![0x0123, 0x4567, 0x89AB, 0xCDEF]);
    }

    #[test]
    fn f64_round_trip() {
        // 1.5 = 0x3FF8000000000000
        let little = ValueOptions::default();
        let words =
            encode_write_words(DataType::HoldingRegF64, &["1.5".to_string()], &little).unwrap();
        assert_eq!(words, vec![0x0000, 0x0000, 0x0000, 0x3FF8]);
        assert_eq!(
            format_registers(&words, DataType::HoldingRegF64, &little).unwrap(),
            vec!["1.5".to_string()]
        );

        let big = ValueOptions {
            big_endian_float: true,
            ..ValueOptions::default()
        };
        let words =
            encode_write_words(DataType::HoldingRegF64, &["1.5".to_string()], &big).unwrap();
        assert_eq!(words, vec![0x3FF8, 0x0000, 0x0000, 0x0000]);
    }

    #[test]
    fn invalid_write_value_rejected() {
        let opts = ValueOptions::default();
        assert!(encode_write_words(DataType::HoldingReg16, &["70000".to_string()], &opts).is_err());
        assert!(encode_write_words(DataType::HoldingReg16, &["abc".to_string()], &opts).is_err());
        assert!(
            encode_write_words(DataType::HoldingRegF32, &["not-a-float".to_string()], &opts)
                .is_err()
        );
        assert!(parse_bits(&["2".to_string()]).is_err());
    }

    #[test]
    fn format_registers_rejects_bad_word_count() {
        let opts = ValueOptions::default();
        // 3 words for a 2-word type (i32)
        assert!(format_registers(&[1, 2, 3], DataType::HoldingRegI32, &opts).is_err());
    }

    #[test]
    fn parse_bits_accepts_aliases() {
        for s in ["1", "true", "on", "0xff00"] {
            assert_eq!(parse_bits(&[s.to_string()]).unwrap(), vec![true]);
        }
        for s in ["0", "false", "off", "0x0000"] {
            assert_eq!(parse_bits(&[s.to_string()]).unwrap(), vec![false]);
        }
    }

    #[test]
    fn format_registers_rejects_non_register_type() {
        let opts = ValueOptions::default();
        assert!(format_registers(&[], DataType::Coil, &opts).is_err());
        assert!(format_registers(&[], DataType::DeviceId, &opts).is_err());
    }

    #[test]
    fn write_accepts_signed_and_boundary_values() {
        let opts = ValueOptions::default();
        assert_eq!(
            encode_write_words(DataType::HoldingReg16, &["-1".to_string()], &opts).unwrap(),
            vec![0xFFFF]
        );
        assert_eq!(
            encode_write_words(DataType::HoldingReg16, &["-32768".to_string()], &opts).unwrap(),
            vec![0x8000]
        );
        assert_eq!(
            encode_write_words(DataType::HoldingReg16, &["65535".to_string()], &opts).unwrap(),
            vec![0xFFFF]
        );
        assert!(encode_write_words(DataType::HoldingReg16, &["65536".to_string()], &opts).is_err());
    }

    #[test]
    fn hex_prefix_is_case_insensitive() {
        let opts = ValueOptions::default();
        assert_eq!(
            encode_write_words(DataType::HoldingReg16, &["0X1234".to_string()], &opts).unwrap(),
            vec![0x1234]
        );
    }

    #[test]
    fn write_32bit_out_of_range_rejected() {
        let opts = ValueOptions::default();
        assert!(
            encode_write_words(DataType::HoldingRegI32, &["4294967296".to_string()], &opts)
                .is_err()
        );
        assert_eq!(
            encode_write_words(DataType::HoldingRegI32, &["-1".to_string()], &opts).unwrap(),
            vec![0xFFFF, 0xFFFF]
        );
    }

    #[test]
    fn unsigned_i32_display() {
        let opts = ValueOptions {
            unsigned: true,
            ..ValueOptions::default()
        };
        let values = format_registers(&[0xFFFF, 0xFFFF], DataType::HoldingRegI32, &opts).unwrap();
        assert_eq!(values, vec!["4294967295".to_string()]);
    }

    #[test]
    fn encode_write_words_rejects_non_writable_type() {
        let opts = ValueOptions::default();
        assert!(encode_write_words(DataType::Coil, &["1".to_string()], &opts).is_err());
        assert!(encode_write_words(DataType::DiscreteInput, &["1".to_string()], &opts).is_err());
    }
}
