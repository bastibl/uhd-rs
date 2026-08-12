//! Intel HEX parsing used by the Cypress FX3 firmware loader.

use crate::{Error, Result};

/// A write to the FX3 address space.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Segment {
    pub address: u32,
    pub data: Vec<u8>,
    pub execute: bool,
}

/// Parse an Intel HEX image and validate every record checksum.
pub fn parse(input: &[u8]) -> Result<Vec<Segment>> {
    let text = std::str::from_utf8(input).map_err(|error| Error::IntelHex {
        line: 0,
        message: format!("image is not ASCII: {error}"),
    })?;
    let mut upper = 0_u16;
    let mut segments = Vec::new();
    let mut saw_eof = false;
    let mut saw_execute = false;

    for (line_index, raw_line) in text.lines().enumerate() {
        let line_number = line_index + 1;
        let line = raw_line.trim();
        if line.is_empty() {
            continue;
        }
        let bytes = decode_record(line, line_number)?;
        let length = usize::from(bytes[0]);
        let address = u16::from_be_bytes([bytes[1], bytes[2]]);
        let kind = bytes[3];
        let data = &bytes[4..4 + length];

        match kind {
            0x00 => {
                if saw_execute {
                    return ihex_error(
                        line_number,
                        "data record appears after the start address record",
                    );
                }
                segments.push(Segment {
                    address: (u32::from(upper) << 16) | u32::from(address),
                    data: data.to_vec(),
                    execute: false,
                });
            }
            0x01 => {
                if address != 0 || !data.is_empty() {
                    return ihex_error(line_number, "EOF record must have address and length zero");
                }
                saw_eof = true;
                break;
            }
            0x04 => {
                if address != 0 || data.len() != 2 {
                    return ihex_error(
                        line_number,
                        "extended linear address record must contain two bytes at address zero",
                    );
                }
                upper = u16::from_be_bytes([data[0], data[1]]);
            }
            0x05 => {
                if address != 0 || data.len() != 4 {
                    return ihex_error(
                        line_number,
                        "start linear address record must contain four bytes at address zero",
                    );
                }
                if saw_execute {
                    return ihex_error(line_number, "multiple start address records");
                }
                saw_execute = true;
                segments.push(Segment {
                    address: u32::from_be_bytes([data[0], data[1], data[2], data[3]]),
                    data: Vec::new(),
                    execute: true,
                });
            }
            other => {
                return ihex_error(
                    line_number,
                    format!("unsupported record type 0x{other:02x}"),
                );
            }
        }
    }

    if !saw_eof {
        return ihex_error(text.lines().count(), "missing EOF record");
    }
    if !saw_execute {
        return ihex_error(text.lines().count(), "missing start address record");
    }
    Ok(segments)
}

fn decode_record(line: &str, line_number: usize) -> Result<Vec<u8>> {
    let Some(hex) = line.strip_prefix(':') else {
        return ihex_error(line_number, "record does not start with ':'");
    };
    if hex.len() < 10 || hex.len() % 2 != 0 {
        return ihex_error(line_number, "record has an invalid length");
    }
    let mut bytes = Vec::with_capacity(hex.len() / 2);
    for offset in (0..hex.len()).step_by(2) {
        bytes.push(
            u8::from_str_radix(&hex[offset..offset + 2], 16).map_err(|_| Error::IntelHex {
                line: line_number,
                message: "record contains a non-hexadecimal byte".into(),
            })?,
        );
    }
    let expected = usize::from(bytes[0]) + 5;
    if bytes.len() != expected {
        return ihex_error(
            line_number,
            format!("record contains {} bytes, expected {expected}", bytes.len()),
        );
    }
    if bytes.iter().fold(0_u8, |sum, byte| sum.wrapping_add(*byte)) != 0 {
        return ihex_error(line_number, "checksum mismatch");
    }
    Ok(bytes)
}

fn ihex_error<T>(line: usize, message: impl Into<String>) -> Result<T> {
    Err(Error::IntelHex {
        line,
        message: message.into(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_extended_and_start_addresses() {
        let input = b":020000040001F9\n:0400100001020304E2\n:0400000500010010E6\n:00000001FF\n";
        let parsed = parse(input).unwrap();
        assert_eq!(
            parsed,
            vec![
                Segment {
                    address: 0x0001_0010,
                    data: vec![1, 2, 3, 4],
                    execute: false,
                },
                Segment {
                    address: 0x0001_0010,
                    data: vec![],
                    execute: true,
                }
            ]
        );
    }

    #[test]
    fn rejects_bad_checksum() {
        let error = parse(b":00000001FE\n").unwrap_err();
        assert!(matches!(error, Error::IntelHex { line: 1, .. }));
    }

    #[test]
    fn requires_eof() {
        assert!(parse(b":0400000500010010E6\n").is_err());
    }

    #[test]
    fn requires_start_address() {
        assert!(parse(b":00000001FF\n").is_err());
    }
}
