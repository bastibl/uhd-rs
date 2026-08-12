//! The compact VITA/CHDR framing used by the B2xx FPGA.

use crate::{Error, Result};

const CONTEXT: u32 = 1 << 31;
const HAS_TIME: u32 = 1 << 29;
const END_OF_BURST: u32 = 1 << 28;
const LENGTH_MASK: u32 = 0xffff;
const SEQUENCE_MASK: u16 = 0x0fff;

/// Parsed metadata and payload for a CHDR packet.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Packet<'a> {
    pub context: bool,
    pub end_of_burst: bool,
    pub sequence: u16,
    pub stream_id: u32,
    pub timestamp: Option<u64>,
    pub payload: &'a [u8],
}

/// Build the B2xx context packet used for register transactions.
pub fn encode_control(
    stream_id: u32,
    sequence: u16,
    address: u32,
    value: u32,
    timestamp: Option<u64>,
) -> Vec<u8> {
    let (words, packet_bytes) = if timestamp.is_some() {
        (6_usize, 24_u32)
    } else {
        (4_usize, 16_u32)
    };
    let mut header = CONTEXT | (u32::from(sequence & SEQUENCE_MASK) << 16) | packet_bytes;
    if timestamp.is_some() {
        header |= HAS_TIME;
    }
    let mut bytes = Vec::with_capacity(words * 4);
    push_word(&mut bytes, header);
    push_word(&mut bytes, stream_id);
    if let Some(ticks) = timestamp {
        let (low, high) = split_u64(ticks);
        push_word(&mut bytes, high);
        push_word(&mut bytes, low);
    }
    push_word(&mut bytes, address);
    push_word(&mut bytes, value);
    bytes
}

/// Build a CHDR data packet. Payload length does not need to be word-aligned.
pub fn encode_data(
    stream_id: u32,
    sequence: u16,
    payload: &[u8],
    timestamp: Option<u64>,
    end_of_burst: bool,
) -> Result<Vec<u8>> {
    let header_bytes = if timestamp.is_some() { 16 } else { 8 };
    let packet_bytes = header_bytes + payload.len();
    let packet_length = u16::try_from(packet_bytes)
        .map_err(|_| Error::InvalidArgument("CHDR packet exceeds 65535 bytes".into()))?;
    let mut header = (u32::from(sequence & SEQUENCE_MASK) << 16) | u32::from(packet_length);
    if timestamp.is_some() {
        header |= HAS_TIME;
    }
    if end_of_burst {
        header |= END_OF_BURST;
    }
    let padded_bytes = packet_bytes.next_multiple_of(4);
    let mut bytes = Vec::with_capacity(padded_bytes);
    push_word(&mut bytes, header);
    push_word(&mut bytes, stream_id);
    if let Some(ticks) = timestamp {
        let (low, high) = split_u64(ticks);
        push_word(&mut bytes, high);
        push_word(&mut bytes, low);
    }
    bytes.extend_from_slice(payload);
    bytes.resize(padded_bytes, 0);
    Ok(bytes)
}

/// Parse one CHDR packet from the beginning of `bytes`.
pub fn parse(bytes: &[u8]) -> Result<Packet<'_>> {
    if bytes.len() < 8 {
        return Err(Error::Chdr("packet is shorter than the CHDR header".into()));
    }
    let header = read_word(bytes, 0)?;
    let packet_len = (header & LENGTH_MASK) as usize;
    if packet_len < 8 || packet_len > bytes.len() {
        return Err(Error::Chdr(format!(
            "header declares {packet_len} bytes but buffer contains {}",
            bytes.len()
        )));
    }
    let has_time = header & HAS_TIME != 0;
    let payload_offset = if has_time { 16 } else { 8 };
    if packet_len < payload_offset {
        return Err(Error::Chdr("timestamp flag exceeds packet length".into()));
    }
    let timestamp = if has_time {
        Some((u64::from(read_word(bytes, 8)?) << 32) | u64::from(read_word(bytes, 12)?))
    } else {
        None
    };
    Ok(Packet {
        context: header & CONTEXT != 0,
        end_of_burst: header & END_OF_BURST != 0,
        sequence: ((header >> 16) as u16) & SEQUENCE_MASK,
        stream_id: read_word(bytes, 4)?,
        timestamp,
        payload: &bytes[payload_offset..packet_len],
    })
}

fn push_word(output: &mut Vec<u8>, word: u32) {
    output.extend_from_slice(&word.to_le_bytes());
}

fn split_u64(value: u64) -> (u32, u32) {
    let bytes = value.to_le_bytes();
    (
        u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
        u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
    )
}

fn read_word(input: &[u8], offset: usize) -> Result<u32> {
    let word = input
        .get(offset..offset + 4)
        .ok_or_else(|| Error::Chdr("truncated 32-bit word".into()))?;
    Ok(u32::from_le_bytes(
        word.try_into().expect("four-byte slice"),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_round_trip() {
        let encoded = encode_control(0x40, 0xabc, 32, 7, Some(0x1122_3344_5566_7788));
        let parsed = parse(&encoded).unwrap();
        assert!(parsed.context);
        assert_eq!(parsed.sequence, 0xabc);
        assert_eq!(parsed.stream_id, 0x40);
        assert_eq!(parsed.timestamp, Some(0x1122_3344_5566_7788));
        assert_eq!(parsed.payload, &[32, 0, 0, 0, 7, 0, 0, 0]);
    }

    #[test]
    fn data_preserves_non_word_payload_length() {
        let encoded = encode_data(0xa0, 1, &[1, 2, 3], None, true).unwrap();
        let parsed = parse(&encoded).unwrap();
        assert_eq!(parsed.payload, &[1, 2, 3]);
        assert!(parsed.end_of_burst);
    }

    #[test]
    fn rejects_fragment() {
        let mut encoded = encode_control(0x40, 0, 0, 0, None);
        encoded[0] = 64;
        assert!(parse(&encoded).is_err());
    }
}
