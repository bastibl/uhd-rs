use std::time::Duration;

use futures_lite::future;
use futures_timer::Delay;
use nusb::transfer::{Buffer, Bulk, In, Out};

use crate::{Error, Result, chdr};

const DATA_OUT_INTERFACE: u8 = 1;
const DATA_IN_INTERFACE: u8 = 2;
const CONTROL_OUT_INTERFACE: u8 = 3;
const CONTROL_IN_INTERFACE: u8 = 4;
const DATA_OUT_ENDPOINT: u8 = 0x02;
const DATA_IN_ENDPOINT: u8 = 0x86;
const CONTROL_OUT_ENDPOINT: u8 = 0x04;
const CONTROL_IN_ENDPOINT: u8 = 0x88;
const CONTROL_RESPONSE_TIMEOUT: Duration = Duration::from_secs(2);

/// Well-known B2xx CHDR stream identifiers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StreamId {
    LocalControl,
    RadioControl(u8),
    TxData(u8),
    RxData(u8),
    GpsUart,
}

impl StreamId {
    #[must_use]
    pub const fn value(self) -> u32 {
        match self {
            Self::LocalControl => 0x0000_0040,
            Self::RadioControl(0) => 0x0000_0010,
            Self::RadioControl(_) => 0x0000_0020,
            Self::TxData(0) => 0x0000_0050,
            Self::TxData(_) => 0x0000_0060,
            Self::RxData(0) => 0x0000_00a0,
            Self::RxData(_) => 0x0000_00b0,
            Self::GpsUart => 0x0000_0030,
        }
    }

    #[must_use]
    pub const fn response_value(self) -> u32 {
        self.value().rotate_left(16)
    }
}

/// Exclusive raw access to the two control and two sample bulk endpoints.
pub struct B2xxTransport {
    control_in: nusb::Endpoint<Bulk, In>,
    control_out: nusb::Endpoint<Bulk, Out>,
    data_in: nusb::Endpoint<Bulk, In>,
    data_out: nusb::Endpoint<Bulk, Out>,
}

impl std::fmt::Debug for B2xxTransport {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("B2xxTransport")
            .field("control_in_packet_size", &self.control_in.max_packet_size())
            .field("data_in_packet_size", &self.data_in.max_packet_size())
            .finish_non_exhaustive()
    }
}

impl B2xxTransport {
    pub(crate) async fn open(device: &nusb::Device) -> Result<Self> {
        let data_out_interface = device
            .detach_and_claim_interface(DATA_OUT_INTERFACE)
            .await?;
        let data_in_interface = device.detach_and_claim_interface(DATA_IN_INTERFACE).await?;
        let control_out_interface = device
            .detach_and_claim_interface(CONTROL_OUT_INTERFACE)
            .await?;
        let control_in_interface = device
            .detach_and_claim_interface(CONTROL_IN_INTERFACE)
            .await?;

        Ok(Self {
            data_out: data_out_interface.endpoint::<Bulk, Out>(DATA_OUT_ENDPOINT)?,
            data_in: data_in_interface.endpoint::<Bulk, In>(DATA_IN_ENDPOINT)?,
            control_out: control_out_interface.endpoint::<Bulk, Out>(CONTROL_OUT_ENDPOINT)?,
            control_in: control_in_interface.endpoint::<Bulk, In>(CONTROL_IN_ENDPOINT)?,
        })
    }

    /// Send one complete transfer to the FPGA control path.
    pub async fn send_control(&mut self, bytes: Vec<u8>) -> Result<()> {
        transfer_out(&mut self.control_out, bytes).await
    }

    /// Receive from the FPGA control path.
    pub async fn receive_control(&mut self) -> Result<Vec<u8>> {
        let size = self.control_in.max_packet_size() * 2;
        transfer_in(&mut self.control_in, size).await
    }

    fn cancel_control_receive(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        self.control_in.cancel_all();
    }

    /// Send CHDR sample data to the FPGA.
    pub async fn send_data(&mut self, bytes: Vec<u8>) -> Result<()> {
        transfer_out(&mut self.data_out, bytes).await
    }

    /// Receive CHDR sample data from the FPGA.
    ///
    /// `nusb` 0.2 requires `requested_length` to be a multiple of the endpoint
    /// packet size. B2xx's established optimal receive lengths (8176 or 16360)
    /// intentionally are not multiples, so callers must currently choose an
    /// aligned size here. See the repository README for the resulting streaming
    /// limitation.
    pub async fn receive_data(&mut self, requested_length: usize) -> Result<Vec<u8>> {
        let packet_size = self.data_in.max_packet_size();
        if requested_length == 0 || requested_length % packet_size != 0 {
            return Err(Error::InvalidArgument(format!(
                "nusb requires the IN length to be a nonzero multiple of endpoint packet size {packet_size}"
            )));
        }
        transfer_in(&mut self.data_in, requested_length).await
    }

    #[must_use]
    pub const fn into_radio_control(self, stream: StreamId) -> RadioControl {
        RadioControl {
            transport: self,
            stream,
            sequence: 0,
            command_time: None,
        }
    }
}

/// Serialized Wishbone register transactions over the B2xx control stream.
pub struct RadioControl {
    transport: B2xxTransport,
    stream: StreamId,
    sequence: u16,
    command_time: Option<u64>,
}

impl RadioControl {
    #[must_use]
    pub const fn stream(&self) -> StreamId {
        self.stream
    }

    pub fn set_stream(&mut self, stream: StreamId) {
        self.stream = stream;
        self.sequence = 0;
    }

    pub fn set_command_time(&mut self, ticks: Option<u64>) {
        self.command_time = ticks;
    }

    #[must_use]
    pub const fn transport_mut(&mut self) -> &mut B2xxTransport {
        &mut self.transport
    }

    #[must_use]
    pub fn into_transport(self) -> B2xxTransport {
        self.transport
    }

    pub async fn poke32(&mut self, byte_address: u32, value: u32) -> Result<()> {
        if byte_address % 4 != 0 {
            return Err(Error::InvalidArgument(
                "poke32 address must be 4-byte aligned".into(),
            ));
        }
        self.transaction(byte_address / 4, value).await.map(|_| ())
    }

    pub async fn peek32(&mut self, byte_address: u32) -> Result<u32> {
        if byte_address % 4 != 0 {
            return Err(Error::InvalidArgument(
                "peek32 address must be 4-byte aligned".into(),
            ));
        }
        let value = self.transaction(32, byte_address / 8).await?;
        let words = value.to_le_bytes();
        Ok(if (byte_address / 4) & 1 == 0 {
            u32::from_le_bytes([words[0], words[1], words[2], words[3]])
        } else {
            u32::from_le_bytes([words[4], words[5], words[6], words[7]])
        })
    }

    pub async fn peek64(&mut self, byte_address: u32) -> Result<u64> {
        if byte_address % 8 != 0 {
            return Err(Error::InvalidArgument(
                "peek64 address must be 8-byte aligned".into(),
            ));
        }
        self.transaction(32, byte_address / 8).await
    }

    async fn transaction(&mut self, address: u32, value: u32) -> Result<u64> {
        let sequence = self.sequence & 0x0fff;
        self.sequence = self.sequence.wrapping_add(1) & 0x0fff;
        let request = chdr::encode_control(
            self.stream.value(),
            sequence,
            address,
            value,
            self.command_time,
        );
        self.transport.send_control(request).await?;

        loop {
            let response = future::race(self.transport.receive_control(), async {
                Delay::new(CONTROL_RESPONSE_TIMEOUT).await;
                Err(Error::ControlTimeout {
                    timeout: CONTROL_RESPONSE_TIMEOUT,
                })
            })
            .await;
            let response = match response {
                Ok(response) => response,
                Err(error @ Error::ControlTimeout { .. }) => {
                    self.transport.cancel_control_receive();
                    return Err(error);
                }
                Err(error) => return Err(error),
            };
            let packet = chdr::parse(&response)?;
            if packet.stream_id != self.stream.response_value() || packet.sequence != sequence {
                continue;
            }
            if !packet.context || packet.payload.len() != 8 {
                return Err(Error::Chdr(
                    "register response is not an eight-byte context payload".into(),
                ));
            }
            let high = u64::from(u32::from_le_bytes(
                packet.payload[0..4]
                    .try_into()
                    .expect("four-byte high word"),
            ));
            let low = u64::from(u32::from_le_bytes(
                packet.payload[4..8].try_into().expect("four-byte low word"),
            ));
            return Ok((high << 32) | low);
        }
    }
}

async fn transfer_out(endpoint: &mut nusb::Endpoint<Bulk, Out>, bytes: Vec<u8>) -> Result<()> {
    let expected = bytes.len();
    endpoint.submit(bytes.into());
    let completion = endpoint.next_complete().await;
    completion.status?;
    if completion.actual_len != expected {
        return Err(Error::ShortTransfer {
            expected,
            actual: completion.actual_len,
        });
    }
    Ok(())
}

async fn transfer_in(
    endpoint: &mut nusb::Endpoint<Bulk, In>,
    requested_length: usize,
) -> Result<Vec<u8>> {
    endpoint.submit(Buffer::new(requested_length));
    let completion = endpoint.next_complete().await;
    completion.status?;
    Ok(completion.buffer.into_vec())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stream_ids_match_b2xx_routing() {
        assert_eq!(StreamId::LocalControl.value(), 0x40);
        assert_eq!(StreamId::LocalControl.response_value(), 0x0040_0000);
        assert_eq!(StreamId::RadioControl(1).value(), 0x20);
        assert_eq!(StreamId::RxData(0).value(), 0xa0);
    }
}
