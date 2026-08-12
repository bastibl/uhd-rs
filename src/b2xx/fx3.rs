use std::time::Duration;

use futures_timer::Delay;
use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};

use super::{B2xxDeviceInfo, Product, SUPPORTED_IDS};
use crate::{Error, Result, ihex};

const FIRMWARE_LOAD: u8 = 0xa0;
const FPGA_START: u8 = 0x02;
const FPGA_DATA: u8 = 0x12;
const GET_COMPAT: u8 = 0x15;
const SET_FPGA_HASH: u8 = 0x1c;
const GET_FPGA_HASH: u8 = 0x1d;
const SET_FW_HASH: u8 = 0x1e;
const GET_FW_HASH: u8 = 0x1f;
const LOOP: u8 = 0x22;
const FPGA_CONFIG: u8 = 0x55;
const GPIF_RESET: u8 = 0x72;
const GET_USB_SPEED: u8 = 0x80;
const GET_STATUS: u8 = 0x83;
const FX3_RESET: u8 = 0x99;
const EEPROM_WRITE: u8 = 0xba;
const EEPROM_READ: u8 = 0xbb;

const CONTROL_TIMEOUT: Duration = Duration::from_secs(1);
const IMAGE_TIMEOUT: Duration = Duration::from_secs(5);
const POLL_INTERVAL: Duration = Duration::from_millis(10);

/// Firmware ABI required by this implementation.
pub const REQUIRED_FIRMWARE: FirmwareCompatibility = FirmwareCompatibility { major: 8, minor: 0 };

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FirmwareCompatibility {
    pub major: u8,
    pub minor: u8,
}

/// USB link speed reported by the FX3 firmware.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UsbSpeed {
    HighSpeed,
    SuperSpeed,
}

impl UsbSpeed {
    #[must_use]
    pub const fn major_version(self) -> u8 {
        match self {
            Self::HighSpeed => 2,
            Self::SuperSpeed => 3,
        }
    }

    #[must_use]
    pub const fn control_transfer_size(self) -> usize {
        match self {
            Self::HighSpeed => 64,
            Self::SuperSpeed => 512,
        }
    }
}

/// State of FPGA configuration in the FX3 firmware.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Fx3State {
    Undefined,
    FpgaReady,
    ConfiguringFpga,
    Busy,
    Running,
    Unconfigured,
    Error,
    Unknown(u8),
}

impl From<u8> for Fx3State {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Undefined,
            1 => Self::FpgaReady,
            2 => Self::ConfiguringFpga,
            3 => Self::Busy,
            4 => Self::Running,
            5 => Self::Unconfigured,
            6 => Self::Error,
            other => Self::Unknown(other),
        }
    }
}

impl std::fmt::Display for Fx3State {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Undefined => formatter.write_str("undefined"),
            Self::FpgaReady => formatter.write_str("FPGA ready"),
            Self::ConfiguringFpga => formatter.write_str("configuring FPGA"),
            Self::Busy => formatter.write_str("busy"),
            Self::Running => formatter.write_str("running"),
            Self::Unconfigured => formatter.write_str("unconfigured"),
            Self::Error => formatter.write_str("error"),
            Self::Unknown(value) => write!(formatter, "unknown ({value})"),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LoadOutcome {
    AlreadyLoaded,
    Loaded,
}

/// Parsed motherboard identity stored in the B2xx EEPROM.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct B2xxIdentity {
    pub eeprom_revision: u16,
    pub revision: u16,
    pub product_code: u16,
    pub product: Option<Product>,
    pub name: String,
    pub serial: String,
    pub vendor_id: Option<u16>,
    pub product_id: Option<u16>,
}

/// An open B2xx FX3 control connection.
pub struct B2xxDevice {
    info: B2xxDeviceInfo,
    usb: nusb::Device,
    control: nusb::Interface,
}

impl std::fmt::Debug for B2xxDevice {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("B2xxDevice")
            .field("info", &self.info)
            .finish_non_exhaustive()
    }
}

impl B2xxDevice {
    pub(crate) async fn open(info: B2xxDeviceInfo) -> Result<Self> {
        if !SUPPORTED_IDS.contains(&(info.vendor_id, info.product_id)) {
            return Err(Error::UnsupportedDevice {
                vendor_id: info.vendor_id,
                product_id: info.product_id,
            });
        }
        let usb = info.nusb_info().open().await?;
        let control = usb.detach_and_claim_interface(0).await?;
        Ok(Self { info, usb, control })
    }

    #[must_use]
    pub const fn info(&self) -> &B2xxDeviceInfo {
        &self.info
    }

    pub async fn usb_speed(&self) -> Result<UsbSpeed> {
        match self
            .control_read(GET_USB_SPEED, 0, 0, 1, CONTROL_TIMEOUT)
            .await?[0]
        {
            2 => Ok(UsbSpeed::HighSpeed),
            3 => Ok(UsbSpeed::SuperSpeed),
            other => Err(Error::InvalidArgument(format!(
                "FX3 returned invalid USB speed {other}"
            ))),
        }
    }

    pub async fn fx3_state(&self) -> Result<Fx3State> {
        Ok(self
            .control_read(GET_STATUS, 0, 0, 1, CONTROL_TIMEOUT)
            .await?[0]
            .into())
    }

    pub async fn firmware_compatibility(&self) -> Result<FirmwareCompatibility> {
        let value = self
            .control_read(GET_COMPAT, 0, 0, 2, CONTROL_TIMEOUT)
            .await?;
        Ok(FirmwareCompatibility {
            major: value[0],
            minor: value[1],
        })
    }

    pub async fn check_firmware_compatibility(&self) -> Result<FirmwareCompatibility> {
        let actual = self.firmware_compatibility().await?;
        if actual.major != REQUIRED_FIRMWARE.major {
            return Err(Error::FirmwareCompatibility {
                expected_major: REQUIRED_FIRMWARE.major,
                expected_minor: REQUIRED_FIRMWARE.minor,
                actual_major: actual.major,
                actual_minor: actual.minor,
            });
        }
        Ok(actual)
    }

    /// Load a Cypress FX3 Intel HEX image. The USB device normally
    /// re-enumerates after the execute record, invalidating this handle.
    pub async fn load_firmware(&self, image: &[u8]) -> Result<()> {
        if self.info.firmware_loaded {
            return Err(Error::InvalidArgument(
                "FX3 firmware can only be loaded while the device is in its bootloader".into(),
            ));
        }
        let segments = ihex::parse(image)?;
        for segment in segments {
            let address = segment.address.to_le_bytes();
            self.control_write(
                FIRMWARE_LOAD,
                u16::from_le_bytes([address[0], address[1]]),
                u16::from_le_bytes([address[2], address[3]]),
                &segment.data,
                CONTROL_TIMEOUT,
            )
            .await?;
        }
        Ok(())
    }

    pub async fn reset_fx3(&self) -> Result<()> {
        self.control_write(FX3_RESET, 0, 0, &[0; 4], CONTROL_TIMEOUT)
            .await
    }

    pub async fn reset_gpif(&self) -> Result<()> {
        self.control_write(GPIF_RESET, 0, 0, &[0; 4], CONTROL_TIMEOUT)
            .await
    }

    /// Load a raw B2xx FPGA bitstream through FX3 vendor requests.
    pub async fn load_fpga(&self, image: &[u8], force: bool) -> Result<LoadOutcome> {
        if image.is_empty() {
            return Err(Error::InvalidArgument("FPGA image is empty".into()));
        }
        let hash = image_hash(image);
        if !force && self.fpga_hash().await? == hash {
            return Ok(LoadOutcome::AlreadyLoaded);
        }

        let transfer_size = self.usb_speed().await?.control_transfer_size();
        self.control_read(LOOP, 0, 0, transfer_size, CONTROL_TIMEOUT)
            .await?;
        self.set_fpga_hash(0).await?;
        self.control_write(FPGA_CONFIG, 0, 0, &[0], CONTROL_TIMEOUT)
            .await?;
        self.wait_for_state(Fx3State::FpgaReady, IMAGE_TIMEOUT)
            .await?;
        self.control_write(FPGA_START, 0, 0, &[0], CONTROL_TIMEOUT)
            .await?;
        self.wait_for_state(Fx3State::ConfiguringFpga, Duration::from_secs(10))
            .await?;
        for chunk in image.chunks(transfer_size) {
            self.control_write(FPGA_DATA, 0, 0, chunk, IMAGE_TIMEOUT)
                .await?;
        }
        self.wait_for_state(Fx3State::Running, IMAGE_TIMEOUT)
            .await?;
        self.set_fpga_hash(hash).await?;
        Ok(LoadOutcome::Loaded)
    }

    pub async fn firmware_hash(&self) -> Result<u32> {
        self.read_hash(GET_FW_HASH).await
    }

    pub async fn set_firmware_hash(&self, hash: u32) -> Result<()> {
        self.control_write(SET_FW_HASH, 0, 0, &hash.to_le_bytes(), CONTROL_TIMEOUT)
            .await
    }

    pub async fn fpga_hash(&self) -> Result<u32> {
        self.read_hash(GET_FPGA_HASH).await
    }

    pub async fn set_fpga_hash(&self, hash: u32) -> Result<()> {
        self.control_write(SET_FPGA_HASH, 0, 0, &hash.to_le_bytes(), CONTROL_TIMEOUT)
            .await
    }

    pub async fn read_eeprom(&self, address: u16, length: usize) -> Result<Vec<u8>> {
        if length > usize::from(u16::MAX) {
            return Err(Error::InvalidArgument("EEPROM read is too large".into()));
        }
        let device_address = address >> 8;
        let offset = address & 0xff;
        self.control_read(
            EEPROM_READ,
            0,
            offset | (device_address << 8),
            length,
            CONTROL_TIMEOUT,
        )
        .await
    }

    pub async fn write_eeprom(&self, address: u16, bytes: &[u8]) -> Result<()> {
        if bytes.len() > usize::from(u16::MAX) {
            return Err(Error::InvalidArgument("EEPROM write is too large".into()));
        }
        let device_address = address >> 8;
        let offset = address & 0xff;
        self.control_write(
            EEPROM_WRITE,
            0,
            offset | (device_address << 8),
            bytes,
            CONTROL_TIMEOUT,
        )
        .await
    }

    pub async fn identity(&self) -> Result<B2xxIdentity> {
        const REV0_SIGNATURE: u32 = 0xb214_5943;
        const REV1_SIGNATURE: u32 = 0xb01a_5943;
        let signature_bytes = self.read_eeprom(0, 4).await?;
        let signature =
            u32::from_le_bytes(signature_bytes.try_into().map_err(|bytes: Vec<u8>| {
                Error::ShortTransfer {
                    expected: 4,
                    actual: bytes.len(),
                }
            })?);
        match signature {
            REV0_SIGNATURE => {
                let bytes = self.read_eeprom(0x04dc, 36).await?;
                Ok(identity_from_fields(
                    0,
                    &bytes,
                    IdentityLayout {
                        revision: 0,
                        product: 2,
                        name: 4,
                        serial: 27,
                    },
                    None,
                    None,
                ))
            }
            REV1_SIGNATURE => {
                let bytes = self.read_eeprom(0x7f00, 46).await?;
                let magic = field_u16(&bytes, 0);
                if magic != 0xb200 {
                    return Err(Error::EepromField {
                        field: "magic",
                        actual: magic,
                        expected: 0xb200,
                    });
                }
                let compatibility = field_u16(&bytes, 4);
                if compatibility != 1 {
                    return Err(Error::EepromField {
                        field: "compatibility",
                        actual: compatibility,
                        expected: 1,
                    });
                }
                Ok(identity_from_fields(
                    field_u16(&bytes, 2),
                    &bytes,
                    IdentityLayout {
                        revision: 10,
                        product: 12,
                        name: 14,
                        serial: 37,
                    },
                    Some(field_u16(&bytes, 6)),
                    Some(field_u16(&bytes, 8)),
                ))
            }
            other => Err(Error::EepromSignature(other)),
        }
    }

    /// Claim the four FPGA bulk interfaces and create the raw transport.
    pub async fn open_transport(&self) -> Result<super::B2xxTransport> {
        super::B2xxTransport::open(&self.usb).await
    }

    /// Load and validate the FPGA image, then open a checked FPGA session.
    pub async fn start(
        self,
        fpga_image: &[u8],
        force: bool,
    ) -> Result<(super::B2xxSession, LoadOutcome)> {
        super::B2xxSession::start(self, fpga_image, force).await
    }

    async fn wait_for_state(&self, expected: Fx3State, timeout: Duration) -> Result<()> {
        let mut remaining = timeout;
        loop {
            let actual = self.fx3_state().await?;
            if actual == expected {
                return Ok(());
            }
            if matches!(actual, Fx3State::Error | Fx3State::Undefined) {
                return Err(Error::Fx3State(actual));
            }
            if remaining.is_zero() {
                return Err(Error::Fx3Timeout { expected, timeout });
            }
            let delay = remaining.min(POLL_INTERVAL);
            Delay::new(delay).await;
            remaining = remaining.saturating_sub(delay);
        }
    }

    async fn read_hash(&self, request: u8) -> Result<u32> {
        let bytes = self.control_read(request, 0, 0, 4, CONTROL_TIMEOUT).await?;
        Ok(u32::from_le_bytes(
            bytes.try_into().expect("four-byte hash"),
        ))
    }

    async fn control_read(
        &self,
        request: u8,
        value: u16,
        index: u16,
        length: usize,
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        let length = u16::try_from(length)
            .map_err(|_| Error::InvalidArgument("control transfer is too large".into()))?;
        let bytes = self
            .control
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    length,
                },
                timeout,
            )
            .await?;
        if bytes.len() != usize::from(length) {
            return Err(Error::ShortTransfer {
                expected: usize::from(length),
                actual: bytes.len(),
            });
        }
        Ok(bytes)
    }

    async fn control_write(
        &self,
        request: u8,
        value: u16,
        index: u16,
        data: &[u8],
        timeout: Duration,
    ) -> Result<()> {
        if data.len() > usize::from(u16::MAX) {
            return Err(Error::InvalidArgument(
                "control transfer is too large".into(),
            ));
        }
        self.control
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    data,
                },
                timeout,
            )
            .await?;
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct IdentityLayout {
    revision: usize,
    product: usize,
    name: usize,
    serial: usize,
}

fn identity_from_fields(
    eeprom_revision: u16,
    bytes: &[u8],
    layout: IdentityLayout,
    vendor_id: Option<u16>,
    product_id: Option<u16>,
) -> B2xxIdentity {
    let product_code = field_u16(bytes, layout.product);
    B2xxIdentity {
        eeprom_revision,
        revision: field_u16(bytes, layout.revision),
        product_code,
        product: Product::from_eeprom_code(product_code),
        name: field_string(&bytes[layout.name..layout.name + 23]),
        serial: field_string(&bytes[layout.serial..layout.serial + 9]),
        vendor_id,
        product_id,
    }
}

fn field_u16(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

fn field_string(bytes: &[u8]) -> String {
    let end = bytes
        .iter()
        .position(|byte| matches!(byte, 0 | 0xff))
        .unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).trim().to_owned()
}

/// UHD's stable 32-bit image identity hash (derived from `hash_combine`).
#[must_use]
pub fn image_hash(bytes: &[u8]) -> u32 {
    let mut hash = 0_u32;
    for &byte in bytes {
        let signed = if byte & 0x80 == 0 {
            u32::from(byte)
        } else {
            0xffff_ff00 | u32::from(byte)
        };
        hash ^= signed
            .wrapping_add(0x9e37_79b9)
            .wrapping_add(hash << 6)
            .wrapping_add(hash >> 2);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_revision_one_identity() {
        let mut bytes = [0_u8; 46];
        bytes[0..2].copy_from_slice(&0xb200_u16.to_le_bytes());
        bytes[2..4].copy_from_slice(&1_u16.to_le_bytes());
        bytes[4..6].copy_from_slice(&1_u16.to_le_bytes());
        bytes[6..8].copy_from_slice(&0x2500_u16.to_le_bytes());
        bytes[8..10].copy_from_slice(&0x20_u16.to_le_bytes());
        bytes[10..12].copy_from_slice(&5_u16.to_le_bytes());
        bytes[12..14].copy_from_slice(&1_u16.to_le_bytes());
        bytes[14..18].copy_from_slice(b"lab\0");
        bytes[37..41].copy_from_slice(b"ABC\0");
        let identity = identity_from_fields(
            1,
            &bytes,
            IdentityLayout {
                revision: 10,
                product: 12,
                name: 14,
                serial: 37,
            },
            Some(0x2500),
            Some(0x20),
        );
        assert_eq!(identity.product, Some(Product::B200));
        assert_eq!(identity.name, "lab");
        assert_eq!(identity.serial, "ABC");
    }

    #[test]
    fn image_hash_is_sensitive_to_signed_bytes() {
        assert_ne!(image_hash(&[0x7f]), image_hash(&[0xff]));
        assert_eq!(image_hash(&[]), 0);
    }
}
