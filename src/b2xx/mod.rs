//! Support for the USRP B200, B210, B200mini, and B205mini.

mod device;
mod fx3;
mod session;
mod spi;
mod transport;

pub use device::{B2xxDeviceInfo, list_devices, request_device};
pub use fx3::{B2xxDevice, B2xxIdentity, FirmwareCompatibility, Fx3State, LoadOutcome, UsbSpeed};
pub use session::{B2xxSession, FpgaCompatibility};
pub use spi::{Ad9361Io, B2xxSpi, SpiConfig, SpiEdge};
pub use transport::{B2xxTransport, RadioControl, StreamId};

pub const ETTUS_VENDOR_ID: u16 = 0x2500;
pub const NI_VENDOR_ID: u16 = 0x3923;
pub const CYPRESS_VENDOR_ID: u16 = 0x04b4;

pub const B200_PRODUCT_ID: u16 = 0x0020;
pub const B200MINI_PRODUCT_ID: u16 = 0x0021;
pub const B205MINI_PRODUCT_ID: u16 = 0x0022;
pub const NI_B200_PRODUCT_ID: u16 = 0x7813;
pub const NI_B210_PRODUCT_ID: u16 = 0x7814;
pub const CYPRESS_BOOT_PRODUCT_ID: u16 = 0x00f3;
pub const CYPRESS_REENUM_PRODUCT_ID: u16 = 0x00f0;

pub(crate) const SUPPORTED_IDS: &[(u16, u16)] = &[
    (ETTUS_VENDOR_ID, B200_PRODUCT_ID),
    (ETTUS_VENDOR_ID, B200MINI_PRODUCT_ID),
    (ETTUS_VENDOR_ID, B205MINI_PRODUCT_ID),
    (NI_VENDOR_ID, NI_B200_PRODUCT_ID),
    (NI_VENDOR_ID, NI_B210_PRODUCT_ID),
    (CYPRESS_VENDOR_ID, CYPRESS_BOOT_PRODUCT_ID),
    (CYPRESS_VENDOR_ID, CYPRESS_REENUM_PRODUCT_ID),
];

/// A member of the B2xx product family.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Product {
    B200,
    B210,
    B200Mini,
    B205Mini,
}

impl Product {
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::B200 => "B200",
            Self::B210 => "B210",
            Self::B200Mini => "B200mini",
            Self::B205Mini => "B205mini",
        }
    }

    #[must_use]
    pub const fn fpga_filename(self) -> &'static str {
        match self {
            Self::B200 => "usrp_b200_fpga.bin",
            Self::B210 => "usrp_b210_fpga.bin",
            Self::B200Mini => "usrp_b200mini_fpga.bin",
            Self::B205Mini => "usrp_b205mini_fpga.bin",
        }
    }

    #[must_use]
    pub const fn fpga_compatibility(self) -> u16 {
        match self {
            Self::B205Mini => 7,
            Self::B200 | Self::B210 | Self::B200Mini => 16,
        }
    }

    pub(crate) const fn from_usb_id(vendor_id: u16, product_id: u16) -> Option<Self> {
        match (vendor_id, product_id) {
            (ETTUS_VENDOR_ID, B200MINI_PRODUCT_ID) => Some(Self::B200Mini),
            (ETTUS_VENDOR_ID, B205MINI_PRODUCT_ID) => Some(Self::B205Mini),
            (NI_VENDOR_ID, NI_B200_PRODUCT_ID) => Some(Self::B200),
            (NI_VENDOR_ID, NI_B210_PRODUCT_ID) => Some(Self::B210),
            _ => None,
        }
    }

    pub(crate) const fn from_eeprom_code(code: u16) -> Option<Self> {
        match code {
            0x0001 | 0x7737 | NI_B200_PRODUCT_ID => Some(Self::B200),
            0x0002 | 0x7738 | NI_B210_PRODUCT_ID => Some(Self::B210),
            0x0003 | 0x7739 => Some(Self::B200Mini),
            0x0004 | 0x773a => Some(Self::B205Mini),
            _ => None,
        }
    }
}

impl std::fmt::Display for Product {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(self.name())
    }
}
