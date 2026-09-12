use std::time::Duration;

/// Errors returned by the pure-Rust UHD implementation.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("a stream or control operation still owns the device")]
    Busy,
    #[error("device shutdown has started")]
    Shutdown,
    #[error("RX stream is stopped or failed")]
    StreamClosed,
    #[error("operation timed out")]
    Timeout,
    #[error("image {0} is required; enable embedded-images or supply bytes on DeviceBuilder")]
    MissingImage(&'static str),
    #[error(
        "request WebUSB permission again and select the reconnected device from a user gesture"
    )]
    PermissionRequired,
    #[error("reopen the device before continuing RX")]
    ReopenRequired,
    #[error("WebUSB: {0}")]
    Browser(String),

    #[error("USB error: {0}")]
    Usb(#[from] nusb::Error),

    #[error("USB transfer error: {0}")]
    Transfer(#[from] nusb::transfer::TransferError),

    #[error("no supported USRP B2xx device was found")]
    DeviceNotFound,

    #[error("timed out after {timeout:?} waiting for the B2xx to {state}")]
    DeviceReenumerationTimeout {
        state: &'static str,
        timeout: Duration,
    },

    #[error("the device returned {actual} bytes, expected {expected}")]
    ShortTransfer { expected: usize, actual: usize },

    #[error("invalid Intel HEX image at line {line}: {message}")]
    IntelHex { line: usize, message: String },

    #[error("invalid CHDR packet: {0}")]
    Chdr(String),

    #[error("receive overflow: expected CHDR sequence {expected}, got {actual}")]
    ReceiveOverflow { expected: u16, actual: u16 },

    #[error("the B2xx reported a receive FIFO overflow at CHDR sequence {sequence}")]
    DeviceReceiveOverflow { sequence: u16 },

    #[error("the B2xx reported receive context code 0x{code:02x} at CHDR sequence {sequence}")]
    ReceiveContext { code: u8, sequence: u16 },

    #[error("timed out after {timeout:?} waiting for an FPGA control response")]
    ControlTimeout { timeout: Duration },

    #[error("unsupported B2xx device {vendor_id:04x}:{product_id:04x}")]
    UnsupportedDevice { vendor_id: u16, product_id: u16 },

    #[error("unsupported operation: {0}")]
    Unsupported(&'static str),

    #[error("FX3 entered state {0}")]
    Fx3State(crate::b2xx::Fx3State),

    #[error("timed out after {timeout:?} waiting for FX3 state {expected}")]
    Fx3Timeout {
        expected: crate::b2xx::Fx3State,
        timeout: Duration,
    },

    #[error(
        "firmware compatibility mismatch: device is {actual_major}.{actual_minor}, host requires {expected_major}.{expected_minor}; use DeviceBuilder::reload_firmware(true) to reload"
    )]
    FirmwareCompatibility {
        expected_major: u8,
        expected_minor: u8,
        actual_major: u8,
        actual_minor: u8,
    },

    #[error("FPGA signature mismatch: got 0x{actual:08x}, expected 0x{expected:08x}")]
    FpgaSignature { expected: u32, actual: u32 },

    #[error("FPGA compatibility mismatch: device is {actual}, host requires {expected}")]
    FpgaCompatibility { expected: u16, actual: u16 },

    #[error("FPGA reported an invalid number of radio chains: {0}")]
    RadioChainCount(u8),

    #[error("the AD9361 is in state 0x{state:x}; cold-start initialization is required")]
    Ad9361NotInitialized { state: u8 },

    #[error("timed out waiting for AD9361 state 0x{expected:x}")]
    Ad9361StateTimeout { expected: u8 },

    #[error("the AD9361 receive PLL did not lock at {frequency_hz} Hz")]
    Ad9361PllUnlocked { frequency_hz: f64 },

    #[error("AD9361 initialization failed during {stage}: {detail}")]
    Ad9361Initialization { stage: &'static str, detail: String },

    #[error("timed out during AD9361 {stage}")]
    Ad9361CalibrationTimeout { stage: &'static str },

    #[error(
        "AD9361 digital loopback failed: expected 0x{expected:08x}, got TX=0x{tx:08x}, RX=0x{rx:08x}"
    )]
    Ad9361Loopback { expected: u32, tx: u32, rx: u32 },

    #[error("motherboard EEPROM has an unknown signature 0x{0:08x}")]
    EepromSignature(u32),

    #[error(
        "motherboard EEPROM field {field} has invalid value 0x{actual:04x}, expected 0x{expected:04x}"
    )]
    EepromField {
        field: &'static str,
        actual: u16,
        expected: u16,
    },

    #[error("invalid argument: {0}")]
    InvalidArgument(String),

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
}

pub type Result<T, E = Error> = std::result::Result<T, E>;
