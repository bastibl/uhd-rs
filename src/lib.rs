//! Pure Rust B2xx discovery, image loading and owned B2xx RX streams.
//!
//! [`Device::builder`] prepares a radio without starting reception. Claim an
//! [`RxStream`], explicitly start it, and read [`Complex32`] samples into your
//! own buffers. Operations implement [`MaybeFuture`]: `.wait()` natively or
//! `.await` natively and in wasm. Close streams and shut down devices explicitly
//! to observe cleanup errors; drop also attempts cleanup.
//!
//! The default `embedded-images` feature includes all six pinned UHD B2xx
//! images. B200, B210, B200mini and B205mini support RX channel zero on RX2.
//! The [`b2xx`] module also exposes image loading and diagnostics.

pub mod b2xx;
pub mod chdr;
mod error;
pub mod ihex;

mod high_level;
pub mod images;
mod operation;
pub use b2xx::{RxConfig, RxGain, RxTuneRequest, RxTuneResult};
pub use error::{Error, Result};
pub use high_level::{Device, DeviceBuilder, DeviceDescriptor, RxStream, StreamingStats};
pub use num_complex::Complex32;
pub use nusb::MaybeFuture;
#[cfg(target_arch = "wasm32")]
mod browser_usb;
