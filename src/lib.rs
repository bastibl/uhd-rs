//! Pure Rust B2xx discovery, image loading and owned B200 RX streams.
//!
//! [`Device::builder`] prepares a radio without starting reception. Claim an
//! [`RxStream`], explicitly start it, and read [`Complex32`] samples into your
//! own buffers. Operations implement [`MaybeFuture`]: `.wait()` natively or
//! `.await` natively and in wasm. Close streams and shut down devices explicitly
//! to observe cleanup errors; drop also attempts cleanup.
//!
//! The default `embedded-images` feature includes all six pinned UHD B2xx
//! images. Radio support is B200 revision 5+, RX channel zero. The [`b2xx`]
//! module retains image loading and diagnostics for all B2xx models.

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
