//! A pure Rust implementation of the host side of UHD.
//!
//! The first supported hardware family is the USB-connected USRP B2xx. The
//! transport is asynchronous from the bottom up so the same code can run on
//! native hosts through `nusb` and in a browser through `WebUSB`.
//!
//! This crate currently provides device discovery, `WebUSB` permission requests,
//! FX3 and FPGA image loading, motherboard EEPROM identity, raw B2xx bulk
//! transport, CHDR packet framing, Wishbone register access, and pure-Rust
//! AD9364 cold-start initialization for revision 5 or newer B200 hardware. The
//! higher-level APIs include automatic radio initialization and channel-zero
//! receive streaming.

pub mod b2xx;
pub mod chdr;
mod error;
pub mod ihex;

#[cfg(all(target_arch = "wasm32", feature = "wasm"))]
mod wasm;

pub use error::{Error, Result};
