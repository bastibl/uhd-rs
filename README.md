# uhd-rs

[![CI](https://github.com/bastibl/uhd-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/bastibl/uhd-rs/actions/workflows/ci.yml)

A pure Rust USRP B2xx driver using [nusb](https://crates.io/crates/nusb), without libusb or C++ UHD. Native and browser applications use the same Rust API.

Radio support is **B200 revision 5+ RX, channel zero, RX2**. B210, B200mini and B205mini support discovery, firmware/FPGA loading and diagnostics; their radio initialization, TX and multichannel streaming are not implemented.

## Owned devices and streams

```rust,no_run
use std::time::Duration;
use uhd_rs::{Complex32, Device, MaybeFuture};

# fn main() -> uhd_rs::Result<()> {
let mut device = Device::builder().open().wait()?;
let mut rx = device.rx_stream()?;
rx.start().wait()?;
let mut samples = [Complex32::default(); 4096];
let count = rx.read(&mut samples, Some(Duration::from_secs(2))).wait()?;
println!("Received {count} samples");
rx.stop().wait()?;
rx.start().wait()?;
let stats = rx.close().wait()?;
device.shutdown().wait()?;
# Ok(())
# }
```

Opening prepares the radio at 100 MHz, 1 MS/s and manual gain 30 dB; it does not start RX. Discovery (`Device::list`), opening, configuration, stream operations and shutdown are lazy `MaybeFuture` operations: call `.wait()` natively, or `.await` natively/in wasm. Dropping an unpolled ordinary operation does nothing. Native futures are `Send`. The default native blocking API needs no async runtime. Optional `smol` and `tokio` features enable nusb's executor integration; with neither enabled, nusb discovery/open/interface/control operations use native blocking paths even when awaited. The shared radio algorithms do not depend on an executor.

Select a device with `.serial("...")` or `.descriptor(descriptor)`. Configure with builder methods `.frequency_hz(...)`, `.sample_rate_hz(...)`, `.gain(RxGain::Manual(...))`, `.gain(RxGain::Automatic)` and `.lo_offset_hz(...)`. After opening, use `set_center_frequency`, `tune(RxTuneRequest::with_lo_offset(...))`, `set_sample_rate` and `set_gain`. Tune and rate operations return actual quantized values. The FPGA DDC uses a 16 MHz master clock and supported integer decimation.

An `RxStream` owns its sample endpoint and persistent queue of 16 × 16,384-byte transfers. Ordinary reads do not acquire the radio control lock. Reads return available samples promptly, keep unread samples for the next call, and return `Error::Timeout` when no samples arrive before the deadline. CHDR loss and device overflow are recovered internally and counted in `StreamingStats`; malformed packets and fatal USB errors invalidate the stream. `None` waits indefinitely. `stop` retains the queue, and restart discards stale packets and resets CHDR synchronization.

Close streams explicitly to observe cleanup errors. `close` consumes the stream and returns an owned lazy operation; dropping or cancelling it still attempts cleanup. `shutdown` returns `Busy` while any stream (including a pending close) owns the claim. Once shutdown starts, configuration and new claims are disabled. Cleanup failures can be retried; successful shutdown is idempotent. A stream remains usable after dropping its `Device`. Final-owner drop attempts bounded synchronous cleanup natively and background cleanup in a browser.

## Images and startup

The default `embedded-images` feature embeds six assets in debug and release builds, including wasm and Cargo packages: the FX3 firmware, available bootloader, and B200/B210/B200mini/B205mini FPGA binaries. `rust-embed` uses `debug-embed`, compression and deterministic timestamps. Builds perform no image downloads and need no runtime image directory.

The pinned UHD 4.8 image set uses FPGA revision `c37b318` and firmware revision `7f7d016`. Archive/file checksums, source URLs, upstream notices and the maintainer refresh procedure are in [images/README.md](images/README.md).

Opening loads firmware if needed, reconnects, reads motherboard identity, selects the corresponding FPGA and validates compatibility before radio initialization. Compatible running firmware is reused. Use `.reload_firmware(true)` to explicitly reload firmware; malformed firmware is rejected before resetting a working device. FPGA loading is skipped only when the selected image hash matches and FX3 reports it running. Opening never installs the bootloader.

Override individual assets with `.image(Image::Firmware, bytes)` or supply an `ImageCatalog` with `.images(catalog)`. Overrides take precedence over embedded bytes. With `default-features = false`, supply any image that must be loaded. A running FPGA with the pinned catalog hash may be reused without its bytes.

Native reconnection matches the original physical connector and known running-device serial. Linux USB 2/3 companion ports are resolved through the kernel's `peer` links. There is no fallback to an unrelated device. Set `.reconnect_timeout(Duration::from_secs(...))` to change the default 10-second deadline.

## Rust in a browser

Use this crate as a Rust dependency in your wasm application. The former JavaScript wrapper and `/web` probe page have been removed. The legacy `wasm` feature is an empty compatibility feature; the target selects WebUSB automatically.

```rust,no_run
# #[cfg(target_arch = "wasm32")]
# async fn example() -> uhd_rs::Result<()> {
use uhd_rs::Device;
// Poll from a click/tap handler so the permission chooser has user activation.
let selected = Device::request_permission().await?
    .ok_or(uhd_rs::Error::PermissionRequired)?;
let mut device = Device::builder().descriptor(selected).open().await?;
let rx = device.rx_stream()?;
rx.close().await?;
device.shutdown().await?;
# Ok(())
# }
```

WebUSB requires a secure browser-window context. Set `--cfg=web_sys_unstable_apis` in the **application's** wasm Rust flags; dependencies' `.cargo/config.toml` files are not inherited. This repository supplies it for local builds:

```console
cargo check --target wasm32-unknown-unknown --all-targets
```

Permission requests are separate from opening. If a re-enumerated device cannot be identified among authorized devices, opening returns `Error::PermissionRequired`; request a new user gesture and select it again. Devices without a known serial cannot safely be matched across browser reconnection.

Stopping RX leaves the browser USB device open. After closing or losing a stream that submitted transfers, `rx_stream` returns `ReopenRequired`: call `shutdown`, then open a new `Device`. WebUSB cannot cancel individual transfers. Terminal shutdown awaits the retained browser `UsbDevice.close()`, aborting pending operations and releasing interfaces. Drop cleanup retains ownership until its background attempt finishes.

## Examples and diagnostics

```console
cargo run --release --example rx_100mhz -- capture.fc32
cargo run --features smol --example rx_async
cargo run -- list
cargo run -- probe
cargo run -- load-firmware images/assets/usrp_b200_fw.hex
cargo run -- load-fpga images/assets/usrp_b210_fpga.bin
cargo run -- peek 0x50
```

The capture example writes ten seconds of headerless interleaved little-endian `f32` IQ. Both examples close the stream and shut down the device explicitly, including after a receive error. Pass a serial after diagnostic command arguments when multiple devices are connected. `b2xx::load_firmware_and_reconnect` and `B2xxDevice::open_fpga_session` expose image/reconnection and checked local-control diagnostics without initializing an unsupported radio. Radio-register loopback needs the radio clock initialized.

B2xx normally uses unaligned 8176/16360-byte IN requests. nusb 0.2.7 requires requests aligned to endpoint packet size; the driver retains the existing 16384-byte workaround and accepts short completions. This limitation still needs B200 hardware RX validation.

## Migration and verification

Replace `B2xxReceiver::open`/`receive` and `RxPacket` with `Device::builder().open`, `rx_stream`, explicit `start`, and `read(&mut [Complex32], timeout)`. Samples are `num_complex::Complex32`. Configuration belongs to `Device`; data and queue ownership belong to `RxStream`. Replace dropping a JavaScript wrapper with explicit Rust `close` and `shutdown`.

Run `cargo test --all-targets` for the native test suite. Browser tests use `wasm-bindgen-test-runner` and ChromeDriver. Physical hardware tests are opt-in, ignored by default, and should run serially:

```console
cargo test --features hardware-tests --test hardware -- --ignored --test-threads=1 --nocapture
```

Run only tests appropriate for the attached model. Set `UHD_RS_SERIAL` when necessary. `UHD_RS_RELOAD_FIRMWARE=1` makes the B2xx diagnostic test exercise a cold firmware reload. The B200 test includes RX restart/cancellation/reopening; optionally set `UHD_RS_HANDOFF_COMMAND` to an executable that opens the released device in another application.

## Releases

The GitHub Actions release workflow publishes `uhd-rs` to crates.io and creates a GitHub release when a `v*` tag matches the crate version. Before tagging a release, configure a crates.io trusted publisher for repository `bastibl/uhd-rs` and workflow `release.yml`. CI runs native checks, browser lifecycle tests, and embedded-image/package verification on pushes and pull requests.

## License

[GPL-3.0-or-later](LICENSE), matching UHD's host driver. Embedded image notices and corresponding-source links are documented separately in [images/README.md](images/README.md).

If an alternative UHD license is obtained as described by UHD, the authors agree to the same license for this crate without additional compensation.
