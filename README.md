# uhd-pure

`uhd-pure` is a pure Rust host driver for Ettus/National Instruments USRPs. Its
USB path uses [`nusb`](https://crates.io/crates/nusb), including nusb's WebUSB
backend, instead of linking to libusb or the C++ UHD library.

The implemented B2xx foundation currently includes:

- native discovery and browser WebUSB permission requests;
- Cypress FX3 Intel HEX firmware loading;
- B200/B210/B200mini/B205mini FPGA loading and image hash checks;
- FX3 state, compatibility, USB-speed, and motherboard EEPROM queries;
- all four FPGA bulk endpoints;
- CHDR packet encoding/decoding;
- checked FPGA sessions plus local/radio Wishbone register transactions;
- the B2xx SPI core with raw AD9361 register reads and writes;
- pure-Rust B200 AD9364 cold-start initialization, calibration, and digital
  interface loopback verification; and
- continuous channel-zero B200 receive streaming, tuning, FPGA DDC rate
  selection, manual/automatic gain, CHDR validation, and normalized `f32` IQ.

Radio initialization and the receive API currently target a revision 5 or newer
B200/AD9364. B210/mini radio initialization, transmit streaming, and
multi-channel operation are not implemented yet, so this should not be
considered a drop-in replacement for the whole C++ `multi_usrp` API.

## Native diagnostic CLI

```console
cargo run -- list
cargo run -- probe
cargo run -- load-firmware /usr/share/uhd/images/usrp_b200_fw.hex
cargo run -- load-fpga /usr/share/uhd/images/usrp_b200_fpga.bin
cargo run -- init-radio
cargo run -- peek 0x50
```

Pass a serial number as the last argument when more than one B2xx is attached.
Firmware and FPGA image files remain external inputs; they are not vendored.

The native fixed-frequency receive example captures 10 seconds at 100 MHz and
1 MS/s into headerless, interleaved, little-endian `f32` IQ data:

```console
cargo run --release --example rx_100mhz -- capture.fc32
```

The example targets a revision 5 or newer B200. It cold-starts and calibrates
the AD9364, verifies its digital interface, performs the 100 MHz retune, and
sets up the FPGA receive stream automatically.

## WebUSB

Build the `cdylib` with the JS-facing wrapper enabled:

```console
wasm-pack build --target web --dev --out-dir web/pkg . -- --features wasm
cp /usr/share/uhd/images/usrp_b200_fw.hex web/pkg/
cp /usr/share/uhd/images/usrp_b200_fpga.bin web/pkg/
python3 -m http.server 8000 --directory web
```

Call `B2xxDevice.request()` directly from a click/tap handler because browsers
require transient user activation for the WebUSB chooser. The checked-in Cargo
target configuration enables the unstable `web-sys` WebUSB bindings required
by nusb. `web/index.html` is a small device/firmware/FPGA/register probe that
exercises the generated bindings at `http://localhost:8000`. By default the
page downloads `usrp_b200_fw.hex` and `usrp_b200_fpga.bin` from the same
`web/pkg` directory as `uhd_pure_bg.wasm`; the file picker can override the FPGA
image. Loading the FPGA from the page also initializes and verifies the radio.
Applications can call `initializeRadio()` explicitly, while the Rust
`B2xxReceiver::open()` and session startup paths do so automatically. Firmware
and FPGA images are external build/deployment assets and are not checked into
this repository. Click **Release B200 for another app** (or call the generated
wasm-bindgen object's `free()` method) before opening the device from another
page or a native process; the WebUSB handle otherwise keeps its interfaces
claimed.

## B2xx receive transfer sizing

UHD deliberately requests B2xx sample IN transfers of 8176 or 16360 bytes. The
length must be 8-byte aligned but *not* aligned to the USB maximum packet size,
which avoids an FX3 failure mode. nusb 0.2.7 currently rejects all IN transfer
lengths that are not a multiple of the endpoint maximum packet size, including
on WebUSB.

The receive API submits an aligned 16384-byte buffer and accepts the short
transfer produced when a B2xx frame ends. An upstream nusb API that permits the
traditional unaligned request length would avoid relying on this short-transfer
behavior. This repository does not vendor or patch nusb.

## License

GPL-3.0-or-later, matching UHD.
