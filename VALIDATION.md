# Verification

Run from the repository root:

```console
python3 scripts/check.py
python3 scripts/check-package.py
```

The suite retains the original 25 tests. Additional tests exercise startup orchestration with a fake USB backend, product image selection, incompatible firmware/FPGA handling, malformed firmware before reset, cancellation and release, reconnection identity, Linux USB 2/3 companion-port topology, stream claims, partial reads, timeout/cancellation queue reuse, overflow recovery, restart, surviving streams, consuming close, terminal/retryable shutdown and native `Send` futures. Fake backends replace I/O; they do not claim to emulate RF or USB timing.

The image tests verify all six assets, parse the firmware, check pinned UHD hashes and test override precedence. `check-package.py` extracts a Cargo package, compiles the debug image tests, removes the runtime asset directory and runs the compiled tests. Archive and extracted-file SHA256 values are in `images/checksums.json`.

Browser tests require a wasm-bindgen test runner matching Cargo.lock, Chrome and ChromeDriver:

```console
CHROMEDRIVER=/path/to/chromedriver \
CARGO_TARGET_WASM32_UNKNOWN_UNKNOWN_RUNNER=/path/to/wasm-bindgen-test-runner \
WASM_BINDGEN_USE_BROWSER=1 \
cargo test --target wasm32-unknown-unknown --lib
```

These tests run production ownership and browser-close code against controlled I/O and mocked WebUSB objects in a real browser. They cover deferred final-owner cleanup, outstanding sample transfers, stop/restart without duplicate submission, explicit close/reopen requirements, permission loss, cancelled initialization cleanup and terminal `UsbDevice.close()` aborting a pending operation. They do not access a real browser USB device.

## Hardware scope

Opt-in tests live in `tests/hardware.rs` and are ignored unless selected with `--ignored`. They must run serially. `b2xx_startup_images_and_diagnostics` applies to all B2xx models. `b210_high_level_rejects_unsupported_radio_and_releases_usb` applies to B210. `b200_cold_receive_restart_cancel_and_reopen` requires B200 revision 5+.

B200 RX throughput, RF correctness, short-transfer behavior, hardware overflow/restart/cancellation and handoff to another application require separate physical B200 testing. A connected B210 cannot validate those claims.

## Results recorded 2026-09-11

- Native default and no-default-feature suites: **51 passed** each, including the original 25 tests.
- Native smol/tokio and wasm default/no-default configurations: all targets compile.
- Native Clippy, including all features: passes with warnings denied. Formatting and diff whitespace checks pass.
- Chrome 153 / wasm-bindgen 0.2.127 browser suite: **10 passed**, including decompression and hash verification of every embedded image in wasm.
- Debug/release embedded image tests and extracted Cargo-package tests: pass. The package contains all six assets and `LICENSE`; debug tests run successfully with runtime assets unavailable.
- Physical B210, EEPROM revision 4, serial `3228514`: cold firmware reload/reconnection, firmware ABI 8.0, product selection, embedded B210 FPGA load, hash-based reuse, FPGA ABI 16.0/two-chain discovery, local register reads, and USB release/reopen pass. High-level RX opening correctly reports that only B200 radio initialization is supported, and all interfaces can subsequently be reopened.
- The cold-start checks exposed and resolved Linux USB 2/3 companion-port matching and enumeration-before-udev-permissions races. Initial radio-register loopback timed out with the B210 radio uninitialized; radio-register/RF validation is outside the supported B200-only radio scope.
- No physical B200 RX tests or external-application handoff were run. Browser tests use mocked WebUSB devices, not the attached B210.
