#![cfg(target_arch = "wasm32")]

use uhd_rs::{Device, Error, Result, b2xx, images};

// Exercise the same handle lookup and cleanup tests inside an actual worker.
#[path = "../src/browser_usb.rs"]
mod browser_usb;

wasm_bindgen_test::wasm_bindgen_test_configure!(run_in_dedicated_worker);
