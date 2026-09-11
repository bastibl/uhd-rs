//! Shared Rust async API; call receive() from your native or browser executor.
use std::time::Duration;
use uhd_rs::{Complex32, Device};
async fn receive() -> uhd_rs::Result<()> {
    // On wasm, first call Device::request_permission().await from a user gesture.
    let mut device = Device::builder().open().await?;
    let mut rx = device.rx_stream()?;
    let result: uhd_rs::Result<()> = async {
        rx.start().await?;
        let mut samples = [Complex32::default(); 4096];
        let count = rx.read(&mut samples, Some(Duration::from_secs(2))).await?;
        println!("Received {count} samples");
        rx.stop().await?;
        rx.start().await?;
        rx.read(&mut samples, Some(Duration::from_secs(2))).await?;
        Ok(())
    }
    .await;
    let close = rx.close().await;
    let shutdown = device.shutdown().await;
    result?;
    close?;
    shutdown
}
#[cfg(not(target_arch = "wasm32"))]
fn main() -> uhd_rs::Result<()> {
    futures_lite::future::block_on(receive())
}
#[cfg(target_arch = "wasm32")]
fn main() {
    wasm_bindgen_futures::spawn_local(async {
        let _ = receive().await;
    });
}
