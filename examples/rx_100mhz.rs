//! Write ten seconds of interleaved little-endian f32 IQ to a new file.
#[cfg(not(target_arch = "wasm32"))]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use std::{
        fs::OpenOptions,
        io::{BufWriter, Write},
        time::Duration,
    };
    use uhd_pure::{Complex32, Device, MaybeFuture};
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "capture.fc32".into());
    let mut output = BufWriter::new(OpenOptions::new().write(true).create_new(true).open(path)?);
    let mut device = Device::builder().open().wait()?;
    let mut rx = device.rx_stream()?;
    let capture = (|| -> uhd_pure::Result<()> {
        rx.start().wait()?;
        let mut samples = [Complex32::default(); 4096];
        let mut remaining = 10_000_000;
        while remaining > 0 {
            let size = samples.len().min(remaining);
            let count = rx
                .read(&mut samples[..size], Some(Duration::from_secs(2)))
                .wait()?;
            for sample in &samples[..count] {
                output.write_all(&sample.re.to_le_bytes())?;
                output.write_all(&sample.im.to_le_bytes())?;
            }
            remaining -= count;
        }
        output.flush()?;
        Ok(())
    })();
    let close = rx.close().wait();
    let shutdown = device.shutdown().wait();
    capture?;
    println!("{:?}", close?);
    shutdown?;
    Ok(())
}
#[cfg(target_arch = "wasm32")]
fn main() {}
