//! Capture 10 seconds of 100 MHz complex baseband at 1 MS/s.
//!
//! The output is headerless interleaved little-endian `f32` IQ data:
//! `I0, Q0, I1, Q1, ...`.
//!
//! ```text
//! cargo run --release --example rx_100mhz -- capture.fc32
//! ```

use std::fs::OpenOptions;
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use uhd_pure::b2xx::{B2xxReceiver, RxConfig, RxGain};

const CENTER_FREQUENCY_HZ: f64 = 100_000_000.0;
const SAMPLE_RATE_HZ: f64 = 1_000_000.0;
const DEFAULT_DURATION_SECONDS: f64 = 10.0;

fn main() {
    if let Err(error) = futures_lite::future::block_on(run()) {
        eprintln!("error: {error}");
        std::process::exit(1);
    }
}

async fn run() -> std::result::Result<(), Box<dyn std::error::Error>> {
    let (output_path, duration_seconds) = arguments()?;
    let requested_samples = (SAMPLE_RATE_HZ * duration_seconds).round() as usize;
    if requested_samples == 0 {
        return Err("duration must request at least one sample".into());
    }

    let devices = uhd_pure::b2xx::list_devices().await?;
    let info = match devices.as_slice() {
        [] => return Err("no B200 found".into()),
        [device] => device.clone(),
        _ => return Err("multiple B2xx devices found; attach only the capture device".into()),
    };
    if !info.firmware_loaded {
        return Err("the B200 is in its FX3 bootloader; load firmware first".into());
    }

    let device = info.open().await?;
    let mut receiver = B2xxReceiver::open(
        device,
        RxConfig {
            center_frequency_hz: CENTER_FREQUENCY_HZ,
            sample_rate_hz: SAMPLE_RATE_HZ,
            gain: RxGain::Manual(30.0),
        },
    )
    .await?;
    println!(
        "Using {} serial={} name={:?}; output={}",
        receiver.product(),
        receiver.identity().serial,
        receiver.identity().name,
        output_path.display()
    );

    let file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&output_path)?;
    let mut output = BufWriter::with_capacity(1024 * 1024, file);
    let mut samples_written = 0_usize;
    while samples_written < requested_samples {
        let packet = receiver.receive().await?;
        let keep = packet
            .samples
            .len()
            .min(requested_samples - samples_written);
        for sample in &packet.samples[..keep] {
            output.write_all(&sample.re.to_le_bytes())?;
            output.write_all(&sample.im.to_le_bytes())?;
        }
        samples_written += keep;
        if samples_written % 1_000_000 < keep {
            println!("  {samples_written}/{requested_samples} samples");
        }
    }
    receiver.stop().await?;
    output.flush()?;
    println!(
        "Wrote {samples_written} interleaved little-endian fc32 samples to {}",
        output_path.display()
    );
    Ok(())
}

fn arguments() -> std::result::Result<(PathBuf, f64), Box<dyn std::error::Error>> {
    let mut args = std::env::args_os().skip(1);
    let output = args
        .next()
        .map_or_else(|| PathBuf::from("rx_100mhz_1msps.fc32"), PathBuf::from);
    let duration = match args.next() {
        Some(value) => value
            .to_str()
            .ok_or("duration is not valid UTF-8")?
            .parse::<f64>()?,
        None => DEFAULT_DURATION_SECONDS,
    };
    if args.next().is_some() || !duration.is_finite() || duration <= 0.0 {
        return Err("usage: rx_100mhz [OUTPUT.fc32] [SECONDS]".into());
    }
    Ok((output, duration))
}
