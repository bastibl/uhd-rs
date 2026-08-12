//! Capture 10 seconds of 100 MHz complex baseband at 1 MS/s.
//!
//! The output is headerless interleaved little-endian `f32` IQ data:
//! `I0, Q0, I1, Q1, ...`.
//!
//! ```text
//! cargo run --release --example rx_100mhz -- capture.fc32
//! ```
//!
//! A second positional argument overrides the duration for diagnostics. The
//! normal/default duration is 10 seconds.

use std::fs::OpenOptions;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::time::Duration;

use futures_timer::Delay;
use uhd_pure::b2xx::{Ad9361Io, Fx3State, Product, RadioControl, StreamId};
use uhd_pure::{Error, Result, chdr};

const CENTER_FREQUENCY_HZ: f64 = 100_000_000.0;
const SAMPLE_RATE_SPS: f64 = 1_000_000.0;
const MASTER_CLOCK_HZ: f64 = 16_000_000.0;
const DEFAULT_DURATION_SECONDS: f64 = 10.0;
const SAMPLES_PER_PACKET: u32 = 1_000;
const DATA_TRANSFER_BYTES: usize = 16_384;

// Local settings registers, in byte-address form.
const SR_CORE_MISC: u32 = 16 * 4;

// Radio settings registers, in byte-address form.
const SR_ATR: u32 = 12 * 4;
const SR_RX_CTRL: u32 = 96 * 4;
const SR_RX_FMT: u32 = 136 * 4;
const SR_RX_DSP: u32 = 144 * 4;

const ATR_IDLE: u32 = SR_ATR;
const ATR_RX_ONLY: u32 = SR_ATR + 4;
const ATR_TX_ONLY: u32 = SR_ATR + 8;
const ATR_FULL_DUPLEX: u32 = SR_ATR + 12;
const ATR_DISABLE: u32 = SR_ATR + 20;

const RX_CTRL_COMMAND: u32 = SR_RX_CTRL;
const RX_CTRL_TIME_HIGH: u32 = SR_RX_CTRL + 4;
const RX_CTRL_TIME_LOW: u32 = SR_RX_CTRL + 8;
const RX_FRAMER_MAX_SAMPLES: u32 = SR_RX_CTRL + 16;
const RX_FRAMER_STREAM_ID: u32 = SR_RX_CTRL + 20;

const RX_DSP_FREQUENCY: u32 = SR_RX_DSP;
const RX_DSP_SCALE_IQ: u32 = SR_RX_DSP + 4;
const RX_DSP_DECIMATION: u32 = SR_RX_DSP + 8;
const RX_DSP_MUX: u32 = SR_RX_DSP + 12;

const RX_DATA_STREAM_ID: u32 = 0x0000_00a0;
const MAX_FINITE_SAMPLES: u32 = 0x0fff_ffff;

// B200 rev 5, frontend 1, RX2. These values come from the B200 ATR bit map.
const CORE_MISC_LOW_RX_BAND: u32 = (1 << 6) | (1 << 3);
const ATR_RX_OFF: u32 = (1 << 6) | (1 << 3);
const ATR_RX2_ACTIVE: u32 = ATR_RX_OFF | (1 << 2);

fn main() {
    if let Err(error) = futures_lite::future::block_on(run()) {
        eprintln!("error: {error}");
        std::process::exit(1);
    }
}

async fn run() -> std::result::Result<(), Box<dyn std::error::Error>> {
    let (output_path, duration_seconds) = arguments()?;
    let sample_count_f64 = SAMPLE_RATE_SPS * duration_seconds;
    if !(1.0..=f64::from(MAX_FINITE_SAMPLES)).contains(&sample_count_f64) {
        return Err(
            format!("duration must request between 1 and {MAX_FINITE_SAMPLES} samples").into(),
        );
    }
    let requested_samples = sample_count_f64.round() as u32;

    let devices = uhd_pure::b2xx::list_devices().await?;
    let info = match devices.as_slice() {
        [] => return Err("no B2xx device found".into()),
        [device] => device.clone(),
        _ => return Err("multiple B2xx devices found; attach only the capture device".into()),
    };
    if !info.firmware_loaded {
        return Err("the B2xx is in its FX3 bootloader; load firmware first".into());
    }

    let device = info.open().await?;
    device.check_firmware_compatibility().await?;
    let identity = device.identity().await?;
    if identity.product != Some(Product::B200) || identity.revision < 5 {
        return Err(format!(
            "this example's RF routing is for a revision 5+ B200, found {:?} revision {}",
            identity.product, identity.revision
        )
        .into());
    }
    if device.fx3_state().await? != Fx3State::Running {
        return Err("the FPGA is not running; load the B200 FPGA image first".into());
    }

    println!(
        "Using B200 serial={} name={:?}; output={}",
        identity.serial,
        identity.name,
        output_path.display()
    );

    let transport = device.open_transport().await?;
    let mut control = transport.into_radio_control(StreamId::LocalControl);

    // Select the board's sub-2.2 GHz receive filter. Keep the TX low-band
    // selection at its harmless initialization value as this register is
    // write-only.
    control.poke32(SR_CORE_MISC, CORE_MISC_LOW_RX_BAND).await?;
    tune_initialized_ad9361_to_100mhz(&mut control).await?;

    let file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&output_path)?;
    let mut output = BufWriter::with_capacity(1024 * 1024, file);
    let host_scale = configure_rx_stream(&mut control, requested_samples).await?;

    println!(
        "Capturing {requested_samples} samples at {:.3} MS/s, centered at {:.3} MHz...",
        SAMPLE_RATE_SPS / 1e6,
        CENTER_FREQUENCY_HZ / 1e6
    );

    let mut samples_written = 0_u32;
    let mut expected_sequence = None;
    while samples_written < requested_samples {
        let transfer = control
            .transport_mut()
            .receive_data(DATA_TRANSFER_BYTES)
            .await?;
        let packet = chdr::parse(&transfer)?;
        if packet.context {
            continue;
        }
        if packet.stream_id != RX_DATA_STREAM_ID {
            return Err(format!("unexpected data stream ID 0x{:08x}", packet.stream_id).into());
        }
        if packet.payload.len() % 8 != 0 {
            return Err(format!(
                "FC32 payload length {} is not a whole number of complex samples",
                packet.payload.len()
            )
            .into());
        }
        if let Some(expected) = expected_sequence
            && packet.sequence != expected
        {
            return Err(format!(
                "receive overflow: expected CHDR sequence {expected}, got {}",
                packet.sequence
            )
            .into());
        }
        expected_sequence = Some(packet.sequence.wrapping_add(1) & 0x0fff);

        let packet_samples = u32::try_from(packet.payload.len() / 8)?;
        let keep_samples = packet_samples.min(requested_samples - samples_written);
        let keep_bytes = usize::try_from(keep_samples)? * 8;
        write_normalized_fc32(&mut output, &packet.payload[..keep_bytes], host_scale)?;
        samples_written += keep_samples;

        if samples_written % 1_000_000 < keep_samples {
            println!("  {samples_written}/{requested_samples} samples");
        }
    }
    output.flush()?;

    let byte_count = u64::from(samples_written) * 8;
    println!(
        "Wrote {samples_written} interleaved little-endian FC32 samples ({byte_count} bytes) to {}",
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

/// Tune an AD9361/AD9364 which has already received its normal initialization
/// tables and calibrations. This is deliberately narrow: it applies the UHD
/// 80 MHz-reference synthesizer settings needed by the fixed 100 MHz example.
async fn tune_initialized_ad9361_to_100mhz(control: &mut RadioControl) -> Result<()> {
    control.set_stream(StreamId::LocalControl);
    let mut codec = Ad9361Io::new(uhd_pure::b2xx::B2xxSpi::new(control));
    let initial_state = codec.read_register(0x017).await? & 0x0f;
    let return_to_fdd = match initial_state {
        0x05 => false,
        0x0a => true,
        state => {
            return Err(Error::InvalidArgument(format!(
                "AD9361 is in state 0x{state:x}; cold-start initialization is required"
            )));
        }
    };
    if return_to_fdd {
        codec.write_register(0x014, 0x01).await?;
        wait_for_ad9361_state(&mut codec, 0x05).await?;
    }

    let input_selection = codec.read_register(0x004).await?;
    codec
        .write_register(0x004, (input_selection & 0xc0) | 0x30)
        .await?;

    // Enable RX chain 1, preserving the initialized filter configuration.
    let rx_filter = codec.read_register(0x003).await?;
    codec.write_register(0x003, rx_filter | 0x40).await?;

    // 100 MHz * 64 = 6.4 GHz RF VCO. This is row 44 of the AD9361 80 MHz
    // reference synthesizer calibration table used by UHD 4.8.
    let synth = [10_u8, 3, 7, 3, 15, 12, 18, 13, 4, 13, 15, 9];
    codec.write_register(0x23a, 0x40 | synth[0]).await?;
    codec.write_register(0x239, 0xc0 | synth[1]).await?;
    codec
        .write_register(0x242, synth[2] | (synth[3] << 3))
        .await?;
    codec.write_register(0x238, synth[4] << 3).await?;
    codec.write_register(0x245, 0x00).await?;
    codec.write_register(0x251, synth[5]).await?;
    codec.write_register(0x250, 0x70).await?;
    codec.write_register(0x23b, 0x80 | synth[6]).await?;
    codec
        .write_register(0x23e, synth[8] | (synth[7] << 4))
        .await?;
    codec
        .write_register(0x23f, synth[10] | (synth[9] << 4))
        .await?;
    codec.write_register(0x240, synth[11]).await?;

    // With a 6.4 GHz VCO and 80 MHz reference, Nint=80 and Nfrac=0.
    codec.write_register(0x233, 0).await?;
    codec.write_register(0x234, 0).await?;
    codec.write_register(0x235, 0).await?;
    codec.write_register(0x232, 0).await?;
    codec.write_register(0x231, 80).await?;
    let dividers = codec.read_register(0x005).await?;
    codec.write_register(0x005, (dividers & 0xf0) | 5).await?;

    Delay::new(Duration::from_millis(2)).await;
    if codec.read_register(0x247).await? & 0x02 == 0 {
        return Err(Error::InvalidArgument(
            "AD9361 receive synthesizer did not lock at 100 MHz".into(),
        ));
    }

    // A moderate manual gain keeps the example deterministic.
    codec.write_register(0x109, 30).await?;
    if return_to_fdd {
        codec.write_register(0x014, 0x21).await?;
        wait_for_ad9361_state(&mut codec, 0x0a).await?;
    }
    Ok(())
}

async fn wait_for_ad9361_state(codec: &mut Ad9361Io<'_>, expected: u8) -> Result<()> {
    for _ in 0..100 {
        if codec.read_register(0x017).await? & 0x0f == expected {
            return Ok(());
        }
        Delay::new(Duration::from_millis(1)).await;
    }
    Err(Error::InvalidArgument(format!(
        "AD9361 did not enter state 0x{expected:x}"
    )))
}

async fn configure_rx_stream(control: &mut RadioControl, samples: u32) -> Result<f32> {
    control.set_stream(StreamId::RadioControl(0));

    // Use RX2 and let the FPGA ATR state machine control the RF switches.
    control.poke32(ATR_DISABLE, 0).await?;
    control.poke32(ATR_IDLE, ATR_RX_OFF).await?;
    control.poke32(ATR_RX_ONLY, ATR_RX2_ACTIVE).await?;
    control.poke32(ATR_TX_ONLY, ATR_RX_OFF).await?;
    control.poke32(ATR_FULL_DUPLEX, ATR_RX2_ACTIVE).await?;

    // Request native float32 IQ on the wire. The file can therefore be
    // written without a host-side integer-to-float conversion.
    control.poke32(SR_RX_FMT, 2).await?;
    control.poke32(RX_DSP_MUX, 0).await?;
    control.poke32(RX_DSP_FREQUENCY, 0).await?;

    let decimation = (MASTER_CLOCK_HZ / SAMPLE_RATE_SPS).round() as u32;
    if decimation != 16 {
        return Err(Error::InvalidArgument(
            "the fixed example expects 16 MHz / 16 = 1 MS/s".into(),
        ));
    }
    // Two half-band filters leave a CIC decimation of four.
    control
        .poke32(RX_DSP_DECIMATION, (1 << 9) | (1 << 8) | 4)
        .await?;
    let cic_gain = 4_f64.powi(4);
    let scaling_adjustment = 2_f64.powi(cic_gain.log2().ceil() as i32) / (1.648 * cic_gain);
    let target_scalar = 65_536.0 * scaling_adjustment;
    let scalar = target_scalar.round() as u32;
    control.poke32(RX_DSP_SCALE_IQ, scalar).await?;

    control
        .poke32(RX_FRAMER_MAX_SAMPLES, SAMPLES_PER_PACKET)
        .await?;
    control
        .poke32(RX_FRAMER_STREAM_ID, RX_DATA_STREAM_ID)
        .await?;

    // Finite, immediate capture. Writing TIME_LOW latches the command.
    control.poke32(RX_CTRL_COMMAND, (1 << 31) | samples).await?;
    control.poke32(RX_CTRL_TIME_HIGH, 0).await?;
    control.poke32(RX_CTRL_TIME_LOW, 0).await?;
    Ok((target_scalar / f64::from(scalar) / 32_767.0) as f32)
}

fn write_normalized_fc32(
    output: &mut impl Write,
    wire_bytes: &[u8],
    scale: f32,
) -> std::io::Result<()> {
    let mut normalized = Vec::with_capacity(wire_bytes.len());
    for word in wire_bytes.chunks_exact(4) {
        let value = f32::from_bits(u32::from_le_bytes(
            word.try_into().expect("four-byte float word"),
        ));
        normalized.extend_from_slice(&(value * scale).to_le_bytes());
    }
    output.write_all(&normalized)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalizes_little_endian_wire_floats() {
        let wire = [2.0_f32.to_le_bytes(), (-4.0_f32).to_le_bytes()].concat();
        let mut output = Vec::new();
        write_normalized_fc32(&mut output, &wire, 0.25).unwrap();
        assert_eq!(
            output,
            [0.5_f32.to_le_bytes(), (-1.0_f32).to_le_bytes()].concat()
        );
    }
}
