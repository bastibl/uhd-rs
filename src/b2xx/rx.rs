//! Receive streaming for a B200.
//!
//! The current implementation configures channel zero, the `RX2` antenna,
//! native `fc32` samples on the USB wire, the FPGA DDC, and cold-starts the
//! AD9364 before applying the requested stream configuration.

use super::{B2xxDevice, B2xxIdentity, Product, RadioControl, StreamId, ad9361::Ad9361Controller};
use crate::{Error, Result, chdr};

const MASTER_CLOCK_HZ: f64 = 16_000_000.0;
const MIN_RF_HZ: f64 = 70_000_000.0;
const MAX_RF_HZ: f64 = 6_000_000_000.0;
const MIN_SAMPLE_RATE_HZ: f64 = MASTER_CLOCK_HZ / 512.0;
const MAX_SAMPLE_RATE_HZ: f64 = MASTER_CLOCK_HZ;
const MIN_GAIN_DB: f64 = 0.0;
const MAX_GAIN_DB: f64 = 76.0;
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
const RX_CONTEXT_OVERFLOW: u8 = 0x08;

// ATR state for frontend 1 (RF B on B200/B210), using RX2.
const SFDX1_RX: u32 = 1 << 6;
const SRX1_TX: u32 = 1 << 3;
const LED_RX1: u32 = 1 << 2;
const STATE_RX1_OFF: u32 = SFDX1_RX | SRX1_TX;
const STATE_RX1_RX2: u32 = STATE_RX1_OFF | LED_RX1;

/// Input gain selection for a B2xx receive channel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RxGain {
    /// Let the AD9361 slow-attack AGC select receive gain.
    Automatic,
    /// Select a manual gain in dB. The supported range is 0 through 76 dB.
    Manual(f64),
}

/// Configuration used when opening a channel-zero B2xx receiver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RxConfig {
    /// RF center frequency in Hz.
    pub center_frequency_hz: f64,
    /// Complex samples per second requested from the FPGA DDC.
    pub sample_rate_hz: f64,
    /// Receive gain mode.
    pub gain: RxGain,
}

/// Requested logical receive center and RF-LO offset.
///
/// The RF synthesizer is tuned to `center_frequency_hz + lo_offset_hz` and
/// the FPGA DDC translates the result back to the requested logical center.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RxTuneRequest {
    /// Logical center frequency presented by the receive stream, in Hz.
    pub center_frequency_hz: f64,
    /// Signed offset from the logical center to the physical RF LO, in Hz.
    pub lo_offset_hz: f64,
}

impl RxTuneRequest {
    /// Construct a zero-IF tune request.
    #[must_use]
    pub const fn new(center_frequency_hz: f64) -> Self {
        Self {
            center_frequency_hz,
            lo_offset_hz: 0.0,
        }
    }

    /// Construct a low-IF tune request with an explicit RF-LO offset.
    #[must_use]
    pub const fn with_lo_offset(center_frequency_hz: f64, lo_offset_hz: f64) -> Self {
        Self {
            center_frequency_hz,
            lo_offset_hz,
        }
    }

    /// Validate the logical center, physical LO, and CORDIC range.
    pub fn validate(self) -> Result<Self> {
        validate_range(
            "center frequency",
            self.center_frequency_hz,
            MIN_RF_HZ,
            MAX_RF_HZ,
        )?;
        if !self.lo_offset_hz.is_finite() {
            return Err(Error::InvalidArgument("LO offset must be finite".into()));
        }
        let rf_frequency_hz = self.center_frequency_hz + self.lo_offset_hz;
        validate_range("RF LO frequency", rf_frequency_hz, MIN_RF_HZ, MAX_RF_HZ)?;
        if self.lo_offset_hz.abs() >= MASTER_CLOCK_HZ / 2.0 {
            return Err(Error::InvalidArgument(format!(
                "absolute LO offset must be below {}",
                MASTER_CLOCK_HZ / 2.0
            )));
        }
        Ok(self)
    }
}

/// Actual RF and DSP frequencies selected for one receive tune.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RxTuneResult {
    /// Logical center requested by the caller, in Hz.
    pub requested_center_frequency_hz: f64,
    /// Physical RF-LO frequency requested from the AD9361, in Hz.
    pub target_rf_frequency_hz: f64,
    /// Physical RF-LO frequency reached by the AD9361, in Hz.
    pub actual_rf_frequency_hz: f64,
    /// Frequency requested from the FPGA RX CORDIC, in Hz.
    pub target_dsp_frequency_hz: f64,
    /// Quantized FPGA RX CORDIC frequency, in Hz.
    pub actual_dsp_frequency_hz: f64,
    /// Resulting logical receive-stream center, in Hz.
    pub actual_center_frequency_hz: f64,
}

impl RxConfig {
    /// Validate and normalize this configuration without accessing hardware.
    pub fn validate(self) -> Result<Self> {
        validate_range(
            "center frequency",
            self.center_frequency_hz,
            MIN_RF_HZ,
            MAX_RF_HZ,
        )?;
        validate_range(
            "sample rate",
            self.sample_rate_hz,
            MIN_SAMPLE_RATE_HZ,
            MAX_SAMPLE_RATE_HZ,
        )?;
        if let RxGain::Manual(gain) = self.gain {
            validate_range("gain", gain, MIN_GAIN_DB, MAX_GAIN_DB)?;
        }
        Ok(self)
    }
}

/// One normalized complex `f32` receive sample.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Complex32 {
    pub re: f32,
    pub im: f32,
}

impl Complex32 {
    #[must_use]
    pub const fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }
}

/// Metadata and samples returned by one B2xx receive packet.
#[derive(Clone, Debug, PartialEq)]
pub struct RxPacket {
    pub timestamp: Option<u64>,
    pub samples: Vec<Complex32>,
}

/// Continuous channel-zero receive stream from a B200.
pub struct B2xxReceiver {
    control: RadioControl,
    radio: Ad9361Controller,
    identity: B2xxIdentity,
    product: Product,
    center_frequency_hz: f64,
    rf_frequency_hz: f64,
    dsp_frequency_hz: f64,
    sample_rate_hz: f64,
    gain: RxGain,
    host_scale: f32,
    sequence_state: RxSequenceState,
    streaming: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RxSequenceState {
    /// Ignore packets queued before writing the stream ID reset the FPGA's
    /// sequence counter. The first packet from the new stream is sequence zero.
    Synchronizing,
    Tracking(u16),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SequenceDisposition {
    Discard,
    Accept,
    Overflow { expected: u16, actual: u16 },
}

impl RxSequenceState {
    fn observe(&mut self, actual: u16) -> SequenceDisposition {
        match *self {
            Self::Synchronizing if actual != 0 => SequenceDisposition::Discard,
            Self::Synchronizing => {
                *self = Self::Tracking(1);
                SequenceDisposition::Accept
            }
            Self::Tracking(expected) => {
                *self = Self::Tracking(actual.wrapping_add(1) & 0x0fff);
                if actual == expected {
                    SequenceDisposition::Accept
                } else {
                    SequenceDisposition::Overflow { expected, actual }
                }
            }
        }
    }
}

impl std::fmt::Debug for B2xxReceiver {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("B2xxReceiver")
            .field("identity", &self.identity)
            .field("product", &self.product)
            .field("center_frequency_hz", &self.center_frequency_hz)
            .field("rf_frequency_hz", &self.rf_frequency_hz)
            .field("dsp_frequency_hz", &self.dsp_frequency_hz)
            .field("sample_rate_hz", &self.sample_rate_hz)
            .field("gain", &self.gain)
            .field("streaming", &self.streaming)
            .finish_non_exhaustive()
    }
}

impl B2xxReceiver {
    /// Open and configure a B200, including AD9364 cold-start initialization.
    pub async fn open(device: B2xxDevice, config: RxConfig) -> Result<Self> {
        let config = config.validate()?;
        let session = device.open_session().await?;
        let (control, identity, product, radio) = session.into_radio_parts()?;
        let mut receiver = Self {
            control,
            radio,
            identity,
            product,
            center_frequency_hz: config.center_frequency_hz,
            rf_frequency_hz: config.center_frequency_hz,
            dsp_frequency_hz: 0.0,
            sample_rate_hz: config.sample_rate_hz,
            gain: config.gain,
            host_scale: 1.0,
            sequence_state: RxSequenceState::Synchronizing,
            streaming: false,
        };
        receiver.configure_frontend().await?;
        receiver.set_sample_rate(config.sample_rate_hz).await?;
        receiver
            .set_center_frequency(config.center_frequency_hz)
            .await?;
        receiver.set_gain(config.gain).await?;
        receiver.start().await?;
        Ok(receiver)
    }

    #[must_use]
    pub const fn identity(&self) -> &B2xxIdentity {
        &self.identity
    }

    #[must_use]
    pub const fn product(&self) -> Product {
        self.product
    }

    #[must_use]
    pub const fn center_frequency_hz(&self) -> f64 {
        self.center_frequency_hz
    }

    /// Return the physical AD9361 receive-LO frequency.
    #[must_use]
    pub const fn rf_frequency_hz(&self) -> f64 {
        self.rf_frequency_hz
    }

    /// Return the FPGA RX CORDIC frequency.
    #[must_use]
    pub const fn dsp_frequency_hz(&self) -> f64 {
        self.dsp_frequency_hz
    }

    #[must_use]
    pub const fn sample_rate_hz(&self) -> f64 {
        self.sample_rate_hz
    }

    #[must_use]
    pub const fn gain(&self) -> RxGain {
        self.gain
    }

    /// Retune to a logical zero-IF center frequency.
    pub async fn set_center_frequency(&mut self, frequency_hz: f64) -> Result<f64> {
        Ok(self
            .tune(RxTuneRequest::new(frequency_hz))
            .await?
            .actual_center_frequency_hz)
    }

    /// Retune the AD9361 receive synthesizer and FPGA DDC as one logical tune.
    pub async fn tune(&mut self, request: RxTuneRequest) -> Result<RxTuneResult> {
        let request = request.validate()?;
        let target_rf_frequency_hz = request.center_frequency_hz + request.lo_offset_hz;
        self.control.set_stream(StreamId::LocalControl);
        let misc = receiver_misc_word(target_rf_frequency_hz);
        self.control.poke32(SR_CORE_MISC, misc).await?;
        let actual_rf_frequency_hz = self
            .radio
            .tune_rx(&mut self.control, target_rf_frequency_hz)
            .await?;
        let target_dsp_frequency_hz = actual_rf_frequency_hz - request.center_frequency_hz;
        let (actual_dsp_frequency_hz, frequency_word) =
            ddc_frequency_word(target_dsp_frequency_hz)?;
        self.control.set_stream(StreamId::RadioControl(0));
        self.control
            .poke32(RX_DSP_FREQUENCY, frequency_word)
            .await?;
        let actual_center_frequency_hz = actual_rf_frequency_hz - actual_dsp_frequency_hz;

        self.center_frequency_hz = actual_center_frequency_hz;
        self.rf_frequency_hz = actual_rf_frequency_hz;
        self.dsp_frequency_hz = actual_dsp_frequency_hz;
        Ok(RxTuneResult {
            requested_center_frequency_hz: request.center_frequency_hz,
            target_rf_frequency_hz,
            actual_rf_frequency_hz,
            target_dsp_frequency_hz,
            actual_dsp_frequency_hz,
            actual_center_frequency_hz,
        })
    }

    /// Set the FPGA DDC rate. The nearest supported integer decimation is used.
    pub async fn set_sample_rate(&mut self, sample_rate_hz: f64) -> Result<f64> {
        validate_range(
            "sample rate",
            sample_rate_hz,
            MIN_SAMPLE_RATE_HZ,
            MAX_SAMPLE_RATE_HZ,
        )?;
        let (actual_rate, decimation_word, host_scale) = ddc_settings(sample_rate_hz)?;
        self.control.set_stream(StreamId::RadioControl(0));
        self.control.poke32(RX_DSP_MUX, 0).await?;
        let (_, frequency_word) = ddc_frequency_word(self.dsp_frequency_hz)?;
        self.control
            .poke32(RX_DSP_FREQUENCY, frequency_word)
            .await?;
        self.control
            .poke32(RX_DSP_DECIMATION, decimation_word)
            .await?;
        self.control.poke32(RX_DSP_SCALE_IQ, host_scale.1).await?;
        self.host_scale = host_scale.0;
        self.sample_rate_hz = actual_rate;
        Ok(actual_rate)
    }

    /// Set manual receive gain or slow-attack automatic gain control.
    pub async fn set_gain(&mut self, gain: RxGain) -> Result<()> {
        if let RxGain::Manual(value) = gain {
            validate_range("gain", value, MIN_GAIN_DB, MAX_GAIN_DB)?;
        }
        match gain {
            RxGain::Automatic => self.radio.set_slow_agc_on(&mut self.control).await?,
            RxGain::Manual(value) => {
                self.radio
                    .set_manual_rx_gain_on(&mut self.control, value.round())
                    .await?;
            }
        }
        self.gain = gain;
        Ok(())
    }

    /// Start or restart continuous immediate reception.
    pub async fn start(&mut self) -> Result<()> {
        self.control.set_stream(StreamId::RadioControl(0));
        self.control.poke32(SR_RX_FMT, 2).await?;
        self.control
            .poke32(RX_FRAMER_MAX_SAMPLES, SAMPLES_PER_PACKET)
            .await?;
        self.control
            .poke32(RX_FRAMER_STREAM_ID, RX_DATA_STREAM_ID)
            .await?;
        issue_stream_command(&mut self.control, StreamCommand::StartContinuous).await?;
        // Setting the stream ID resets the FPGA sequence counter. WebUSB may
        // still have packets from an earlier stream queued, so receive() waits
        // for sequence zero before exposing samples or checking continuity.
        self.sequence_state = RxSequenceState::Synchronizing;
        self.streaming = true;
        Ok(())
    }

    /// Stop continuous reception. A later call to [`Self::start`] resumes it.
    pub async fn stop(&mut self) -> Result<()> {
        if self.streaming {
            self.control.set_stream(StreamId::RadioControl(0));
            issue_stream_command(&mut self.control, StreamCommand::StopContinuous).await?;
            self.streaming = false;
        }
        Ok(())
    }

    /// Receive and decode the next non-context CHDR packet.
    pub async fn receive(&mut self) -> Result<RxPacket> {
        if !self.streaming {
            return Err(Error::InvalidArgument(
                "receive called while the B2xx stream is stopped".into(),
            ));
        }
        loop {
            let transfer = self
                .control
                .transport_mut()
                .receive_data(DATA_TRANSFER_BYTES)
                .await?;
            let packet = chdr::parse(&transfer)?;
            if packet.stream_id != RX_DATA_STREAM_ID {
                return Err(Error::Chdr(format!(
                    "unexpected receive stream ID 0x{:08x}",
                    packet.stream_id
                )));
            }
            if packet.context {
                let code = receive_context_code(packet.payload)?;
                if code == RX_CONTEXT_OVERFLOW {
                    // Reset the framer sequence as part of restarting. Any
                    // packets already queued before the overflow are then
                    // discarded by the normal startup synchronization path.
                    self.start().await?;
                    return Err(Error::DeviceReceiveOverflow {
                        sequence: packet.sequence,
                    });
                }
                return Err(Error::ReceiveContext {
                    code,
                    sequence: packet.sequence,
                });
            }
            match self.sequence_state.observe(packet.sequence) {
                SequenceDisposition::Discard => continue,
                SequenceDisposition::Accept => {}
                SequenceDisposition::Overflow { expected, actual } => {
                    return Err(Error::ReceiveOverflow { expected, actual });
                }
            }
            return Ok(RxPacket {
                timestamp: packet.timestamp,
                samples: decode_fc32(packet.payload, self.host_scale)?,
            });
        }
    }

    async fn configure_frontend(&mut self) -> Result<()> {
        self.control.set_stream(StreamId::RadioControl(0));
        self.control.poke32(ATR_DISABLE, 0).await?;
        self.control.poke32(ATR_IDLE, STATE_RX1_OFF).await?;
        self.control.poke32(ATR_RX_ONLY, STATE_RX1_RX2).await?;
        self.control.poke32(ATR_TX_ONLY, STATE_RX1_OFF).await?;
        self.control.poke32(ATR_FULL_DUPLEX, STATE_RX1_RX2).await?;
        Ok(())
    }
}

fn receive_context_code(payload: &[u8]) -> Result<u8> {
    let word = payload
        .get(..4)
        .ok_or_else(|| Error::Chdr("receive context packet has no context word".into()))?;
    let word = u32::from_le_bytes(word.try_into().expect("four-byte context word"));
    Ok(((word | word.swap_bytes()) & 0xff) as u8)
}

fn validate_range(name: &str, value: f64, minimum: f64, maximum: f64) -> Result<()> {
    if !value.is_finite() || !(minimum..=maximum).contains(&value) {
        return Err(Error::InvalidArgument(format!(
            "{name} must be finite and between {minimum} and {maximum}"
        )));
    }
    Ok(())
}

fn receiver_misc_word(frequency_hz: f64) -> u32 {
    let band = if frequency_hz < 2_200_000_000.0 {
        1 << 3
    } else if frequency_hz < 4_000_000_000.0 {
        1 << 4
    } else {
        1 << 5
    };
    band | (1 << 6)
}

/// Quantize an RX CORDIC frequency to the signed 32-bit FPGA phase word.
fn ddc_frequency_word(requested_hz: f64) -> Result<(f64, u32)> {
    if !requested_hz.is_finite() || requested_hz.abs() >= MASTER_CLOCK_HZ / 2.0 {
        return Err(Error::InvalidArgument(format!(
            "RX DSP frequency must be finite and have magnitude below {}",
            MASTER_CLOCK_HZ / 2.0
        )));
    }
    const SCALE: f64 = 4_294_967_296.0;
    let scaled = (requested_hz / MASTER_CLOCK_HZ * SCALE).round();
    if scaled < f64::from(i32::MIN) || scaled > f64::from(i32::MAX) {
        return Err(Error::InvalidArgument(
            "RX DSP frequency cannot be represented by the CORDIC".into(),
        ));
    }
    let signed_word = scaled as i32;
    let actual_hz = f64::from(signed_word) / SCALE * MASTER_CLOCK_HZ;
    Ok((actual_hz, signed_word as u32))
}

fn ddc_settings(requested_rate: f64) -> Result<(f64, u32, (f32, u32))> {
    let decimation = (MASTER_CLOCK_HZ / requested_rate).round() as u32;
    if !(1..=512).contains(&decimation) {
        return Err(Error::InvalidArgument(
            "sample rate cannot be represented by the B2xx DDC".into(),
        ));
    }
    let mut cic_decimation = decimation;
    let mut halfband_small = 0;
    let mut halfband_large = 0;
    if cic_decimation % 2 == 0 {
        halfband_small = 1;
        cic_decimation /= 2;
    }
    if cic_decimation % 2 == 0 {
        halfband_large = 1;
        cic_decimation /= 2;
    }
    if cic_decimation > 255 {
        return Err(Error::InvalidArgument(
            "sample rate requires an unsupported CIC decimation".into(),
        ));
    }
    let decimation_word = (halfband_small << 9) | (halfband_large << 8) | cic_decimation;
    let cic_gain = f64::from(cic_decimation).powi(4);
    let scaling_adjustment = 2_f64.powf(cic_gain.log2().ceil()) / (1.648 * cic_gain);
    let target_scalar = 65_536.0 * scaling_adjustment;
    let scalar = target_scalar.round() as u32;
    let host_scale = (target_scalar / f64::from(scalar) / 32_767.0) as f32;
    Ok((
        MASTER_CLOCK_HZ / f64::from(decimation),
        decimation_word,
        (host_scale, scalar),
    ))
}

enum StreamCommand {
    StartContinuous,
    StopContinuous,
}

async fn issue_stream_command(control: &mut RadioControl, command: StreamCommand) -> Result<()> {
    let command_word = match command {
        StreamCommand::StartContinuous => (1 << 31) | (1 << 30) | (1 << 29) | 1,
        StreamCommand::StopContinuous => (1 << 31) | (1 << 28),
    };
    control.poke32(RX_CTRL_COMMAND, command_word).await?;
    control.poke32(RX_CTRL_TIME_HIGH, 0).await?;
    control.poke32(RX_CTRL_TIME_LOW, 0).await
}

fn decode_fc32(bytes: &[u8], scale: f32) -> Result<Vec<Complex32>> {
    if bytes.len() % 8 != 0 {
        return Err(Error::Chdr(format!(
            "fc32 receive payload has {} bytes, not a whole number of complex samples",
            bytes.len()
        )));
    }
    let mut output = Vec::with_capacity(bytes.len() / 8);
    for sample in bytes.chunks_exact(8) {
        let re = f32::from_le_bytes(sample[0..4].try_into().expect("four-byte float"));
        let im = f32::from_le_bytes(sample[4..8].try_into().expect("four-byte float"));
        output.push(Complex32::new(re * scale, im * scale));
    }
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validates_receiver_configuration() {
        let valid = RxConfig {
            center_frequency_hz: 100e6,
            sample_rate_hz: 250e3,
            gain: RxGain::Automatic,
        };
        assert_eq!(valid.validate().unwrap(), valid);
        assert!(
            RxConfig {
                center_frequency_hz: f64::NAN,
                ..valid
            }
            .validate()
            .is_err()
        );
        assert!(
            RxConfig {
                sample_rate_hz: 1.0,
                ..valid
            }
            .validate()
            .is_err()
        );
        assert!(
            RxConfig {
                gain: RxGain::Manual(77.0),
                ..valid
            }
            .validate()
            .is_err()
        );
    }

    #[test]
    fn validates_low_if_tune_requests() {
        let request = RxTuneRequest::with_lo_offset(100e6, 3.6e6);
        assert_eq!(request.validate().unwrap(), request);
        assert!(
            RxTuneRequest::with_lo_offset(70e6, -1.0)
                .validate()
                .is_err()
        );
        assert!(RxTuneRequest::with_lo_offset(6e9, 1.0).validate().is_err());
        assert!(
            RxTuneRequest::with_lo_offset(100e6, 8e6)
                .validate()
                .is_err()
        );
        assert!(
            RxTuneRequest::with_lo_offset(100e6, f64::NAN)
                .validate()
                .is_err()
        );
    }

    #[test]
    fn quantizes_signed_cordic_frequencies() {
        assert_eq!(ddc_frequency_word(1e6).unwrap(), (1e6, 0x1000_0000));
        assert_eq!(ddc_frequency_word(-1e6).unwrap(), (-1e6, 0xf000_0000));
        let (actual, word) = ddc_frequency_word(123_456.789).unwrap();
        assert!((actual - 123_456.789).abs() < 0.002);
        assert_ne!(word, 0);
    }

    #[test]
    fn computes_ddc_for_fm_rate() {
        let (rate, word, (scale, scalar)) = ddc_settings(250_000.0).unwrap();
        assert_eq!(rate, 250_000.0);
        assert_eq!(word, (1 << 9) | (1 << 8) | 16);
        assert!(scale.is_finite() && scale > 0.0);
        assert!(scalar > 0);
    }

    #[test]
    fn decodes_normalized_fc32() {
        let bytes = [2.0_f32.to_le_bytes(), (-4.0_f32).to_le_bytes()].concat();
        assert_eq!(
            decode_fc32(&bytes, 0.25).unwrap(),
            vec![Complex32::new(0.5, -1.0)]
        );
        assert!(decode_fc32(&bytes[..7], 1.0).is_err());
    }

    #[test]
    fn selects_expected_b200_bands() {
        assert_eq!(receiver_misc_word(100e6), (1 << 6) | (1 << 3));
        assert_eq!(receiver_misc_word(3e9), (1 << 6) | (1 << 4));
    }

    #[test]
    fn startup_discards_packets_queued_before_sequence_reset() {
        let mut state = RxSequenceState::Synchronizing;
        assert_eq!(state.observe(1_809), SequenceDisposition::Discard);
        assert_eq!(state.observe(1_810), SequenceDisposition::Discard);
        assert_eq!(state.observe(0), SequenceDisposition::Accept);
        assert_eq!(state.observe(1), SequenceDisposition::Accept);
    }

    #[test]
    fn sequence_tracking_reports_loss_and_resynchronizes() {
        let mut state = RxSequenceState::Tracking(1_810);
        assert_eq!(
            state.observe(0),
            SequenceDisposition::Overflow {
                expected: 1_810,
                actual: 0,
            }
        );
        assert_eq!(state.observe(1), SequenceDisposition::Accept);
    }

    #[test]
    fn parses_receive_context_code_in_either_word_byte_order() {
        assert_eq!(receive_context_code(&[8, 0, 0, 0]).unwrap(), 8);
        assert_eq!(receive_context_code(&[0, 0, 0, 8]).unwrap(), 8);
        assert!(receive_context_code(&[8, 0, 0]).is_err());
    }
}
