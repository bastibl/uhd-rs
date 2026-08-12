//! Receive streaming for an initialized B200.
//!
//! The current implementation configures channel zero, the `RX2` antenna,
//! native `fc32` samples on the USB wire, and the FPGA DDC. It deliberately
//! checks that the AD9361 has already received its normal cold-start
//! initialization; loading firmware and an FPGA image alone is not sufficient.

use std::time::Duration;

use futures_timer::Delay;

use super::{Ad9361Io, B2xxDevice, B2xxIdentity, Fx3State, Product, RadioControl, StreamId};
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
    identity: B2xxIdentity,
    product: Product,
    center_frequency_hz: f64,
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
            .field("sample_rate_hz", &self.sample_rate_hz)
            .field("gain", &self.gain)
            .field("streaming", &self.streaming)
            .finish_non_exhaustive()
    }
}

impl B2xxReceiver {
    /// Open and configure a B200 whose FX3 firmware, FPGA image, and
    /// AD9361 cold-start initialization are already running.
    pub async fn open(device: B2xxDevice, config: RxConfig) -> Result<Self> {
        let config = config.validate()?;
        device.check_firmware_compatibility().await?;
        if device.fx3_state().await? != Fx3State::Running {
            return Err(Error::Unsupported(
                "the B2xx FPGA is not running; load its FPGA image before opening a receiver",
            ));
        }
        let identity = device.identity().await?;
        let product = identity.product.ok_or(Error::UnsupportedDevice {
            vendor_id: device.info().vendor_id,
            product_id: device.info().product_id,
        })?;
        if product != Product::B200 {
            return Err(Error::Unsupported(
                "the receive streamer currently supports only B200 hardware",
            ));
        }
        if identity.revision < 5 {
            return Err(Error::Unsupported(
                "the receive streamer's RF routing requires a revision 5 or newer B200",
            ));
        }

        let transport = device.open_transport().await?;
        let mut receiver = Self {
            control: transport.into_radio_control(StreamId::LocalControl),
            identity,
            product,
            center_frequency_hz: config.center_frequency_hz,
            sample_rate_hz: config.sample_rate_hz,
            gain: config.gain,
            host_scale: 1.0,
            sequence_state: RxSequenceState::Synchronizing,
            streaming: false,
        };
        receiver.require_initialized_ad9361().await?;
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

    #[must_use]
    pub const fn sample_rate_hz(&self) -> f64 {
        self.sample_rate_hz
    }

    #[must_use]
    pub const fn gain(&self) -> RxGain {
        self.gain
    }

    /// Retune the AD9361 receive synthesizer and B200 RF filter selection.
    pub async fn set_center_frequency(&mut self, frequency_hz: f64) -> Result<f64> {
        validate_range("center frequency", frequency_hz, MIN_RF_HZ, MAX_RF_HZ)?;
        self.control.set_stream(StreamId::LocalControl);
        let misc = receiver_misc_word(frequency_hz);
        self.control.poke32(SR_CORE_MISC, misc).await?;
        let actual = tune_initialized_ad9361(&mut self.control, frequency_hz).await?;
        self.center_frequency_hz = actual;
        Ok(actual)
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
        self.control.poke32(RX_DSP_FREQUENCY, 0).await?;
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
        self.control.set_stream(StreamId::LocalControl);
        let mut codec = Ad9361Io::new(super::B2xxSpi::new(&mut self.control));
        match gain {
            RxGain::Automatic => set_slow_agc(&mut codec).await?,
            RxGain::Manual(value) => {
                set_manual_gain(&mut codec, value.round() as u8).await?;
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

    async fn require_initialized_ad9361(&mut self) -> Result<()> {
        self.control.set_stream(StreamId::LocalControl);
        let mut codec = Ad9361Io::new(super::B2xxSpi::new(&mut self.control));
        match codec.read_register(0x017).await? & 0x0f {
            0x05 | 0x0a => Ok(()),
            state => Err(Error::Ad9361NotInitialized { state }),
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

async fn tune_initialized_ad9361(control: &mut RadioControl, frequency_hz: f64) -> Result<f64> {
    control.set_stream(StreamId::LocalControl);
    let mut codec = Ad9361Io::new(super::B2xxSpi::new(control));
    let initial_state = codec.read_register(0x017).await? & 0x0f;
    let return_to_fdd = match initial_state {
        0x05 => false,
        0x0a => true,
        state => return Err(Error::Ad9361NotInitialized { state }),
    };
    if return_to_fdd {
        codec.write_register(0x014, 0x01).await?;
        wait_for_ad9361_state(&mut codec, 0x05).await?;
    }

    let input_selection = codec.read_register(0x004).await? & 0xc0;
    let input_selection = input_selection
        | if frequency_hz < 2_200_000_000.0 {
            0x30
        } else if frequency_hz < 4_000_000_000.0 {
            0x0c
        } else {
            0x03
        };
    codec.write_register(0x004, input_selection).await?;
    let rx_filter = codec.read_register(0x003).await?;
    codec.write_register(0x003, rx_filter | 0x40).await?;

    let (actual_frequency, divider_index, nint, nfrac, vco_rate) = rf_synth_settings(frequency_hz)?;
    let synth = synth_settings(vco_rate);
    codec.write_register(0x23a, 0x40 | synth[0]).await?;
    codec.write_register(0x239, 0xc0 | synth[1]).await?;
    codec
        .write_register(0x242, synth[2] | (synth[3] << 3))
        .await?;
    codec.write_register(0x238, synth[4] << 3).await?;
    codec.write_register(0x245, 0).await?;
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

    codec.write_register(0x233, nfrac as u8).await?;
    codec.write_register(0x234, (nfrac >> 8) as u8).await?;
    codec.write_register(0x235, (nfrac >> 16) as u8).await?;
    codec.write_register(0x232, (nint >> 8) as u8).await?;
    codec.write_register(0x231, nint as u8).await?;
    let dividers = codec.read_register(0x005).await?;
    codec
        .write_register(0x005, (dividers & 0xf0) | divider_index)
        .await?;
    Delay::new(Duration::from_millis(2)).await;
    if codec.read_register(0x247).await? & 0x02 == 0 {
        return Err(Error::Ad9361PllUnlocked {
            frequency_hz: actual_frequency,
        });
    }

    if return_to_fdd {
        codec.write_register(0x014, 0x21).await?;
        wait_for_ad9361_state(&mut codec, 0x0a).await?;
    }
    Ok(actual_frequency)
}

fn rf_synth_settings(frequency_hz: f64) -> Result<(f64, u8, u32, u32, f64)> {
    const REFERENCE_HZ: f64 = 80_000_000.0;
    const MODULUS: f64 = 8_388_593.0;
    for divider_index in 0_u8..=6 {
        let divider = f64::from(2_u32 << divider_index);
        let vco_rate = frequency_hz * divider;
        if (6_000_000_000.0..=12_000_000_000.0).contains(&vco_rate) {
            let nint = (vco_rate / REFERENCE_HZ).floor() as u32;
            let nfrac = (((vco_rate / REFERENCE_HZ) - f64::from(nint)) * MODULUS).round() as u32;
            let actual_vco = REFERENCE_HZ * (f64::from(nint) + f64::from(nfrac) / MODULUS);
            return Ok((actual_vco / divider, divider_index, nint, nfrac, actual_vco));
        }
    }
    Err(Error::InvalidArgument(
        "center frequency cannot be represented by the AD9361 RF PLL".into(),
    ))
}

fn synth_settings(vco_rate: f64) -> &'static [u8; 12] {
    let index = VCO_INDEX_HZ
        .iter()
        .position(|boundary| vco_rate > *boundary)
        .unwrap_or(VCO_INDEX_HZ.len() - 1);
    &SYNTH_CAL_LUT[index]
}

async fn set_manual_gain(codec: &mut Ad9361Io<'_>, gain_index: u8) -> Result<()> {
    configure_gain_control(codec, false).await?;
    codec.write_register(0x109, gain_index.min(76)).await
}

async fn set_slow_agc(codec: &mut Ad9361Io<'_>) -> Result<()> {
    let mode = (codec.read_register(0x0fa).await? & !0x03) | 0x02;
    codec.write_register(0x0fa, mode).await?;
    configure_gain_control(codec, true).await
}

async fn configure_gain_control(codec: &mut Ad9361Io<'_>, automatic: bool) -> Result<()> {
    let settings: &[(u16, u8)] = if automatic {
        &[
            (0x0fb, 0x08),
            (0x0fc, 0x23),
            (0x0fd, 0x4c),
            (0x0fe, 0x44),
            (0x100, 0x6f),
            (0x101, 0x0a),
            (0x103, 0x08),
            (0x104, 0x2f),
            (0x105, 0x3a),
            (0x106, 0x22),
            (0x107, 0x2b),
            (0x108, 0x31),
            (0x111, 0x0a),
            (0x11a, 0x1c),
            (0x120, 0x0c),
            (0x121, 0x44),
            (0x122, 0x44),
            (0x123, 0x11),
            (0x124, 0xf5),
            (0x125, 0x3b),
            (0x128, 0x03),
            (0x129, 0x56),
            (0x12a, 0x22),
        ]
    } else {
        &[
            (0x0fa, 0xe0),
            (0x0fb, 0x08),
            (0x0fc, 0x23),
            (0x0fd, 0x4c),
            (0x0fe, 0x44),
            (0x100, 0x6f),
            (0x104, 0x2f),
            (0x105, 0x3a),
            (0x107, 0x31),
            (0x108, 0x39),
            (0x109, 0x23),
            (0x10a, 0x58),
            (0x10b, 0x00),
            (0x10c, 0x23),
            (0x10d, 0x18),
            (0x10e, 0x00),
            (0x114, 0x30),
            (0x11a, 0x27),
            (0x081, 0x00),
        ]
    };
    for &(register, value) in settings {
        codec.write_register(register, value).await?;
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
    Err(Error::Ad9361StateTimeout { expected })
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

// UHD 4.8's AD9361 80 MHz-reference synthesizer table.
const VCO_INDEX_HZ: [f64; 53] = [
    12_605e6, 12_245e6, 11_906e6, 11_588e6, 11_288e6, 11_007e6, 10_742e6, 10_492e6, 10_258e6,
    10_036e6, 9_827.8e6, 9_631.1e6, 9_445.3e6, 9_269.8e6, 9_103.6e6, 8_946.3e6, 8_797e6, 8_655.3e6,
    8_520.6e6, 8_392.3e6, 8_269.9e6, 8_153.1e6, 8_041.4e6, 7_934.4e6, 7_831.8e6, 7_733.2e6,
    7_638.4e6, 7_547.1e6, 7_459e6, 7_374e6, 7_291.9e6, 7_212.4e6, 7_135.5e6, 7_061e6, 6_988.7e6,
    6_918.6e6, 6_850.6e6, 6_784.6e6, 6_720.5e6, 6_658.2e6, 6_597.8e6, 6_539.2e6, 6_482.3e6,
    6_427e6, 6_373.4e6, 6_321.4e6, 6_270.9e6, 6_222e6, 6_174.5e6, 6_128.4e6, 6_083.6e6, 6_040.1e6,
    5_997.7e6,
];

const SYNTH_CAL_LUT: [[u8; 12]; 53] = [
    [10, 0, 4, 0, 15, 8, 8, 13, 4, 13, 15, 9],
    [10, 0, 4, 0, 15, 8, 9, 13, 4, 13, 15, 9],
    [10, 0, 4, 0, 15, 8, 10, 13, 4, 13, 15, 9],
    [10, 0, 4, 0, 15, 8, 11, 13, 4, 13, 15, 9],
    [10, 0, 4, 0, 15, 8, 11, 13, 4, 13, 15, 9],
    [10, 0, 4, 0, 14, 8, 12, 13, 4, 13, 15, 9],
    [10, 0, 4, 0, 14, 8, 13, 13, 4, 13, 15, 9],
    [10, 0, 5, 1, 14, 9, 13, 13, 4, 13, 15, 9],
    [10, 0, 5, 1, 14, 9, 14, 13, 4, 13, 15, 9],
    [10, 0, 5, 1, 14, 9, 15, 13, 4, 13, 15, 9],
    [10, 0, 5, 1, 14, 9, 15, 13, 4, 13, 15, 9],
    [10, 0, 5, 1, 13, 9, 16, 13, 4, 13, 15, 9],
    [10, 0, 5, 1, 13, 9, 17, 13, 4, 13, 15, 9],
    [10, 0, 5, 1, 13, 9, 18, 13, 4, 13, 15, 9],
    [10, 0, 5, 1, 13, 9, 18, 13, 4, 13, 15, 9],
    [10, 0, 5, 1, 13, 9, 19, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 14, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 14, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 15, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 15, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 16, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 16, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 17, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 17, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 18, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 18, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 19, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 19, 13, 4, 13, 15, 9],
    [10, 1, 6, 1, 15, 11, 20, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 12, 20, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 12, 21, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 12, 21, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 14, 22, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 14, 22, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 14, 23, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 14, 23, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 14, 24, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 14, 24, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 14, 25, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 14, 25, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 14, 26, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 14, 26, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 14, 27, 13, 4, 13, 15, 9],
    [10, 1, 7, 2, 15, 14, 27, 13, 4, 13, 15, 9],
    [10, 3, 7, 3, 15, 12, 18, 13, 4, 13, 15, 9],
    [10, 3, 7, 3, 15, 12, 18, 13, 4, 13, 15, 9],
    [10, 3, 7, 3, 15, 12, 18, 13, 4, 13, 15, 9],
    [10, 3, 7, 3, 15, 12, 19, 13, 4, 13, 15, 9],
    [10, 3, 7, 3, 15, 12, 19, 13, 4, 13, 15, 9],
    [10, 3, 7, 3, 15, 12, 19, 13, 4, 13, 15, 9],
    [10, 3, 7, 3, 15, 12, 19, 13, 4, 13, 15, 9],
    [10, 3, 7, 3, 15, 12, 20, 13, 4, 13, 15, 9],
    [10, 3, 7, 3, 15, 12, 20, 13, 4, 13, 15, 9],
];

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
    fn computes_ddc_for_fm_rate() {
        let (rate, word, (scale, scalar)) = ddc_settings(250_000.0).unwrap();
        assert_eq!(rate, 250_000.0);
        assert_eq!(word, (1 << 9) | (1 << 8) | 16);
        assert!(scale.is_finite() && scale > 0.0);
        assert!(scalar > 0);
    }

    #[test]
    fn rf_synth_reproduces_exact_100mhz_settings() {
        let (actual, divider, nint, nfrac, _) = rf_synth_settings(100e6).unwrap();
        assert_eq!(actual, 100e6);
        assert_eq!(divider, 5);
        assert_eq!(nint, 80);
        assert_eq!(nfrac, 0);
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
