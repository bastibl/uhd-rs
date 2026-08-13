//! B200 AD9364 cold-start initialization and runtime control.
//!
//! The register algorithms and fixed tables are derived from UHD 4.8's
//! GPL-3.0-or-later AD9361 driver. They are expressed as async Rust so the
//! same implementation runs over native USB and WebUSB.

use std::time::Duration;

use futures_timer::Delay;

use super::{Ad9361Io, B2xxIdentity, B2xxSpi, Product, RadioControl, StreamId};
use crate::{Error, Result};

use super::ad9361_tables::{
    FIR_48_X4, FIR_64_X4, FIR_96_X4, FIR_128_X4, GAIN_TABLE_1300_TO_4000, GAIN_TABLE_4000_TO_6000,
    GAIN_TABLE_SUB_1300, HB47, HB63, HB95, HB127, SYNTH_CAL_LUT, VCO_INDEX_HZ,
};

pub(crate) const MASTER_CLOCK_HZ: f64 = 16_000_000.0;
const INITIAL_CLOCK_HZ: f64 = 50_000_000.0;
const DEFAULT_RX_FREQUENCY_HZ: f64 = 800_000_000.0;
const DEFAULT_TX_FREQUENCY_HZ: f64 = 850_000_000.0;
const DEFAULT_TUNE_FREQUENCY_HZ: f64 = 100_000_000.0;
const MAX_BANDWIDTH_HZ: f64 = 56_000_000.0;
const CALIBRATION_WINDOW_HZ: f64 = 100_000_000.0;

const SR_CORE_MISC: u32 = 16 * 4;
const CODEC_RESET: u32 = 1 << 2;
const DEFAULT_CORE_MISC: u32 = (1 << 6) | (1 << 3);
const SR_CODEC_IDLE: u32 = 22 * 4;
const RB64_CODEC_READBACK: u32 = 24;
const FPGA_SIGNATURE: u32 = 0xace0_ba5e;
const CORE_STATUS_ADDRESS: u32 = 20;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Direction {
    Rx,
    Tx,
}

trait RegisterIo {
    async fn read(&mut self, register: u16) -> Result<u8>;
    async fn write(&mut self, register: u16, value: u8) -> Result<()>;
    async fn delay(&mut self, duration: Duration);
}

impl RegisterIo for Ad9361Io<'_> {
    async fn read(&mut self, register: u16) -> Result<u8> {
        self.read_register(register).await
    }

    async fn write(&mut self, register: u16, value: u8) -> Result<()> {
        self.write_register(register, value).await
    }

    async fn delay(&mut self, duration: Duration) {
        Delay::new(duration).await;
    }
}

#[derive(Clone, Copy, Debug)]
struct ChipRegisters {
    vco_dividers: u8,
    input_selection: u8,
    rx_filter: u8,
    tx_filter: u8,
    bbpll: u8,
    bbf_tune_config: u8,
    bbf_tune_mode: u8,
}

impl Default for ChipRegisters {
    fn default() -> Self {
        Self {
            vco_dividers: 0,
            input_selection: 0x30,
            rx_filter: 0,
            tx_filter: 0,
            bbpll: 0x02,
            bbf_tune_config: 0x1e,
            bbf_tune_mode: 0x1e,
        }
    }
}

/// Host-side state needed to keep later tuning and gain changes coherent with
/// the cold-start register sequence.
pub(crate) struct Ad9361Controller {
    registers: ChipRegisters,
    rx_frequency_hz: f64,
    tx_frequency_hz: f64,
    requested_rx_frequency_hz: f64,
    requested_tx_frequency_hz: f64,
    last_rx_calibration_hz: f64,
    last_tx_calibration_hz: f64,
    baseband_bandwidth_hz: f64,
    requested_clock_hz: f64,
    requested_core_clock_hz: f64,
    bbpll_frequency_hz: f64,
    adc_clock_hz: f64,
    rx_bbf_tune_divider: u16,
    current_gain_table: u8,
    rx_gain_db: f64,
    tx_gain_db: f64,
    tx_fir_factor: u8,
    rx_fir_factor: u8,
    rx_bb_lp_bandwidth_hz: f64,
    tx_bb_lp_bandwidth_hz: f64,
    rx_tia_lp_bandwidth_hz: f64,
    tx_secondary_lp_bandwidth_hz: f64,
    dc_offset_tracking: bool,
    iq_balance_tracking: bool,
}

impl Default for Ad9361Controller {
    fn default() -> Self {
        Self {
            registers: ChipRegisters::default(),
            rx_frequency_hz: DEFAULT_RX_FREQUENCY_HZ,
            tx_frequency_hz: DEFAULT_TX_FREQUENCY_HZ,
            requested_rx_frequency_hz: 0.0,
            requested_tx_frequency_hz: 0.0,
            last_rx_calibration_hz: 0.0,
            last_tx_calibration_hz: 0.0,
            baseband_bandwidth_hz: 0.0,
            requested_clock_hz: 0.0,
            requested_core_clock_hz: 0.0,
            bbpll_frequency_hz: 0.0,
            adc_clock_hz: 0.0,
            rx_bbf_tune_divider: 0,
            current_gain_table: 0,
            rx_gain_db: 0.0,
            tx_gain_db: 0.0,
            tx_fir_factor: 0,
            rx_fir_factor: 0,
            rx_bb_lp_bandwidth_hz: 0.0,
            tx_bb_lp_bandwidth_hz: 0.0,
            rx_tia_lp_bandwidth_hz: 0.0,
            tx_secondary_lp_bandwidth_hz: 0.0,
            dc_offset_tracking: true,
            iq_balance_tracking: true,
        }
    }
}

impl Ad9361Controller {
    pub(crate) async fn initialize_b200(
        control: &mut RadioControl,
        identity: &B2xxIdentity,
    ) -> Result<Self> {
        if identity.product != Some(Product::B200) {
            return Err(Error::Unsupported(
                "AD9364 initialization currently supports only B200 hardware",
            ));
        }
        if identity.revision < 5 {
            return Err(Error::Unsupported(
                "AD9364 initialization requires a revision 5 or newer B200",
            ));
        }

        control.set_stream(StreamId::LocalControl);
        let raw_compatibility = control.peek64(0).await?;
        let signature = (raw_compatibility >> 32) as u32;
        if signature != FPGA_SIGNATURE {
            return Err(Error::FpgaSignature {
                expected: FPGA_SIGNATURE,
                actual: signature,
            });
        }
        let fpga_major = ((raw_compatibility >> 16) & 0xffff) as u16;
        if fpga_major != Product::B200.fpga_compatibility() {
            return Err(Error::FpgaCompatibility {
                expected: Product::B200.fpga_compatibility(),
                actual: fpga_major,
            });
        }
        let radio_chains = ((control.peek32(CORE_STATUS_ADDRESS).await? >> 8) & 0xff) as u8;
        if radio_chains != 1 {
            return Err(Error::RadioChainCount(radio_chains));
        }
        control
            .poke32(SR_CORE_MISC, DEFAULT_CORE_MISC | CODEC_RESET)
            .await?;
        control.poke32(SR_CORE_MISC, DEFAULT_CORE_MISC).await?;

        let mut controller = Self::default();
        {
            let mut io = Ad9361Io::new(B2xxSpi::new(control));
            controller.initialize(&mut io).await?;
            controller.set_clock_rate(&mut io, MASTER_CLOCK_HZ).await?;
            controller
                .tune(&mut io, Direction::Rx, DEFAULT_TUNE_FREQUENCY_HZ)
                .await?;
            controller.set_manual_rx_gain(&mut io, 0.0).await?;
        }
        codec_loopback_self_test(control).await?;
        {
            let mut io = Ad9361Io::new(B2xxSpi::new(control));
            controller
                .set_active_chains(&mut io, false, false, true, false)
                .await?;
        }
        Ok(controller)
    }

    pub(crate) async fn tune_rx(
        &mut self,
        control: &mut RadioControl,
        frequency_hz: f64,
    ) -> Result<f64> {
        control.set_stream(StreamId::LocalControl);
        let mut io = Ad9361Io::new(B2xxSpi::new(control));
        self.tune(&mut io, Direction::Rx, frequency_hz).await
    }

    pub(crate) async fn set_manual_rx_gain_on(
        &mut self,
        control: &mut RadioControl,
        gain_db: f64,
    ) -> Result<()> {
        control.set_stream(StreamId::LocalControl);
        let mut io = Ad9361Io::new(B2xxSpi::new(control));
        self.set_manual_rx_gain(&mut io, gain_db).await
    }

    pub(crate) async fn set_slow_agc_on(&mut self, control: &mut RadioControl) -> Result<()> {
        control.set_stream(StreamId::LocalControl);
        let mut io = Ad9361Io::new(B2xxSpi::new(control));
        let mode = (io.read(0x0fa).await? & !0x03) | 0x02;
        io.write(0x0fa, mode).await?;
        self.setup_gain_control(&mut io, true).await
    }

    async fn initialize(&mut self, io: &mut impl RegisterIo) -> Result<()> {
        io.write(0x000, 0x01).await?;
        io.write(0x000, 0x00).await?;
        io.delay(Duration::from_millis(20)).await;

        let device_id = io.read(0x037).await? & 0xf8;
        if device_id != 0x08 {
            return Err(Error::Ad9361Initialization {
                stage: "device identification",
                detail: format!("expected device ID 0x08, got 0x{device_id:02x}"),
            });
        }

        for &(register, value) in &[
            (0x3df, 0x01),
            (0x2a6, 0x0e),
            (0x2a8, 0x0e),
            (0x2ab, 0x07),
            (0x2ac, 0xff),
            // B200 uses the XTAL_N input and a 40 MHz reference.
            (0x009, 0x17),
        ] {
            io.write(register, value).await?;
        }
        io.delay(Duration::from_millis(20)).await;

        self.setup_rates(io, INITIAL_CLOCK_HZ).await?;

        // FDD dual-port DDR CMOS, I/Q swap, 1R1T timing, and B200 delays.
        for &(register, value) in &[
            (0x010, 0xc8),
            (0x011, 0x00),
            (0x012, 0x02),
            (0x006, 0x0f),
            (0x007, 0x0f),
            (0x018, 0x00),
            (0x019, 0x00),
            (0x01a, 0x00),
            (0x01b, 0x00),
            (0x023, 0xff),
            (0x026, 0x00),
            (0x030, 0x00),
            (0x031, 0x00),
            (0x032, 0x00),
            (0x033, 0x00),
            (0x022, 0x0a),
            (0x00b, 0x00),
            (0x00c, 0x00),
            (0x00d, 0x00),
            (0x00f, 0x04),
            (0x01c, 0x10),
            (0x01d, 0x01),
            (0x035, 0x01),
            (0x036, 0xff),
            (0x03a, 0x27),
            (0x020, 0x00),
            (0x027, 0x03),
            (0x028, 0x00),
            (0x029, 0x00),
            (0x02a, 0x00),
            (0x02b, 0x00),
            (0x02c, 0x00),
            (0x02d, 0x00),
            (0x02e, 0x00),
            (0x02f, 0x00),
            (0x261, 0x00),
            (0x2a1, 0x00),
            (0x248, 0x0b),
            (0x288, 0x0b),
            (0x246, 0x02),
            (0x286, 0x02),
            (0x249, 0x8e),
            (0x289, 0x8e),
            (0x23b, 0x80),
            (0x27b, 0x80),
            (0x243, 0x0d),
            (0x283, 0x0d),
            (0x23d, 0x00),
            (0x27d, 0x00),
            (0x015, 0x04),
            (0x014, 0x05),
            (0x013, 0x01),
        ] {
            io.write(register, value).await?;
        }
        io.delay(Duration::from_millis(1)).await;

        self.calibrate_synth_charge_pumps(io).await?;
        self.tune_helper(io, Direction::Rx, self.rx_frequency_hz)
            .await?;
        self.tune_helper(io, Direction::Tx, self.tx_frequency_hz)
            .await?;
        self.program_mixer_gm_subtable(io).await?;
        self.program_gain_table(io).await?;
        self.setup_gain_control(io, false).await?;
        self.set_bandwidth(io, Direction::Rx, self.baseband_bandwidth_hz)
            .await?;
        self.set_bandwidth(io, Direction::Tx, self.baseband_bandwidth_hz)
            .await?;
        self.setup_adc(io).await?;
        self.calibrate_baseband_dc_offset(io).await?;
        self.calibrate_rf_dc_offset(io).await?;
        self.calibrate_rx_quadrature(io).await?;
        self.configure_tracking(io).await?;
        self.last_rx_calibration_hz = self.rx_frequency_hz;
        self.last_tx_calibration_hz = self.tx_frequency_hz;

        for &(register, value) in &[
            (0x012, 0x02),
            (0x013, 0x01),
            (0x015, 0x04),
            (0x073, 0x00),
            (0x074, 0x00),
            (0x075, 0x00),
            (0x076, 0x00),
            (0x150, 0x0e),
            (0x151, 0x00),
            (0x152, 0xff),
            (0x153, 0x00),
            (0x154, 0x00),
            (0x155, 0x00),
            (0x156, 0x00),
            (0x157, 0x00),
            (0x158, 0x0d),
            (0x15c, 0x67),
        ] {
            io.write(register, value).await?;
        }
        self.set_manual_rx_gain(io, 0.0).await?;
        self.set_tx_gain(io, 0.0).await?;
        self.set_active_chains(io, true, false, false, false)
            .await?;
        io.write(0x014, 0x21).await?;
        io.write(0x077, 0x00).await?;
        Ok(())
    }

    async fn setup_rates(&mut self, io: &mut impl RegisterIo, rate: f64) -> Result<f64> {
        self.requested_clock_hz = rate;
        let div_factor = if rate < 0.33e6 {
            self.registers.rx_filter = 0xef;
            self.registers.tx_filter = 0xef;
            self.tx_fir_factor = 4;
            self.rx_fir_factor = 4;
            48
        } else if rate < 0.66e6 {
            self.registers.rx_filter = 0xdf;
            self.registers.tx_filter = 0xdf;
            self.tx_fir_factor = 4;
            self.rx_fir_factor = 4;
            32
        } else if rate <= 20e6 {
            self.registers.rx_filter = 0xde;
            self.registers.tx_filter = 0xde;
            self.tx_fir_factor = 2;
            self.rx_fir_factor = 2;
            16
        } else if rate < 23e6 {
            self.registers.rx_filter = 0xee;
            self.registers.tx_filter = 0xe6;
            self.tx_fir_factor = 2;
            self.rx_fir_factor = 2;
            24
        } else if rate < 41e6 {
            self.registers.rx_filter = 0xde;
            self.registers.tx_filter = 0xce;
            self.tx_fir_factor = 2;
            self.rx_fir_factor = 2;
            16
        } else if rate <= 58e6 {
            self.registers.rx_filter = 0xe6;
            self.registers.tx_filter = 0xe2;
            self.tx_fir_factor = 2;
            self.rx_fir_factor = 2;
            12
        } else if rate <= 61.44e6 {
            self.registers.rx_filter = 0xce;
            self.registers.tx_filter = 0xd2;
            self.tx_fir_factor = 2;
            self.rx_fir_factor = 2;
            8
        } else {
            return Err(Error::Ad9361Initialization {
                stage: "clock configuration",
                detail: format!("unsupported master clock rate {rate} Hz"),
            });
        };

        let adc_clock = self.tune_bb_vco(io, rate * f64::from(div_factor)).await?;
        let dac_clock = if adc_clock > 336e6 {
            self.registers.bbpll |= 0x08;
            adc_clock / 2.0
        } else {
            self.registers.bbpll &= !0x08;
            adc_clock
        };
        for &(register, value) in &[
            (0x002, self.registers.tx_filter),
            (0x003, self.registers.rx_filter),
            (0x004, self.registers.input_selection),
            (0x00a, self.registers.bbpll),
        ] {
            io.write(register, value).await?;
        }
        self.baseband_bandwidth_hz = adc_clock / f64::from(div_factor);

        let max_tx_taps = ((16.0 * (dac_clock / rate).round()) as usize)
            .min(128)
            .min(if self.tx_fir_factor == 1 { 64 } else { 128 });
        let max_rx_taps = ((16.0 * (adc_clock / rate).round()) as usize).min(128);
        self.program_fir(
            io,
            Direction::Tx,
            self.tx_fir_factor,
            supported_tap_count(max_tx_taps),
        )
        .await?;
        self.program_fir(
            io,
            Direction::Rx,
            self.rx_fir_factor,
            supported_tap_count(max_rx_taps),
        )
        .await?;
        Ok(self.baseband_bandwidth_hz)
    }

    async fn program_fir(
        &self,
        io: &mut impl RegisterIo,
        direction: Direction,
        factor: u8,
        tap_count: usize,
    ) -> Result<()> {
        let coefficients = fir_coefficients(factor, tap_count)?;
        let base = match direction {
            Direction::Rx => 0x0f0,
            Direction::Tx => 0x060,
        };
        let tap_bits = ((((tap_count / 16) - 1) & 0x07) << 5) as u8;
        let chain_bits = 0x03 << 3;
        io.write(base + 5, tap_bits | chain_bits | 0x02).await?;
        io.delay(Duration::from_millis(1)).await;
        for address in 0..128_u16 {
            let coefficient = coefficients.get(usize::from(address)).copied().unwrap_or(0);
            let bytes = coefficient.to_le_bytes();
            io.write(base, address as u8).await?;
            io.write(base + 1, bytes[0]).await?;
            io.write(base + 2, bytes[1]).await?;
            io.write(base + 5, tap_bits | chain_bits | 0x06).await?;
            io.write(base + 4, 0).await?;
            io.write(base + 4, 0).await?;
        }
        io.write(base + 5, tap_bits | chain_bits | 0x02).await?;
        io.write(base + 5, tap_bits | chain_bits).await?;
        if direction == Direction::Rx {
            io.write(base + 6, 0x02).await?;
        }
        Ok(())
    }

    async fn tune_bb_vco(&mut self, io: &mut impl RegisterIo, rate: f64) -> Result<f64> {
        if nearly_equal(rate, self.requested_core_clock_hz) {
            return Ok(self.adc_clock_hz);
        }
        self.requested_core_clock_hz = rate;
        let Some((divider_index, divider, vco_rate)) = (1_u8..=6).find_map(|index| {
            let divider = 1_u32 << index;
            let vco_rate = rate * f64::from(divider);
            (672e6..=1_430e6)
                .contains(&vco_rate)
                .then_some((index, divider, vco_rate))
        }) else {
            return Err(Error::Ad9361Initialization {
                stage: "BBPLL configuration",
                detail: format!("cannot synthesize ADC clock from {rate} Hz"),
            });
        };
        const REFERENCE_HZ: f64 = 40e6;
        const MODULUS: f64 = 2_088_960.0;
        let n_integer = (vco_rate / REFERENCE_HZ).floor() as u32;
        let n_fractional =
            (((vco_rate / REFERENCE_HZ) - f64::from(n_integer)) * MODULUS).round() as u32;
        let actual_vco = REFERENCE_HZ * (f64::from(n_integer) + f64::from(n_fractional) / MODULUS);
        let charge_pump = ((150e-6 * (actual_vco / 1_280e6)) / 25e-6) as u8 - 1;
        for &(register, value) in &[
            (0x045, 0x00),
            (0x046, charge_pump & 0x3f),
            (0x048, 0xe8),
            (0x049, 0x5b),
            (0x04a, 0x35),
            (0x04b, 0xe0),
            (0x04e, 0x10),
            (0x043, n_fractional as u8),
            (0x042, (n_fractional >> 8) as u8),
            (0x041, (n_fractional >> 16) as u8),
            (0x044, n_integer as u8),
        ] {
            io.write(register, value).await?;
        }
        io.write(0x03f, 0x05).await?;
        io.write(0x03f, 0x01).await?;
        io.write(0x04c, 0x86).await?;
        io.write(0x04d, 0x01).await?;
        io.write(0x04d, 0x05).await?;
        wait_for_mask(
            io,
            0x05e,
            0x80,
            true,
            1_001,
            Duration::from_millis(2),
            "BBPLL lock",
        )
        .await?;
        self.registers.bbpll = (self.registers.bbpll & 0xf8) | divider_index;
        self.bbpll_frequency_hz = actual_vco;
        self.adc_clock_hz = actual_vco / f64::from(divider);
        Ok(self.adc_clock_hz)
    }

    async fn setup_synth(
        &self,
        io: &mut impl RegisterIo,
        direction: Direction,
        vco_rate: f64,
    ) -> Result<()> {
        let settings = synth_settings(vco_rate);
        let base = if direction == Direction::Rx {
            0x230
        } else {
            0x270
        };
        for &(offset, value) in &[
            (0x0a, 0x40 | settings[0]),
            (0x09, 0xc0 | settings[1]),
            (0x12, settings[2] | (settings[3] << 3)),
            (0x08, settings[4] << 3),
            (0x15, 0x00),
            (0x21, settings[5]),
            (0x20, 0x70),
            (0x0b, 0x80 | settings[6]),
            (0x0e, settings[8] | (settings[7] << 4)),
            (0x0f, settings[10] | (settings[9] << 4)),
            (0x10, settings[11]),
        ] {
            io.write(base + offset, value).await?;
        }
        Ok(())
    }

    async fn tune_helper(
        &mut self,
        io: &mut impl RegisterIo,
        direction: Direction,
        frequency_hz: f64,
    ) -> Result<f64> {
        let (actual, divider_index, n_integer, n_fractional, vco_rate) =
            rf_synth_settings(frequency_hz)?;
        match direction {
            Direction::Rx => {
                self.requested_rx_frequency_hz = frequency_hz;
                let port = if frequency_hz < 2.2e9 {
                    0x30
                } else if frequency_hz < 4e9 {
                    0x0c
                } else {
                    0x03
                };
                self.registers.input_selection = (self.registers.input_selection & 0xc0) | port;
                self.registers.vco_dividers = (self.registers.vco_dividers & 0xf0) | divider_index;
            }
            Direction::Tx => {
                self.requested_tx_frequency_hz = frequency_hz;
                if frequency_hz < 2.5e9 {
                    self.registers.input_selection |= 0x40;
                } else {
                    self.registers.input_selection &= !0x40;
                }
                self.registers.vco_dividers =
                    (self.registers.vco_dividers & 0x0f) | (divider_index << 4);
            }
        }
        io.write(0x004, self.registers.input_selection).await?;
        self.setup_synth(io, direction, vco_rate).await?;
        let base = if direction == Direction::Rx {
            0x230
        } else {
            0x270
        };
        for &(offset, value) in &[
            (3, n_fractional as u8),
            (4, (n_fractional >> 8) as u8),
            (5, (n_fractional >> 16) as u8),
            (2, (n_integer >> 8) as u8),
            (1, n_integer as u8),
        ] {
            io.write(base + offset, value).await?;
        }
        io.write(0x005, self.registers.vco_dividers).await?;
        io.delay(Duration::from_millis(2)).await;
        let lock_register = if direction == Direction::Rx {
            0x247
        } else {
            0x287
        };
        if io.read(lock_register).await? & 0x02 == 0 {
            return Err(Error::Ad9361PllUnlocked {
                frequency_hz: actual,
            });
        }
        match direction {
            Direction::Rx => self.rx_frequency_hz = actual,
            Direction::Tx => self.tx_frequency_hz = actual,
        }
        Ok(actual)
    }

    async fn calibrate_synth_charge_pumps(&self, io: &mut impl RegisterIo) -> Result<()> {
        let state = io.read(0x017).await? & 0x0f;
        if state != 0x05 {
            return Err(Error::Ad9361Initialization {
                stage: "synthesizer charge-pump calibration",
                detail: format!("expected ALERT state 0x5, got 0x{state:x}"),
            });
        }
        io.write(0x23d, 0x04).await?;
        wait_for_mask(
            io,
            0x244,
            0x80,
            true,
            6,
            Duration::from_millis(1),
            "RX charge-pump calibration",
        )
        .await?;
        io.write(0x23d, 0x00).await?;
        io.write(0x27d, 0x04).await?;
        wait_for_mask(
            io,
            0x284,
            0x80,
            true,
            6,
            Duration::from_millis(1),
            "TX charge-pump calibration",
        )
        .await?;
        io.write(0x27d, 0x00).await
    }

    async fn program_mixer_gm_subtable(&self, io: &mut impl RegisterIo) -> Result<()> {
        const GAIN: [u8; 16] = [
            0x78, 0x74, 0x70, 0x6c, 0x68, 0x64, 0x60, 0x5c, 0x58, 0x54, 0x50, 0x4c, 0x48, 0x30,
            0x18, 0x00,
        ];
        const GM: [u8; 16] = [
            0x00, 0x0d, 0x15, 0x1b, 0x21, 0x25, 0x29, 0x2c, 0x2f, 0x31, 0x33, 0x34, 0x35, 0x3a,
            0x3d, 0x3e,
        ];
        io.write(0x13f, 0x02).await?;
        for (offset, index) in (0_u8..16).rev().enumerate() {
            io.write(0x138, index).await?;
            io.write(0x139, GAIN[offset]).await?;
            io.write(0x13a, 0).await?;
            io.write(0x13b, GM[offset]).await?;
            io.write(0x13f, 0x06).await?;
            io.write(0x13c, 0).await?;
            io.write(0x13c, 0).await?;
        }
        io.write(0x13f, 0x02).await?;
        io.write(0x13c, 0).await?;
        io.write(0x13c, 0).await?;
        io.write(0x13f, 0).await
    }

    async fn program_gain_table(&mut self, io: &mut impl RegisterIo) -> Result<()> {
        let (table_number, table) = gain_table(self.rx_frequency_hz)?;
        if self.current_gain_table == table_number {
            return Ok(());
        }
        self.current_gain_table = table_number;
        io.write(0x137, 0x1a).await?;
        for index in 0_u8..91 {
            let entry = table.get(usize::from(index)).copied().unwrap_or([0; 3]);
            io.write(0x130, index).await?;
            io.write(0x131, entry[0]).await?;
            io.write(0x132, entry[1]).await?;
            io.write(0x133, entry[2]).await?;
            io.write(0x137, 0x1e).await?;
            io.write(0x134, 0).await?;
            io.write(0x134, 0).await?;
        }
        io.write(0x137, 0x1a).await?;
        io.write(0x134, 0).await?;
        io.write(0x134, 0).await?;
        io.write(0x137, 0).await
    }

    async fn setup_gain_control(&self, io: &mut impl RegisterIo, automatic: bool) -> Result<()> {
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
            io.write(register, value).await?;
        }
        Ok(())
    }

    async fn set_manual_rx_gain(&mut self, io: &mut impl RegisterIo, gain_db: f64) -> Result<()> {
        self.setup_gain_control(io, false).await?;
        self.rx_gain_db = gain_db.clamp(0.0, 76.0);
        io.write(0x109, self.rx_gain_db as u8).await
    }

    async fn set_tx_gain(&mut self, io: &mut impl RegisterIo, gain_db: f64) -> Result<()> {
        self.tx_gain_db = gain_db;
        let attenuation = ((89.75 - gain_db).clamp(0.0, 89.75) * 4.0) as u16;
        io.write(0x073, attenuation as u8).await?;
        io.write(0x074, (attenuation >> 8) as u8 & 1).await?;
        io.write(0x07c, 0x40).await
    }

    async fn reprogram_gains(&mut self, io: &mut impl RegisterIo) -> Result<()> {
        let rx_gain = self.rx_gain_db;
        let tx_gain = self.tx_gain_db;
        self.set_manual_rx_gain(io, rx_gain).await?;
        self.set_tx_gain(io, tx_gain).await
    }

    async fn configure_tracking(&self, io: &mut impl RegisterIo) -> Result<()> {
        io.write(0x18b, if self.dc_offset_tracking { 0xad } else { 0x8d })
            .await?;
        io.write(0x169, if self.iq_balance_tracking { 0xcf } else { 0xc0 })
            .await
    }

    async fn set_active_chains(
        &mut self,
        io: &mut impl RegisterIo,
        tx1: bool,
        tx2: bool,
        rx1: bool,
        rx2: bool,
    ) -> Result<()> {
        self.registers.tx_filter &= 0x3f;
        self.registers.rx_filter &= 0x3f;
        self.registers.tx_filter |= (u8::from(tx1) << 6) | (u8::from(tx2) << 7);
        self.registers.rx_filter |= (u8::from(rx1) << 6) | (u8::from(rx2) << 7);
        let return_to_fdd = io.read(0x017).await? & 0x0f == 0x0a;
        if return_to_fdd {
            io.write(0x014, 0x01).await?;
            wait_for_state_not(io, &[0x0a, 0x0b], "FDD flush").await?;
        }
        io.write(0x002, self.registers.tx_filter).await?;
        io.write(0x003, self.registers.rx_filter).await?;
        if tx1 || tx2 {
            self.calibrate_tx_quadrature(io).await?;
        }
        if return_to_fdd {
            io.write(0x014, 0x21).await?;
            wait_for_state(io, 0x0a, "return to FDD").await?;
        }
        Ok(())
    }

    async fn set_bandwidth(
        &mut self,
        io: &mut impl RegisterIo,
        direction: Direction,
        requested_hz: f64,
    ) -> Result<f64> {
        let bandwidth = requested_hz.clamp(200e3, MAX_BANDWIDTH_HZ);
        match direction {
            Direction::Rx => {
                self.rx_bb_lp_bandwidth_hz =
                    self.calibrate_rx_baseband_filter(io, bandwidth).await?;
                self.rx_tia_lp_bandwidth_hz = self.calibrate_rx_tia(io, bandwidth).await?;
            }
            Direction::Tx => {
                self.tx_bb_lp_bandwidth_hz =
                    self.calibrate_tx_baseband_filter(io, bandwidth).await?;
                self.tx_secondary_lp_bandwidth_hz =
                    self.calibrate_tx_secondary_filter(io, bandwidth).await?;
            }
        }
        Ok(bandwidth)
    }

    async fn calibrate_rx_baseband_filter(
        &mut self,
        io: &mut impl RegisterIo,
        requested_hz: f64,
    ) -> Result<f64> {
        let bandwidth = (requested_hz / 2.0)
            .min(self.baseband_bandwidth_hz / 2.0)
            .clamp(0.143e6, 28e6);
        let tune_clock = 1.4 * bandwidth * 2.0 * std::f64::consts::PI / std::f64::consts::LN_2;
        self.rx_bbf_tune_divider = (self.bbpll_frequency_hz / tune_clock).ceil().min(511.0) as u16;
        self.registers.bbf_tune_config =
            (self.registers.bbf_tune_config & 0xfe) | ((self.rx_bbf_tune_divider >> 8) as u8 & 1);
        let mhz = bandwidth / 1e6;
        let khz = ((((mhz - mhz.floor()) * 1_000.0) / 7.8125).round() as u8).min(127);
        for &(register, value) in &[
            (0x1fb, mhz as u8),
            (0x1fc, khz),
            (0x1f8, self.rx_bbf_tune_divider as u8),
            (0x1f9, self.registers.bbf_tune_config),
            (0x1d5, 0x3f),
            (0x1c0, 0x03),
            (0x1e2, 0x02),
            (0x1e3, 0x02),
            (0x016, 0x80),
        ] {
            io.write(register, value).await?;
        }
        let result = wait_for_mask(
            io,
            0x016,
            0x80,
            false,
            101,
            Duration::from_millis(1),
            "RX baseband-filter calibration",
        )
        .await;
        io.write(0x1e2, 0x03).await?;
        io.write(0x1e3, 0x03).await?;
        result?;
        Ok(bandwidth)
    }

    async fn calibrate_tx_baseband_filter(
        &mut self,
        io: &mut impl RegisterIo,
        requested_hz: f64,
    ) -> Result<f64> {
        let bandwidth = (requested_hz / 2.0)
            .min(self.baseband_bandwidth_hz / 2.0)
            .clamp(0.391e6, 20e6);
        let tune_clock = 1.6 * bandwidth * 2.0 * std::f64::consts::PI / std::f64::consts::LN_2;
        let divider = (self.bbpll_frequency_hz / tune_clock).ceil().min(511.0) as u16;
        self.registers.bbf_tune_mode =
            (self.registers.bbf_tune_mode & 0xfe) | ((divider >> 8) as u8 & 1);
        io.write(0x0d6, divider as u8).await?;
        io.write(0x0d7, self.registers.bbf_tune_mode).await?;
        io.write(0x0ca, 0x22).await?;
        io.write(0x016, 0x40).await?;
        let result = wait_for_mask(
            io,
            0x016,
            0x40,
            false,
            101,
            Duration::from_millis(1),
            "TX baseband-filter calibration",
        )
        .await;
        io.write(0x0ca, 0x26).await?;
        result?;
        Ok(bandwidth)
    }

    async fn calibrate_tx_secondary_filter(
        &self,
        io: &mut impl RegisterIo,
        requested_hz: f64,
    ) -> Result<f64> {
        let bandwidth = (requested_hz / 2.0)
            .min(self.baseband_bandwidth_hz / 2.0)
            .clamp(0.54e6, 20e6);
        let mhz = bandwidth / 1e6;
        let corner = 5.0 * mhz * 2.0 * std::f64::consts::PI;
        let mut resistance = 100;
        let mut capacitance = 0;
        for _ in 0..=3 {
            capacitance =
                (0.5 + (1.0 / (corner * f64::from(resistance) * 1e6)) * 1e12).floor() as i32 - 12;
            if capacitance <= 63 {
                break;
            }
            resistance *= 2;
        }
        capacitance = capacitance.min(63);
        let frequency_setting = if mhz * 2.0 <= 9.0 {
            0x59
        } else if mhz * 2.0 <= 24.0 {
            0x56
        } else {
            0x57
        };
        let resistance_setting = match resistance {
            100 => 0x0c,
            200 => 0x04,
            400 => 0x03,
            800 => 0x01,
            _ => 0x0c,
        };
        io.write(0x0d2, capacitance as u8).await?;
        io.write(0x0d1, resistance_setting).await?;
        io.write(0x0d0, frequency_setting).await?;
        Ok(bandwidth)
    }

    async fn calibrate_rx_tia(&self, io: &mut impl RegisterIo, requested_hz: f64) -> Result<f64> {
        let c3_msb = u32::from(io.read(0x1eb).await? & 0x3f);
        let c3_lsb = u32::from(io.read(0x1ec).await? & 0x7f);
        let resistance_setting = u32::from(io.read(0x1e6).await? & 0x07);
        let bandwidth = (requested_hz / 2.0)
            .min(self.baseband_bandwidth_hz / 2.0)
            .clamp(0.40e6, 28e6);
        let ceil_mhz = (bandwidth / 1e6).ceil();
        let register_1db = if ceil_mhz <= 3.0 {
            0xe0
        } else if ceil_mhz <= 10.0 {
            0x60
        } else {
            0x20
        };
        let c_bbf = c3_msb * 160 + c3_lsb * 10 + 140;
        let r_2346 = 18_300 * resistance_setting;
        let c_tia = f64::from(c_bbf * r_2346) * 0.56 / 3_500.0;
        let (register_1dc, register_1dd) = if c_tia > 2_920.0 {
            (
                0x40,
                ((0.5 + (c_tia - 400.0) / 320.0).floor() as u8).min(127),
            )
        } else {
            (
                ((0.5 + (c_tia - 400.0) / 40.0).floor() as u8).wrapping_add(0x40),
                0,
            )
        };
        for &(register, value) in &[
            (0x1db, register_1db),
            (0x1dd, register_1dd),
            (0x1df, register_1dd),
            (0x1dc, register_1dc),
            (0x1de, register_1dc),
        ] {
            io.write(register, value).await?;
        }
        Ok(bandwidth)
    }

    async fn setup_adc(&self, io: &mut impl RegisterIo) -> Result<()> {
        let mut bbbw_mhz = (((self.bbpll_frequency_hz / 1e6)
            / f64::from(self.rx_bbf_tune_divider))
            * std::f64::consts::LN_2)
            / (1.4 * 2.0 * std::f64::consts::PI);
        bbbw_mhz = bbbw_mhz.clamp(0.20, 28.0);
        let c3_msb = f64::from(io.read(0x1eb).await? & 0x3f);
        let c3_lsb = f64::from(io.read(0x1ec).await? & 0x7f);
        let r_2346 = f64::from(io.read(0x1e6).await? & 0x07);
        let fs_adc = self.adc_clock_hz / 1e6;
        let correction = if bbbw_mhz < 18.0 {
            1.0
        } else {
            1.0 + 0.01 * (bbbw_mhz - 18.0)
        };
        let rc_time_constant = 1.0
            / ((1.4 * 2.0 * std::f64::consts::PI)
                * (18_300.0 * r_2346)
                * ((160e-15 * c3_msb) + (10e-15 * c3_lsb) + 140e-15)
                * (bbbw_mhz * 1e6)
                * correction);
        if !rc_time_constant.is_finite() || rc_time_constant <= 0.0 {
            return Err(Error::Ad9361Initialization {
                stage: "ADC configuration",
                detail: "RX baseband filter calibration returned invalid RC values".into(),
            });
        }
        let scale_resistance = (1.0 / rc_time_constant).sqrt();
        let scale_capacitance = scale_resistance;
        let scale_snr = if self.adc_clock_hz < 80e6 {
            1.0
        } else {
            1.584_893_192
        };
        let max_snr = 4.0;
        let snr_scale = (max_snr * fs_adc / 640.0).sqrt().min(1.0);
        let mut data = [0_u8; 40];
        data[3] = 0x24;
        data[4] = 0x24;
        data[7] = clipped_u8(-0.5 + 80.0 * scale_snr * scale_resistance * snr_scale, 124);
        data[8] = clipped_u8(
            0.5 + 20.0 * (640.0 / fs_adc) * (f64::from(data[7]) / 80.0)
                / (scale_resistance * scale_capacitance),
            255,
        );
        data[10] = clipped_u8(-0.5 + 77.0 * scale_resistance * snr_scale, 127);
        data[9] = clipped_u8(0.8 * f64::from(data[10]), 127);
        data[11] = clipped_u8(
            0.5 + 20.0 * (640.0 / fs_adc) * (f64::from(data[10]) / 77.0)
                / (scale_resistance * scale_capacitance),
            255,
        );
        data[12] = clipped_u8(-0.5 + 80.0 * scale_resistance * snr_scale, 127);
        data[13] = clipped_u8(
            -1.5 + 20.0 * (640.0 / fs_adc) * (f64::from(data[12]) / 80.0)
                / (scale_resistance * scale_capacitance),
            255,
        );
        data[14] = 21_u8.saturating_mul((0.1 * 640.0 / fs_adc).floor() as u8);
        data[15] = clipped_u8(1.025 * f64::from(data[7]), 127);
        data[16] = clipped_u8(
            f64::from(data[15]) * (0.98 + 0.02 * ((640.0 / fs_adc) / max_snr).max(1.0)),
            127,
        );
        data[17] = data[15];
        data[18] = clipped_u8(0.975 * f64::from(data[10]), 127);
        data[19] = clipped_u8(
            f64::from(data[18]) * (0.98 + 0.02 * ((640.0 / fs_adc) / max_snr).max(1.0)),
            127,
        );
        data[20] = data[18];
        data[21] = clipped_u8(0.975 * f64::from(data[12]), 127);
        data[22] = clipped_u8(
            f64::from(data[21]) * (0.98 + 0.02 * ((640.0 / fs_adc) / max_snr).max(1.0)),
            127,
        );
        data[23] = data[21];
        data[24] = 0x2e;
        for offset in [25_usize, 28, 31] {
            data[offset] = clipped_u8(128.0 + (63.0 * fs_adc / 640.0).min(63.0), 255);
            data[offset + 1] = clipped_u8(
                (63.0 * fs_adc / 640.0 * (0.92 + 0.08 * (640.0 / fs_adc))).min(63.0),
                63,
            );
        }
        data[27] = clipped_u8((32.0 * (fs_adc / 640.0).sqrt()).min(63.0), 63);
        data[30] = data[27];
        data[33] = clipped_u8((63.0 * (fs_adc / 640.0).sqrt()).min(63.0), 63);
        data[34] = clipped_u8(64.0 * (fs_adc / 640.0).sqrt(), 127);
        data[35] = 0x40;
        data[36] = 0x40;
        data[37] = 0x2c;
        for (offset, value) in data.into_iter().enumerate() {
            io.write(0x200 + offset as u16, value).await?;
        }
        Ok(())
    }

    async fn calibrate_baseband_dc_offset(&self, io: &mut impl RegisterIo) -> Result<()> {
        for &(register, value) in &[
            (0x18b, 0x83),
            (0x193, 0x3f),
            (0x190, 0x0f),
            (0x194, 0x01),
            (0x016, 0x01),
        ] {
            io.write(register, value).await?;
        }
        wait_for_mask(
            io,
            0x016,
            0x01,
            false,
            101,
            Duration::from_millis(5),
            "baseband DC-offset calibration",
        )
        .await
    }

    async fn calibrate_rf_dc_offset(&self, io: &mut impl RegisterIo) -> Result<()> {
        let settings = if self.rx_frequency_hz < 4e9 {
            [(0x186, 0x32), (0x187, 0x24), (0x188, 0x05)]
        } else {
            [(0x186, 0x28), (0x187, 0x34), (0x188, 0x06)]
        };
        for &(register, value) in &settings {
            io.write(register, value).await?;
        }
        for &(register, value) in &[(0x185, 0x20), (0x18b, 0x83), (0x189, 0x30), (0x016, 0x02)] {
            io.write(register, value).await?;
        }
        wait_for_mask(
            io,
            0x016,
            0x02,
            false,
            201,
            Duration::from_millis(50),
            "RF DC-offset calibration",
        )
        .await?;
        io.write(0x18b, 0x8d).await
    }

    async fn calibrate_rx_quadrature(&mut self, io: &mut impl RegisterIo) -> Result<()> {
        for &(register, value) in &[
            (0x168, 0x03),
            (0x16e, 0x25),
            (0x16a, 0x75),
            (0x16b, 0x95),
            (0x057, 0x33),
            (0x169, 0xc0),
        ] {
            io.write(register, value).await?;
        }
        let original_tx = self.tx_frequency_hz;
        self.tune_helper(
            io,
            Direction::Tx,
            self.rx_frequency_hz + self.rx_bb_lp_bandwidth_hz / 2.0,
        )
        .await?;
        io.write(0x016, 0x20).await?;
        let result = wait_for_mask(
            io,
            0x016,
            0x20,
            false,
            1_001,
            Duration::from_millis(5),
            "RX quadrature calibration",
        )
        .await;
        io.write(0x057, 0x30).await?;
        self.tune_helper(io, Direction::Tx, original_tx).await?;
        result
    }

    async fn calibrate_tx_quadrature(&mut self, io: &mut impl RegisterIo) -> Result<()> {
        let state = io.read(0x017).await? & 0x0f;
        if state != 0x05 {
            return Err(Error::Ad9361Initialization {
                stage: "TX quadrature calibration",
                detail: format!("expected ALERT state 0x5, got 0x{state:x}"),
            });
        }
        io.write(0x169, 0xc0).await?;
        let original_input = self.registers.input_selection;
        for side_b in [false, true] {
            self.registers.input_selection = if side_b {
                original_input | 0x40
            } else {
                original_input & !0x40
            };
            io.write(0x004, self.registers.input_selection).await?;
            self.tx_quadrature_routine(io).await?;
        }
        self.registers.input_selection = original_input;
        io.write(0x004, original_input).await
    }

    async fn tx_quadrature_routine(&self, io: &mut impl RegisterIo) -> Result<()> {
        let mut register_a3 = io.read(0x0a3).await?;
        let nco_frequency = register_a3 & 0xc0;
        io.write(0x0a0, 0x15 | (nco_frequency >> 1)).await?;
        register_a3 = io.read(0x0a3).await?;
        io.write(0x0a3, (register_a3 & 0x3f) | nco_frequency)
            .await?;
        let max_calibration_frequency = self.baseband_bandwidth_hz
            * f64::from(self.tx_fir_factor)
            * (f64::from(nco_frequency >> 6) + 1.0)
            / 16.0;
        let one_sided_bandwidth = (self.baseband_bandwidth_hz / 2.0).clamp(0.20e6, 28e6);
        if max_calibration_frequency > one_sided_bandwidth {
            return Err(Error::Ad9361Initialization {
                stage: "TX quadrature calibration",
                detail: "calibration tone is outside the baseband filter".into(),
            });
        }
        for &(register, value) in &[
            (0x0a1, 0x7b),
            (0x0a9, 0xff),
            (0x0a2, 0x7f),
            (0x0a5, 0x01),
            (0x0a6, 0x01),
            (
                0x0aa,
                if self.rx_frequency_hz < 1.3e9 {
                    0x22
                } else {
                    0x25
                },
            ),
            (0x0a4, 0xf0),
            (0x0ae, 0x00),
            (0x016, 0x10),
        ] {
            io.write(register, value).await?;
        }
        wait_for_mask(
            io,
            0x016,
            0x10,
            false,
            101,
            Duration::from_millis(10),
            "TX quadrature calibration",
        )
        .await
    }

    async fn set_clock_rate(&mut self, io: &mut impl RegisterIo, rate: f64) -> Result<f64> {
        if nearly_equal(rate, self.requested_clock_hz) {
            return Ok(self.baseband_bandwidth_hz);
        }
        let initial_state = io.read(0x017).await? & 0x0f;
        match initial_state {
            0x05 => {
                io.write(0x014, 0x21).await?;
                io.delay(Duration::from_millis(5)).await;
                io.write(0x014, 0x00).await?;
            }
            0x0a => io.write(0x014, 0x00).await?,
            state => {
                return Err(Error::Ad9361Initialization {
                    stage: "master-clock change",
                    detail: format!("expected ALERT/FDD state, got 0x{state:x}"),
                });
            }
        }
        wait_for_state(io, 0x00, "transition to SLEEP").await?;
        let original_tx_chains = self.registers.tx_filter & 0xc0;
        let original_rx_chains = self.registers.rx_filter & 0xc0;
        self.setup_rates(io, rate).await?;

        io.write(0x015, 0x04).await?;
        io.write(0x014, 0x05).await?;
        io.write(0x013, 0x01).await?;
        io.delay(Duration::from_millis(1)).await;
        wait_for_state(io, 0x05, "enter ALERT for clock calibration").await?;
        self.calibrate_synth_charge_pumps(io).await?;
        self.tune_helper(io, Direction::Rx, self.rx_frequency_hz)
            .await?;
        self.tune_helper(io, Direction::Tx, self.tx_frequency_hz)
            .await?;
        self.program_mixer_gm_subtable(io).await?;
        self.current_gain_table = 0;
        self.program_gain_table(io).await?;
        self.setup_gain_control(io, false).await?;
        self.reprogram_gains(io).await?;
        let baseband_bandwidth = self.baseband_bandwidth_hz;
        self.set_bandwidth(io, Direction::Rx, baseband_bandwidth)
            .await?;
        self.set_bandwidth(io, Direction::Tx, baseband_bandwidth)
            .await?;
        self.setup_adc(io).await?;
        self.calibrate_baseband_dc_offset(io).await?;
        self.calibrate_rf_dc_offset(io).await?;
        self.calibrate_rx_quadrature(io).await?;
        self.configure_tracking(io).await?;
        self.last_rx_calibration_hz = self.rx_frequency_hz;
        self.last_tx_calibration_hz = self.tx_frequency_hz;
        io.write(0x012, 0x02).await?;
        io.write(0x013, 0x01).await?;
        io.write(0x015, 0x04).await?;

        if initial_state == 0x0a {
            self.registers.tx_filter = (self.registers.tx_filter & 0x3f) | original_tx_chains;
            self.registers.rx_filter = (self.registers.rx_filter & 0x3f) | original_rx_chains;
            io.write(0x002, self.registers.tx_filter).await?;
            io.write(0x003, self.registers.rx_filter).await?;
            io.write(0x014, 0x21).await?;
            wait_for_state(io, 0x0a, "return to FDD after clock calibration").await?;
        }
        Ok(self.baseband_bandwidth_hz)
    }

    async fn tune(
        &mut self,
        io: &mut impl RegisterIo,
        direction: Direction,
        frequency_hz: f64,
    ) -> Result<f64> {
        let (requested, current, last_calibration) = match direction {
            Direction::Rx => (
                self.requested_rx_frequency_hz,
                self.rx_frequency_hz,
                self.last_rx_calibration_hz,
            ),
            Direction::Tx => (
                self.requested_tx_frequency_hz,
                self.tx_frequency_hz,
                self.last_tx_calibration_hz,
            ),
        };
        if nearly_equal(frequency_hz, requested) {
            return Ok(current);
        }
        let initial_state = io.read(0x017).await? & 0x0f;
        let return_to_fdd = match initial_state {
            0x05 => false,
            0x0a => {
                io.write(0x014, 0x01).await?;
                wait_for_state(io, 0x05, "enter ALERT for RF tune").await?;
                true
            }
            state => return Err(Error::Ad9361NotInitialized { state }),
        };
        let actual = self.tune_helper(io, direction, frequency_hz).await?;
        if direction == Direction::Rx {
            self.program_gain_table(io).await?;
        }
        self.reprogram_gains(io).await?;
        if (last_calibration - actual).abs() > CALIBRATION_WINDOW_HZ {
            match direction {
                Direction::Rx => {
                    self.calibrate_rf_dc_offset(io).await?;
                    if !self.iq_balance_tracking {
                        self.calibrate_rx_quadrature(io).await?;
                    }
                    self.last_rx_calibration_hz = actual;
                }
                Direction::Tx => {
                    self.calibrate_tx_quadrature(io).await?;
                    self.last_tx_calibration_hz = actual;
                }
            }
            self.configure_tracking(io).await?;
        }
        if return_to_fdd {
            io.write(0x014, 0x21).await?;
            wait_for_state(io, 0x0a, "return to FDD after RF tune").await?;
        }
        Ok(actual)
    }
}

fn nearly_equal(left: f64, right: f64) -> bool {
    (left - right).abs() < 1.0
}

fn clipped_u8(value: f64, maximum: u8) -> u8 {
    value.floor().clamp(0.0, f64::from(maximum)) as u8
}

fn supported_tap_count(maximum: usize) -> usize {
    [16, 32, 48, 64, 80, 96, 112, 128]
        .into_iter()
        .rev()
        .find(|count| *count <= maximum)
        .unwrap_or(16)
}

fn fir_coefficients(factor: u8, tap_count: usize) -> Result<&'static [i16]> {
    match (factor, tap_count) {
        (4, 48) => Ok(&FIR_48_X4),
        (4, 64) => Ok(&FIR_64_X4),
        (4, 96) => Ok(&FIR_96_X4),
        (4, 128) => Ok(&FIR_128_X4),
        (1 | 2, 48) => Ok(&HB47),
        (1 | 2, 64) => Ok(&HB63),
        (1 | 2, 96) => Ok(&HB95),
        (1 | 2, 128) => Ok(&HB127),
        _ => Err(Error::Ad9361Initialization {
            stage: "FIR configuration",
            detail: format!("unsupported {tap_count}-tap FIR with factor {factor}"),
        }),
    }
}

fn synth_settings(vco_rate: f64) -> &'static [u8; 12] {
    let index = VCO_INDEX_HZ
        .iter()
        .position(|boundary| vco_rate > *boundary)
        .unwrap_or(VCO_INDEX_HZ.len() - 1);
    &SYNTH_CAL_LUT[index]
}

fn rf_synth_settings(frequency_hz: f64) -> Result<(f64, u8, u32, u32, f64)> {
    const REFERENCE_HZ: f64 = 80_000_000.0;
    const MODULUS: f64 = 8_388_593.0;
    for divider_index in 0_u8..=6 {
        let divider = f64::from(2_u32 << divider_index);
        let vco_rate = frequency_hz * divider;
        if (6e9..=12e9).contains(&vco_rate) {
            let n_integer = (vco_rate / REFERENCE_HZ).floor() as u32;
            let n_fractional =
                (((vco_rate / REFERENCE_HZ) - f64::from(n_integer)) * MODULUS) as u32;
            let actual_vco =
                REFERENCE_HZ * (f64::from(n_integer) + f64::from(n_fractional) / MODULUS);
            return Ok((
                actual_vco / divider,
                divider_index,
                n_integer,
                n_fractional,
                actual_vco,
            ));
        }
    }
    Err(Error::InvalidArgument(
        "center frequency cannot be represented by the AD9361 RF PLL".into(),
    ))
}

fn gain_table(frequency_hz: f64) -> Result<(u8, &'static [[u8; 3]; 77])> {
    if frequency_hz < 1.3e9 {
        Ok((1, &GAIN_TABLE_SUB_1300))
    } else if frequency_hz < 4e9 {
        Ok((2, &GAIN_TABLE_1300_TO_4000))
    } else if frequency_hz <= 6e9 {
        Ok((3, &GAIN_TABLE_4000_TO_6000))
    } else {
        Err(Error::InvalidArgument(
            "AD9361 receive frequency exceeds 6 GHz".into(),
        ))
    }
}

async fn wait_for_mask(
    io: &mut impl RegisterIo,
    register: u16,
    mask: u8,
    expected_set: bool,
    attempts: usize,
    interval: Duration,
    stage: &'static str,
) -> Result<()> {
    for attempt in 0..attempts {
        let is_set = io.read(register).await? & mask != 0;
        if is_set == expected_set {
            return Ok(());
        }
        if attempt + 1 < attempts {
            io.delay(interval).await;
        }
    }
    Err(calibration_error(stage))
}

async fn wait_for_state(io: &mut impl RegisterIo, expected: u8, stage: &'static str) -> Result<()> {
    for attempt in 0..100 {
        if io.read(0x017).await? & 0x0f == expected {
            return Ok(());
        }
        if attempt < 99 {
            io.delay(Duration::from_millis(1)).await;
        }
    }
    Err(calibration_error(stage))
}

async fn wait_for_state_not(
    io: &mut impl RegisterIo,
    rejected: &[u8],
    stage: &'static str,
) -> Result<()> {
    for attempt in 0..100 {
        let state = io.read(0x017).await? & 0x0f;
        if !rejected.contains(&state) {
            return Ok(());
        }
        if attempt < 99 {
            io.delay(Duration::from_millis(1)).await;
        }
    }
    Err(calibration_error(stage))
}

async fn codec_loopback_self_test(control: &mut RadioControl) -> Result<()> {
    control.set_stream(StreamId::LocalControl);
    {
        let mut io = Ad9361Io::new(B2xxSpi::new(control));
        io.write_register(0x3f5, 0x01).await?;
    }
    Delay::new(Duration::from_millis(1)).await;

    control.set_stream(StreamId::RadioControl(0));
    let test_result = async {
        let mut pattern = 0x6d2b_79f5_u32;
        for _ in 0..100 {
            pattern ^= pattern << 13;
            pattern ^= pattern >> 17;
            pattern ^= pattern << 5;
            let expected = pattern & 0xfff0_fff0;
            control.poke32(SR_CODEC_IDLE, expected).await?;
            let actual = control.peek64(RB64_CODEC_READBACK).await?;
            let tx = (actual >> 32) as u32;
            let rx = actual as u32;
            if tx != expected || rx != expected {
                return Err(Error::Ad9361Loopback { expected, tx, rx });
            }
        }
        Ok(())
    }
    .await;

    let clear_result = control.poke32(SR_CODEC_IDLE, 0).await;
    control.set_stream(StreamId::LocalControl);
    let disable_result = async {
        let mut io = Ad9361Io::new(B2xxSpi::new(control));
        io.write_register(0x3f5, 0x00).await
    }
    .await;

    test_result?;
    clear_result?;
    disable_result
}

fn calibration_error(stage: &'static str) -> Error {
    Error::Ad9361CalibrationTimeout { stage }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;

    #[derive(Default)]
    struct SimulatedAd9364 {
        registers: BTreeMap<u16, u8>,
        writes: Vec<(u16, u8)>,
    }

    impl SimulatedAd9364 {
        fn ready() -> Self {
            let mut registers = BTreeMap::new();
            registers.insert(0x037, 0x08);
            registers.insert(0x05e, 0x80);
            registers.insert(0x244, 0x80);
            registers.insert(0x284, 0x80);
            registers.insert(0x247, 0x02);
            registers.insert(0x287, 0x02);
            registers.insert(0x1eb, 0x20);
            registers.insert(0x1ec, 0x20);
            registers.insert(0x1e6, 0x03);
            Self {
                registers,
                writes: Vec::new(),
            }
        }
    }

    impl RegisterIo for SimulatedAd9364 {
        async fn read(&mut self, register: u16) -> Result<u8> {
            Ok(self.registers.get(&register).copied().unwrap_or(0))
        }

        async fn write(&mut self, register: u16, value: u8) -> Result<()> {
            self.writes.push((register, value));
            match register {
                // Calibration command bits clear when the simulated operation completes.
                0x016 => {
                    self.registers.insert(register, 0);
                }
                // Model the ENSM transitions used by the B200 sequence.
                0x014 => {
                    let state = match value {
                        0x00 => 0x00,
                        0x01 | 0x05 => 0x05,
                        0x21 => 0x0a,
                        _ => *self.registers.get(&0x017).unwrap_or(&0),
                    };
                    self.registers.insert(0x017, state);
                    self.registers.insert(register, value);
                }
                _ => {
                    self.registers.insert(register, value);
                }
            }
            Ok(())
        }

        async fn delay(&mut self, _duration: Duration) {}
    }

    #[test]
    fn rf_synth_reproduces_exact_100_mhz_settings() {
        let (actual, divider, integer, fractional, _) = rf_synth_settings(100e6).unwrap();
        assert_eq!(actual, 100e6);
        assert_eq!(divider, 5);
        assert_eq!(integer, 80);
        assert_eq!(fractional, 0);
    }

    #[test]
    fn selects_supported_fir_sizes() {
        assert_eq!(supported_tap_count(127), 112);
        assert_eq!(supported_tap_count(96), 96);
        assert_eq!(fir_coefficients(2, 96).unwrap(), HB95);
        assert_eq!(fir_coefficients(4, 64).unwrap(), FIR_64_X4);
        assert!(fir_coefficients(2, 112).is_err());
    }

    #[test]
    fn full_register_sequence_reaches_fdd_at_16_mhz() {
        futures_lite::future::block_on(async {
            let mut io = SimulatedAd9364::ready();
            let mut controller = Ad9361Controller::default();
            controller.initialize(&mut io).await.unwrap();
            controller
                .set_clock_rate(&mut io, MASTER_CLOCK_HZ)
                .await
                .unwrap();
            controller
                .tune(&mut io, Direction::Rx, DEFAULT_TUNE_FREQUENCY_HZ)
                .await
                .unwrap();
            controller
                .set_active_chains(&mut io, false, false, true, false)
                .await
                .unwrap();

            assert_eq!(io.registers.get(&0x017), Some(&0x0a));
            assert_eq!(io.registers.get(&0x003).copied().unwrap_or(0) & 0xc0, 0x40);
            assert_eq!(io.registers.get(&0x002).copied().unwrap_or(0) & 0xc0, 0x00);
            assert_eq!(controller.baseband_bandwidth_hz, MASTER_CLOCK_HZ);
            assert!(io.writes.starts_with(&[(0x000, 0x01), (0x000, 0x00)]));
            assert!(io.writes.contains(&(0x006, 0x0f)));
            assert!(io.writes.contains(&(0x007, 0x0f)));
        });
    }

    #[test]
    fn stuck_calibration_reports_its_stage() {
        futures_lite::future::block_on(async {
            let mut io = SimulatedAd9364::default();
            let error = wait_for_mask(
                &mut io,
                0x016,
                0x01,
                true,
                2,
                Duration::from_millis(1),
                "test calibration",
            )
            .await
            .unwrap_err();
            assert!(matches!(
                error,
                Error::Ad9361CalibrationTimeout {
                    stage: "test calibration"
                }
            ));
        });
    }
}
