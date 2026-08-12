use super::RadioControl;
use crate::{Error, Result};

const SPI_DIVIDER_ADDRESS: u32 = 32;
const SPI_CONTROL_ADDRESS: u32 = 36;
const SPI_DATA_ADDRESS: u32 = 40;
const SPI_READBACK_ADDRESS: u32 = 8;
const AD9361_SLAVE: u32 = 1;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpiEdge {
    Rising,
    Falling,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SpiConfig {
    pub mosi_edge: SpiEdge,
    pub miso_edge: SpiEdge,
    /// An optional raw FPGA divider. The resulting SPI frequency is
    /// `100 MHz / (2 * (divider + 1))`.
    pub divider: Option<u32>,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            mosi_edge: SpiEdge::Rising,
            miso_edge: SpiEdge::Falling,
            divider: None,
        }
    }
}

/// Async access to the FPGA's local `spi_core_3000` block.
pub struct B2xxSpi<'a> {
    control: &'a mut RadioControl,
    default_divider: u32,
    divider_cache: Option<u32>,
    control_cache: Option<u32>,
}

impl<'a> B2xxSpi<'a> {
    /// Wrap a local-control stream. The caller must select
    /// [`StreamId::LocalControl`](super::StreamId::LocalControl).
    pub const fn new(control: &'a mut RadioControl) -> Self {
        Self {
            control,
            // UHD configures the 100 MHz bus for a 1 MHz AD9361 SPI clock.
            default_divider: 49,
            divider_cache: None,
            control_cache: None,
        }
    }

    pub fn set_default_divider(&mut self, divider: u32) {
        self.default_divider = divider;
    }

    pub async fn transact(
        &mut self,
        slave_mask: u32,
        config: SpiConfig,
        data: u32,
        bits: u8,
        readback: bool,
    ) -> Result<u32> {
        if !(1..=32).contains(&bits) {
            return Err(Error::InvalidArgument(
                "SPI transaction width must be between 1 and 32 bits".into(),
            ));
        }
        if slave_mask > 0x00ff_ffff {
            return Err(Error::InvalidArgument(
                "SPI slave mask exceeds 24 bits".into(),
            ));
        }
        let divider = config.divider.unwrap_or(self.default_divider);
        if self.divider_cache != Some(divider) {
            self.control.poke32(SPI_DIVIDER_ADDRESS, divider).await?;
            self.divider_cache = Some(divider);
        }

        let control_word = control_word(slave_mask, config, bits);
        if self.control_cache != Some(control_word) {
            self.control
                .poke32(SPI_CONTROL_ADDRESS, control_word)
                .await?;
            self.control_cache = Some(control_word);
        }
        let data_out = if bits == 32 {
            data
        } else {
            data << (32 - u32::from(bits))
        };
        self.control.poke32(SPI_DATA_ADDRESS, data_out).await?;
        if readback {
            self.control.peek32(SPI_READBACK_ADDRESS).await
        } else {
            Ok(0)
        }
    }
}

/// Raw 8-bit AD9361 register access over the B2xx SPI core.
pub struct Ad9361Io<'a> {
    spi: B2xxSpi<'a>,
}

impl<'a> Ad9361Io<'a> {
    pub const fn new(spi: B2xxSpi<'a>) -> Self {
        Self { spi }
    }

    pub async fn read_register(&mut self, register: u16) -> Result<u8> {
        validate_ad9361_register(register)?;
        let command = u32::from(register) << 8;
        let value = self
            .spi
            .transact(AD9361_SLAVE, ad9361_spi_config(), command, 24, true)
            .await?;
        Ok(value.to_le_bytes()[0])
    }

    pub async fn write_register(&mut self, register: u16, value: u8) -> Result<()> {
        validate_ad9361_register(register)?;
        let command = 0x0080_0000 | (u32::from(register) << 8) | u32::from(value);
        self.spi
            .transact(AD9361_SLAVE, ad9361_spi_config(), command, 24, false)
            .await?;
        Ok(())
    }
}

fn validate_ad9361_register(register: u16) -> Result<()> {
    if register > 0x3fff {
        return Err(Error::InvalidArgument(
            "AD9361 register address exceeds 14 bits".into(),
        ));
    }
    Ok(())
}

fn ad9361_spi_config() -> SpiConfig {
    SpiConfig {
        // This matches UHD's FPGA SPI timing workaround.
        mosi_edge: SpiEdge::Falling,
        miso_edge: SpiEdge::Falling,
        divider: None,
    }
}

fn control_word(slave_mask: u32, config: SpiConfig, bits: u8) -> u32 {
    let mut word = (slave_mask & 0x00ff_ffff) | ((u32::from(bits) & 0x3f) << 24);
    if config.mosi_edge == SpiEdge::Falling {
        word |= 1 << 31;
    }
    if config.miso_edge == SpiEdge::Rising {
        word |= 1 << 30;
    }
    word
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spi_control_word_matches_fpga_layout() {
        let config = SpiConfig {
            mosi_edge: SpiEdge::Falling,
            miso_edge: SpiEdge::Rising,
            divider: None,
        };
        assert_eq!(control_word(1, config, 24), 0xd800_0001);
    }

    #[test]
    fn ad9361_command_layout() {
        let register = 0x123_u16;
        let value = 0x5a_u8;
        let write = 0x0080_0000 | (u32::from(register) << 8) | u32::from(value);
        assert_eq!(write, 0x0081_235a);
    }
}
