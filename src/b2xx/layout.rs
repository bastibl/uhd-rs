//! Board routing from UHD 4.8 b200_impl.cpp's frontend mapping and band selection.
//! <https://github.com/EttusResearch/uhd/blob/v4.8.0.0/host/lib/usrp/b200/b200_impl.cpp>

use super::Product;

/// Mapping for logical RX channel zero. This does not enable MIMO or TX.
#[derive(Clone, Copy, Debug)]
pub(crate) struct RadioLayout {
    product: Product,
    pub rx_chain_two: bool,
}

impl RadioLayout {
    pub const fn new(product: Product, revision: u16) -> Self {
        Self {
            product,
            rx_chain_two: matches!(product, Product::B210)
                || (matches!(product, Product::B200) && revision < 5),
        }
    }

    pub const fn radio_chains(self) -> u8 {
        if matches!(self.product, Product::B210) {
            2
        } else {
            1
        }
    }

    const fn mini(self) -> bool {
        matches!(self.product, Product::B200Mini | Product::B205Mini)
    }

    pub fn rx_port(self, frequency_hz: f64) -> u8 {
        if self.mini() {
            0x03
        } else if frequency_hz < 2.2e9 {
            0x30
        } else if frequency_hz < 4e9 {
            0x0c
        } else {
            0x03
        }
    }

    pub fn tx_port_b(self, frequency_hz: f64) -> bool {
        !self.mini() && frequency_hz < 2.5e9
    }

    pub const fn gain_register(self) -> u16 {
        if self.rx_chain_two { 0x10c } else { 0x109 }
    }

    pub const fn agc_shift(self) -> u8 {
        if self.rx_chain_two { 2 } else { 0 }
    }

    pub fn misc_word(self, frequency_hz: f64) -> u32 {
        let swap_atr = u32::from(self.rx_chain_two) << 8;
        if self.mini() {
            return swap_atr;
        }
        let rx_band = if frequency_hz < 2.2e9 {
            1 << 3
        } else if frequency_hz < 4e9 {
            1 << 4
        } else {
            1 << 5
        };
        // TX remains disabled; retain its initialized low-band switch setting.
        swap_atr | rx_band | (1 << 6)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fpga_compatibility_matches_b200_and_mini_families() {
        assert_eq!(Product::B200.fpga_compatibility(), 16);
        assert_eq!(Product::B210.fpga_compatibility(), 16);
        assert_eq!(Product::B200Mini.fpga_compatibility(), 7);
        assert_eq!(Product::B205Mini.fpga_compatibility(), 7);
    }

    #[test]
    fn channel_zero_follows_board_wiring() {
        for (product, revision, swapped, chains) in [
            (Product::B200, 4, true, 1),
            (Product::B200, 5, false, 1),
            (Product::B210, 4, true, 2),
            (Product::B200Mini, 1, false, 1),
            (Product::B205Mini, 1, false, 1),
        ] {
            let layout = RadioLayout::new(product, revision);
            assert_eq!(layout.rx_chain_two, swapped);
            assert_eq!(layout.radio_chains(), chains);
            assert_eq!(layout.misc_word(100e6) & (1 << 8) != 0, swapped);
            assert_eq!(
                layout.misc_word(100e6) & 0x07,
                0,
                "no TX/MIMO/reset/ref override"
            );
            assert_eq!(layout.gain_register(), if swapped { 0x10c } else { 0x109 });
            assert_eq!(layout.agc_shift(), if swapped { 2 } else { 0 });
        }
    }

    #[test]
    fn mini_uses_port_a_without_external_band_switches() {
        for product in [Product::B200Mini, Product::B205Mini] {
            let layout = RadioLayout::new(product, 1);
            for hz in [70e6, 100e6, 2.2e9, 4e9, 6e9] {
                assert_eq!(layout.rx_port(hz), 0x03);
                assert!(!layout.tx_port_b(hz));
                assert_eq!(layout.misc_word(hz), 0);
            }
        }
        let layout = RadioLayout::new(Product::B210, 4);
        assert_eq!(layout.rx_port(2.2e9 - 1.0), 0x30);
        assert_eq!(layout.rx_port(2.2e9), 0x0c);
        assert_eq!(layout.rx_port(4e9), 0x03);
        assert_eq!(layout.misc_word(3e9), (1 << 8) | (1 << 6) | (1 << 4));
        assert!(layout.tx_port_b(2.5e9 - 1.0));
        assert!(!layout.tx_port_b(2.5e9));
    }
}
