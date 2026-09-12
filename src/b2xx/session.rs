use super::{
    B2xxDevice, B2xxIdentity, Fx3State, LoadOutcome, Product, RadioControl, StreamId, UsbSpeed,
    ad9361::Ad9361Controller,
};
use crate::{Error, Result};

const FPGA_SIGNATURE: u32 = 0xace0_ba5e;
const CORE_STATUS_ADDRESS: u32 = 20;

/// Version reported by the B2xx FPGA compatibility register.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FpgaCompatibility {
    pub major: u16,
    pub minor: u16,
}

/// A B2xx with compatible firmware and FPGA and all bulk interfaces open.
///
/// This is the boundary between image/device management and radio setup.
pub struct B2xxSession {
    device: B2xxDevice,
    identity: B2xxIdentity,
    product: Product,
    usb_speed: UsbSpeed,
    fpga: FpgaCompatibility,
    radio_chains: u8,
    control: RadioControl,
    radio: Option<Ad9361Controller>,
}

impl std::fmt::Debug for B2xxSession {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("B2xxSession")
            .field("identity", &self.identity)
            .field("product", &self.product)
            .field("usb_speed", &self.usb_speed)
            .field("fpga", &self.fpga)
            .field("radio_chains", &self.radio_chains)
            .finish_non_exhaustive()
    }
}

impl B2xxSession {
    pub(crate) async fn open(device: B2xxDevice) -> Result<Self> {
        if device.fx3_state().await? != Fx3State::Running {
            return Err(Error::Unsupported(
                "the B2xx FPGA is not running; load its FPGA image before opening a session",
            ));
        }
        device.reset_gpif().await?;
        Self::open_after_gpif(device).await
    }

    pub(crate) async fn start(
        device: B2xxDevice,
        fpga_image: &[u8],
        force: bool,
    ) -> Result<(Self, LoadOutcome)> {
        device.check_firmware_compatibility().await?;
        let load_outcome = device.load_fpga(fpga_image, force).await?;
        device.reset_gpif().await?;
        let mut session = Self::open_after_gpif(device).await?;
        session.initialize_radio().await?;
        Ok((session, load_outcome))
    }

    async fn open_after_gpif(device: B2xxDevice) -> Result<Self> {
        device.check_firmware_compatibility().await?;
        let identity = device.identity().await?;
        let product =
            identity
                .product
                .or(device.info().product)
                .ok_or(Error::UnsupportedDevice {
                    vendor_id: device.info().vendor_id,
                    product_id: device.info().product_id,
                })?;
        let usb_speed = device.usb_speed().await?;
        let transport = device.open_transport().await?;
        let mut control = transport.into_radio_control(StreamId::LocalControl);
        let raw_compatibility = control.peek64(0).await?;
        let signature = upper_u32(raw_compatibility);
        if signature != FPGA_SIGNATURE {
            return Err(Error::FpgaSignature {
                expected: FPGA_SIGNATURE,
                actual: signature,
            });
        }
        let fpga = FpgaCompatibility {
            major: ((raw_compatibility >> 16) & 0xffff) as u16,
            minor: (raw_compatibility & 0xffff) as u16,
        };
        let expected = product.fpga_compatibility();
        if fpga.major != expected {
            return Err(Error::FpgaCompatibility {
                expected,
                actual: fpga.major,
            });
        }

        let radio_chains = ((control.peek32(CORE_STATUS_ADDRESS).await? >> 8) & 0xff) as u8;
        if !(1..=2).contains(&radio_chains) {
            return Err(Error::RadioChainCount(radio_chains));
        }
        Ok(Self {
            device,
            identity,
            product,
            usb_speed,
            fpga,
            radio_chains,
            control,
            radio: None,
        })
    }

    #[must_use]
    pub const fn device(&self) -> &B2xxDevice {
        &self.device
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
    pub const fn usb_speed(&self) -> UsbSpeed {
        self.usb_speed
    }

    #[must_use]
    pub const fn fpga_compatibility(&self) -> FpgaCompatibility {
        self.fpga
    }

    #[must_use]
    pub const fn radio_chains(&self) -> u8 {
        self.radio_chains
    }

    #[must_use]
    pub const fn radio_initialized(&self) -> bool {
        self.radio.is_some()
    }

    /// Reset, configure, calibrate, and verify the B2xx AD9361/AD9364 radio.
    pub async fn initialize_radio(&mut self) -> Result<()> {
        let radio = Ad9361Controller::initialize_b2xx(
            &mut self.control,
            self.product,
            self.identity.revision,
        )
        .await?;
        self.radio = Some(radio);
        Ok(())
    }

    #[must_use]
    pub const fn control_mut(&mut self) -> &mut RadioControl {
        &mut self.control
    }

    /// Borrow the local B2xx SPI core used to access the AD9361.
    pub fn spi(&mut self) -> super::B2xxSpi<'_> {
        self.control.set_stream(StreamId::LocalControl);
        super::B2xxSpi::new(&mut self.control)
    }

    /// Borrow a raw AD9361 register interface.
    pub fn ad9361(&mut self) -> super::Ad9361Io<'_> {
        super::Ad9361Io::new(self.spi())
    }

    pub(crate) fn into_radio_parts(
        self,
    ) -> Result<(
        RadioControl,
        B2xxIdentity,
        Product,
        Ad9361Controller,
        B2xxDevice,
    )> {
        let radio = self.radio.ok_or(Error::Unsupported(
            "the B2xx session radio has not been initialized",
        ))?;
        Ok((
            self.control,
            self.identity,
            self.product,
            radio,
            self.device,
        ))
    }
}

fn upper_u32(value: u64) -> u32 {
    let bytes = value.to_le_bytes();
    u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]])
}
