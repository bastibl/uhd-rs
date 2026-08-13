use wasm_bindgen::prelude::*;

use crate::b2xx::{B2xxDevice, RadioControl, StreamId, ad9361::Ad9361Controller};

/// JavaScript-facing B2xx handle backed by nusb's WebUSB implementation.
#[wasm_bindgen(js_name = B2xxDevice)]
pub struct WebB2xxDevice {
    inner: B2xxDevice,
    control: Option<RadioControl>,
    radio: Option<Ad9361Controller>,
}

#[wasm_bindgen(js_class = B2xxDevice)]
impl WebB2xxDevice {
    /// Display the browser WebUSB chooser. This must be invoked from a user gesture.
    pub async fn request() -> Result<Option<WebB2xxDevice>, JsValue> {
        let Some(info) = crate::b2xx::request_device().await.map_err(js_error)? else {
            return Ok(None);
        };
        let inner = info.open().await.map_err(js_error)?;
        Ok(Some(Self {
            inner,
            control: None,
            radio: None,
        }))
    }

    #[wasm_bindgen(getter, js_name = vendorId)]
    pub fn vendor_id(&self) -> u16 {
        self.inner.info().vendor_id
    }

    #[wasm_bindgen(getter, js_name = productId)]
    pub fn product_id(&self) -> u16 {
        self.inner.info().product_id
    }

    #[wasm_bindgen(getter, js_name = serialNumber)]
    pub fn serial_number(&self) -> Option<String> {
        self.inner.info().serial_number.clone()
    }

    #[wasm_bindgen(getter, js_name = productName)]
    pub fn product_name(&self) -> Option<String> {
        self.inner.info().product_string.clone()
    }

    #[wasm_bindgen(getter, js_name = firmwareLoaded)]
    pub fn firmware_loaded(&self) -> bool {
        self.inner.info().firmware_loaded
    }

    #[wasm_bindgen(js_name = usbVersion)]
    pub async fn usb_version(&self) -> Result<u8, JsValue> {
        self.inner
            .usb_speed()
            .await
            .map(|speed| speed.major_version())
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = fx3State)]
    pub async fn fx3_state(&self) -> Result<String, JsValue> {
        self.inner
            .fx3_state()
            .await
            .map(|state| state.to_string())
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = firmwareCompatibility)]
    pub async fn firmware_compatibility(&self) -> Result<String, JsValue> {
        self.inner
            .firmware_compatibility()
            .await
            .map(|compatibility| format!("{}.{}", compatibility.major, compatibility.minor))
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = motherboardIdentity)]
    pub async fn motherboard_identity(&self) -> Result<String, JsValue> {
        self.inner
            .identity()
            .await
            .map(|identity| {
                format!(
                    "product={},serial={},name={},revision={}",
                    identity.product.map_or("unknown", |product| product.name()),
                    identity.serial,
                    identity.name,
                    identity.revision
                )
            })
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = loadFirmware)]
    pub async fn load_firmware(&self, image: Vec<u8>) -> Result<(), JsValue> {
        self.inner.load_firmware(&image).await.map_err(js_error)
    }

    /// Reset running firmware back to the FX3 bootloader. The device handle is
    /// invalid after this returns and WebUSB permission must be requested again.
    #[wasm_bindgen(js_name = resetFx3)]
    pub async fn reset_fx3(&self) -> Result<(), JsValue> {
        self.inner.reset_fx3().await.map_err(js_error)
    }

    #[wasm_bindgen(js_name = loadFpga)]
    pub async fn load_fpga(&mut self, image: Vec<u8>, force: bool) -> Result<String, JsValue> {
        self.radio = None;
        self.control = None;
        let outcome = self
            .inner
            .load_fpga(&image, force)
            .await
            .map_err(js_error)?;
        self.inner.reset_gpif().await.map_err(js_error)?;
        Ok(format!("{outcome:?}"))
    }

    /// Claim all bulk interfaces and enable local register access.
    #[wasm_bindgen(js_name = openTransport)]
    pub async fn open_transport(&mut self) -> Result<(), JsValue> {
        if self.control.is_some() {
            return Ok(());
        }
        let transport = self.inner.open_transport().await.map_err(js_error)?;
        self.control = Some(transport.into_radio_control(StreamId::LocalControl));
        Ok(())
    }

    #[wasm_bindgen(getter, js_name = radioInitialized)]
    pub fn radio_initialized(&self) -> bool {
        self.radio.is_some()
    }

    /// Reset, configure, calibrate, and verify a revision-5-or-newer B200 radio.
    #[wasm_bindgen(js_name = initializeRadio)]
    pub async fn initialize_radio(&mut self) -> Result<(), JsValue> {
        self.inner
            .check_firmware_compatibility()
            .await
            .map_err(js_error)?;
        let identity = self.inner.identity().await.map_err(js_error)?;
        self.open_transport().await?;
        let radio = Ad9361Controller::initialize_b200(self.control_mut()?, &identity)
            .await
            .map_err(js_error)?;
        self.radio = Some(radio);
        Ok(())
    }

    pub async fn peek32(&mut self, byte_address: u32) -> Result<u32, JsValue> {
        self.control_mut()?
            .peek32(byte_address)
            .await
            .map_err(js_error)
    }

    pub async fn poke32(&mut self, byte_address: u32, value: u32) -> Result<(), JsValue> {
        self.control_mut()?
            .poke32(byte_address, value)
            .await
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = readAd9361Register)]
    pub async fn read_ad9361_register(&mut self, register: u16) -> Result<u8, JsValue> {
        let spi = crate::b2xx::B2xxSpi::new(self.control_mut()?);
        let mut codec = crate::b2xx::Ad9361Io::new(spi);
        codec.read_register(register).await.map_err(js_error)
    }

    #[wasm_bindgen(js_name = writeAd9361Register)]
    pub async fn write_ad9361_register(&mut self, register: u16, value: u8) -> Result<(), JsValue> {
        let spi = crate::b2xx::B2xxSpi::new(self.control_mut()?);
        let mut codec = crate::b2xx::Ad9361Io::new(spi);
        codec
            .write_register(register, value)
            .await
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = sendData)]
    pub async fn send_data(&mut self, bytes: Vec<u8>) -> Result<(), JsValue> {
        self.control_mut()?
            .transport_mut()
            .send_data(bytes)
            .await
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = receiveData)]
    pub async fn receive_data(&mut self, requested_length: usize) -> Result<Vec<u8>, JsValue> {
        self.control_mut()?
            .transport_mut()
            .receive_data(requested_length)
            .await
            .map_err(js_error)
    }
}

impl WebB2xxDevice {
    fn control_mut(&mut self) -> Result<&mut RadioControl, JsValue> {
        self.control
            .as_mut()
            .ok_or_else(|| JsValue::from_str("openTransport() must be called first"))
    }
}

fn js_error(error: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&error.to_string())
}
