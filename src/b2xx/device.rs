use nusb::DeviceSelector;

use super::{Product, SUPPORTED_IDS};
use crate::Result;

/// USB descriptor information for a possible B2xx device.
#[derive(Clone, Debug)]
pub struct B2xxDeviceInfo {
    inner: nusb::DeviceInfo,
    pub vendor_id: u16,
    pub product_id: u16,
    pub manufacturer: Option<String>,
    pub product_string: Option<String>,
    pub serial_number: Option<String>,
    pub product: Option<Product>,
    /// Whether the descriptor indicates that Ettus/NI FX3 firmware is running.
    pub firmware_loaded: bool,
}

impl B2xxDeviceInfo {
    pub(crate) fn new(inner: nusb::DeviceInfo) -> Self {
        let vendor_id = inner.vendor_id();
        let product_id = inner.product_id();
        let manufacturer = inner.manufacturer_string().map(str::to_owned);
        let firmware_loaded = matches!(
            manufacturer.as_deref(),
            Some("Ettus Research LLC" | "National Instruments Corp." | "Free Software Folks")
        );
        Self {
            product: Product::from_usb_id(vendor_id, product_id),
            vendor_id,
            product_id,
            manufacturer,
            product_string: inner.product_string().map(str::to_owned),
            serial_number: inner.serial_number().map(str::to_owned),
            firmware_loaded,
            inner,
        }
    }

    /// Open the device and claim interface zero for FX3 control requests.
    pub async fn open(self) -> Result<super::B2xxDevice> {
        super::B2xxDevice::open(self).await
    }

    pub(crate) fn nusb_info(&self) -> &nusb::DeviceInfo {
        &self.inner
    }
}

/// List B2xx devices for which the application already has OS/browser access.
pub async fn list_devices() -> Result<Vec<B2xxDeviceInfo>> {
    Ok(nusb::list_devices()
        .await?
        .filter(|device| SUPPORTED_IDS.contains(&(device.vendor_id(), device.product_id())))
        .map(B2xxDeviceInfo::new)
        .collect())
}

/// Request access to a B2xx device.
///
/// In a browser, call this synchronously from a user-activation event handler;
/// it opens the `WebUSB` chooser. On native platforms it returns the first
/// matching device without displaying a prompt.
pub async fn request_device() -> Result<Option<B2xxDeviceInfo>> {
    let selectors: Vec<_> = SUPPORTED_IDS
        .iter()
        .map(|&(vendor, product)| DeviceSelector::all().with_vid_pid(vendor, product))
        .collect();
    Ok(nusb::request_device(&selectors)
        .await?
        .map(B2xxDeviceInfo::new))
}
