use nusb::DeviceSelector;

use super::{Product, SUPPORTED_IDS};
use crate::Result;

/// USB descriptor information for a possible B2xx device.
#[derive(Clone, Debug)]
pub struct B2xxDeviceInfo {
    #[cfg(not(target_arch = "wasm32"))]
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
            #[cfg(not(target_arch = "wasm32"))]
            inner,
        }
    }

    /// Open the device and claim interface zero for FX3 control requests.
    pub fn open(self) -> impl nusb::MaybeFuture<Output = Result<super::B2xxDevice>> {
        crate::operation::operation(super::B2xxDevice::open(self))
    }

    /// Platform-specific USB bus identifier. Together with [`Self::port_chain`],
    /// this identifies the physical connector across FX3 re-enumeration.
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn bus_id(&self) -> &str {
        self.inner.bus_id()
    }

    /// USB hub port path used to match a device before and after firmware load.
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn port_chain(&self) -> &[u8] {
        self.inner.port_chain()
    }

    /// Stable Linux connector identity, including USB 2/3 companion ports.
    #[cfg(target_os = "linux")]
    pub(crate) fn physical_port_key(&self) -> Option<std::path::PathBuf> {
        physical_port_key(self.inner.sysfs_path(), *self.port_chain().last()?)
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn nusb_info(&self) -> &nusb::DeviceInfo {
        &self.inner
    }
}

/// List B2xx devices for which the application already has OS/browser access.
pub async fn list_devices() -> Result<Vec<B2xxDeviceInfo>> {
    Ok(crate::operation::usb(nusb::list_devices())
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
    Ok(crate::operation::usb(nusb::request_device(&selectors))
        .await?
        .map(B2xxDeviceInfo::new))
}

#[cfg(target_os = "linux")]
fn physical_port_key(device: &std::path::Path, port: u8) -> Option<std::path::PathBuf> {
    let device = device.canonicalize().ok()?;
    let hub = device.parent()?;
    let hub_name = hub.file_name()?.to_str()?;
    let interface = if let Some(bus) = hub_name.strip_prefix("usb") {
        format!("{bus}-0:1.0")
    } else {
        format!("{hub_name}:1.0")
    };
    let connector = hub
        .join(interface)
        .join(format!("{hub_name}-port{port}"))
        .canonicalize()
        .ok()?;
    // The kernel explicitly identifies companions; never infer them from bus numbers.
    let peer = connector.join("peer").canonicalize().ok();
    Some(peer.map_or(connector.clone(), |peer| peer.min(connector)))
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    #[test]
    fn companion_ports_share_identity_but_unrelated_ports_do_not() {
        let root = std::env::temp_dir().join(format!("uhd-pure-ports-{}", std::process::id()));
        let a = root.join("usb1/1-0:1.0/usb1-port9");
        let b = root.join("usb2/2-0:1.0/usb2-port5");
        for path in [
            &a,
            &b,
            &root.join("usb2/2-0:1.0/usb2-port6"),
            &root.join("usb1/1-9"),
            &root.join("usb2/2-5"),
            &root.join("usb2/2-6"),
        ] {
            std::fs::create_dir_all(path).unwrap();
        }
        std::os::unix::fs::symlink(&b, a.join("peer")).unwrap();
        std::os::unix::fs::symlink(&a, b.join("peer")).unwrap();
        let boot = super::physical_port_key(&root.join("usb1/1-9"), 9).unwrap();
        assert_eq!(
            boot,
            super::physical_port_key(&root.join("usb2/2-5"), 5).unwrap()
        );
        assert_ne!(
            boot,
            super::physical_port_key(&root.join("usb2/2-6"), 6).unwrap()
        );
        std::fs::remove_dir_all(root).unwrap();
    }
}
