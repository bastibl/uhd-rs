//! Retain the browser handle so terminal cleanup can abort pending transfers.
use crate::{Error, Result, b2xx::B2xxDeviceInfo};
use wasm_bindgen::JsCast;
use wasm_bindgen_futures::JsFuture;
pub(crate) struct BrowserHandle {
    pub device: web_sys::UsbDevice,
    closed: std::cell::Cell<bool>,
}
impl BrowserHandle {
    pub async fn find(info: &B2xxDeviceInfo) -> Result<Self> {
        let global = js_sys::global();
        let usb = if let Some(window) = global.dyn_ref::<web_sys::Window>() {
            window.navigator().usb()
        } else if let Some(worker) = global.dyn_ref::<web_sys::WorkerGlobalScope>() {
            worker.navigator().usb()
        } else {
            return Err(Error::Unsupported(
                "WebUSB requires a browser window or worker",
            ));
        };
        if usb.is_undefined() {
            return Err(Error::Unsupported("WebUSB is not available"));
        }
        let devices = JsFuture::from(usb.get_devices()).await.map_err(js_error)?;
        let array = js_sys::Array::from(&devices);
        let mut matched = array
            .iter()
            .filter_map(|v| v.dyn_into::<web_sys::UsbDevice>().ok())
            .filter(|d| {
                d.vendor_id() == info.vendor_id
                    && d.product_id() == info.product_id
                    && d.serial_number() == info.serial_number
            });
        let device = matched.next().ok_or(Error::PermissionRequired)?;
        if matched.next().is_some() {
            return Err(Error::PermissionRequired);
        }
        Ok(Self {
            device,
            closed: std::cell::Cell::new(false),
        })
    }
    pub async fn close(&self) -> Result<()> {
        if !self.closed.get() {
            JsFuture::from(self.device.close())
                .await
                .map_err(js_error)?;
            self.closed.set(true);
        }
        Ok(())
    }
}
impl Drop for BrowserHandle {
    fn drop(&mut self) {
        if !self.closed.get() {
            let device = self.device.clone();
            wasm_bindgen_futures::spawn_local(async move {
                let _ = JsFuture::from(device.close()).await;
            });
        }
    }
}
fn js_error(value: wasm_bindgen::JsValue) -> Error {
    Error::Browser(format!("{value:?}"))
}
#[cfg(test)]
mod tests {
    use super::*;
    use wasm_bindgen::prelude::*;
    use wasm_bindgen_test::*;
    #[wasm_bindgen(inline_js = "
export function install_usb() {
 const d = {vendorId:0x2500,productId:0x20,serialNumber:'test-radio',manufacturerName:'Ettus Research LLC',productName:'B210',deviceVersionMajor:1,deviceVersionMinor:0,deviceVersionSubminor:0,usbVersionMajor:3,usbVersionMinor:0,usbVersionSubminor:0,deviceClass:0,deviceSubclass:0,deviceProtocol:0,configuration:null,opened:true,closed:0,pending:[],close() { this.closed++; this.opened=false; for(const resolve of this.pending) resolve('aborted'); this.pending=[]; return Promise.resolve(); }};
 Object.setPrototypeOf(d, USBDevice.prototype);
 Object.defineProperty(navigator,'usb',{configurable:true,value:{getDevices:()=>Promise.resolve([d]),requestDevice:()=>Promise.resolve(d)}});
 return d;
}
export function empty_usb() { Object.defineProperty(navigator,'usb',{configurable:true,value:{getDevices:()=>Promise.resolve([])}}); }
export function pending_transfer(d) { return new Promise(resolve=>d.pending.push(resolve)); }
export function closed_count(d) { return d.closed; }
export function install_fx3_usb(bootloader) {
 const d = install_usb();
 d.claims = 0;
 d.releases = 0;
 d.writes = 0;
 Object.defineProperties(d, {
  manufacturerName: {value: bootloader ? 'Cypress' : 'Ettus Research LLC', configurable: true},
  configurations: {value: []},
  open: {value: () => { d.opened = true; return Promise.resolve(); }},
  claimInterface: {value: () => { d.claims++; return Promise.reject(new Error('FX3 must use device control transfers')); }},
  releaseInterface: {value: () => { d.releases++; return Promise.reject(new DOMException('Unable to release interface.', 'NetworkError')); }},
  controlTransferIn: {value: (setup, length) => {
   let bytes;
   if (setup.requestType === 'standard' && setup.request === 6) {
    bytes = [18,1,0,3,0,0,0,9,0,0x25,0x20,0,0,0,0,0,0,0];
   } else if (setup.recipient === 'device' && setup.request === 0x15) {
    bytes = [8,0];
   } else { return Promise.reject(new Error('unexpected control read')); }
   return Promise.resolve({status:'ok', data:new DataView(new Uint8Array(bytes.slice(0,length)).buffer)});
  }},
  controlTransferOut: {value: (setup, data) => {
   if (setup.recipient !== 'device' || setup.request !== 0xa0) {
    return Promise.reject(new Error('unexpected firmware write'));
   }
   d.writes++;
   if (data.byteLength === 0) {
    d.opened = false;
    install_fx3_usb(false);
   }
   return Promise.resolve({status:'ok', bytesWritten:data.byteLength});
  }}
 });
 return d;
}
export function interface_calls(d) { return d.claims + d.releases; }
export function firmware_writes(d) { return d.writes; }
export function change_boot_serial(d) { d.serialNumber = 'bootloader-only'; }
")]
    extern "C" {
        fn install_usb() -> web_sys::UsbDevice;
        fn empty_usb();
        fn pending_transfer(d: &web_sys::UsbDevice) -> js_sys::Promise;
        fn closed_count(d: &web_sys::UsbDevice) -> u32;
        fn install_fx3_usb(bootloader: bool) -> web_sys::UsbDevice;
        fn interface_calls(d: &web_sys::UsbDevice) -> u32;
        fn firmware_writes(d: &web_sys::UsbDevice) -> u32;
        fn change_boot_serial(d: &web_sys::UsbDevice);
    }
    #[wasm_bindgen_test]
    async fn terminal_browser_close_aborts_pending_operations_and_is_idempotent() {
        let raw = install_usb();
        let info = crate::Device::list().await.unwrap().pop().unwrap();
        let handle = BrowserHandle::find(&info).await.unwrap();
        let pending = pending_transfer(&raw);
        handle.close().await.unwrap();
        assert_eq!(
            JsFuture::from(pending).await.unwrap().as_string().unwrap(),
            "aborted"
        );
        handle.close().await.unwrap();
        drop(handle);
        assert_eq!(closed_count(&raw), 1);
    }
    #[wasm_bindgen_test]
    async fn permission_loss_returns_typed_reselection_error() {
        install_usb();
        let info = crate::Device::list().await.unwrap().pop().unwrap();
        empty_usb();
        assert!(matches!(
            BrowserHandle::find(&info).await,
            Err(Error::PermissionRequired)
        ));
    }
    #[wasm_bindgen_test]
    async fn dropped_opening_handle_closes_in_background() {
        let raw = install_usb();
        let info = crate::Device::list().await.unwrap().pop().unwrap();
        drop(BrowserHandle::find(&info).await.unwrap());
        futures_timer::Delay::new(std::time::Duration::from_millis(20)).await;
        assert_eq!(closed_count(&raw), 1);
    }

    #[wasm_bindgen_test]
    async fn firmware_reconnect_uses_device_control_without_interface_cleanup() {
        let bootloader = install_fx3_usb(true);
        let info = crate::Device::list().await.unwrap().pop().unwrap();
        assert!(!info.firmware_loaded);
        let mut images = crate::images::ImageCatalog::default();
        images.insert(
            crate::images::Image::Firmware,
            b":020000040001F9\n:0400100001020304E2\n:0400000500010010E6\n:00000001FF\n".to_vec(),
        );
        let device = crate::b2xx::load_firmware_and_reconnect(
            info,
            &images,
            false,
            std::time::Duration::from_secs(1),
        )
        .await
        .unwrap();
        assert!(device.info().firmware_loaded);
        assert_eq!(device.info().serial_number.as_deref(), Some("test-radio"));
        assert_eq!(device.firmware_compatibility().await.unwrap().major, 8);
        drop(device);
        futures_timer::Delay::new(std::time::Duration::from_millis(20)).await;
        assert_eq!(firmware_writes(&bootloader), 2);
        assert_eq!(interface_calls(&bootloader), 0);
        assert_eq!(closed_count(&bootloader), 1);
    }

    #[wasm_bindgen_test]
    async fn changed_serial_requires_reselection_after_firmware_load() {
        let bootloader = install_fx3_usb(true);
        change_boot_serial(&bootloader);
        let info = crate::Device::list().await.unwrap().pop().unwrap();
        let mut images = crate::images::ImageCatalog::default();
        images.insert(
            crate::images::Image::Firmware,
            b":020000040001F9\n:0400100001020304E2\n:0400000500010010E6\n:00000001FF\n".to_vec(),
        );
        assert!(matches!(
            crate::b2xx::load_firmware_and_reconnect(
                info,
                &images,
                false,
                std::time::Duration::from_millis(50),
            )
            .await,
            Err(Error::PermissionRequired)
        ));
        assert_eq!(firmware_writes(&bootloader), 2);
        assert_eq!(interface_calls(&bootloader), 0);
    }
}
