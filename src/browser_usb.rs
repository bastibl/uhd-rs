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
        let usb = web_sys::window()
            .ok_or(Error::Unsupported("WebUSB requires a browser window"))?
            .navigator()
            .usb();
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
")]
    extern "C" {
        fn install_usb() -> web_sys::UsbDevice;
        fn empty_usb();
        fn pending_transfer(d: &web_sys::UsbDevice) -> js_sys::Promise;
        fn closed_count(d: &web_sys::UsbDevice) -> u32;
    }
    #[wasm_bindgen_test]
    async fn terminal_browser_close_aborts_pending_operations_and_is_idempotent() {
        let raw = install_usb();
        let info = crate::Device::request_permission().await.unwrap().unwrap();
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
        let info = crate::Device::request_permission().await.unwrap().unwrap();
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
}
