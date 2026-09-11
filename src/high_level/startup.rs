//! Startup orchestration separated from USB I/O for cold/warm/cancellation tests.
use super::*;
use crate::operation::PortableSend;
use std::future::Future;
pub(super) trait Backend: SyncOnNative {
    type Descriptor: Clone + PortableSend;
    type Handle: PortableSend;
    type Key: PortableSend;
    type Radio: PortableSend;
    fn loaded(&self, info: &Self::Descriptor) -> bool;
    fn key(&self, info: &Self::Descriptor) -> Self::Key;
    fn matches(&self, key: &Self::Key, info: &Self::Descriptor, firmware: bool) -> bool;
    fn list(&self) -> impl Future<Output = Result<Vec<Self::Descriptor>>> + PortableSend;
    fn open(
        &self,
        info: Self::Descriptor,
    ) -> impl Future<Output = Result<Self::Handle>> + PortableSend;
    fn check_firmware(
        &self,
        device: &Self::Handle,
    ) -> impl Future<Output = Result<()>> + PortableSend;
    fn reset(&self, device: &Self::Handle) -> impl Future<Output = Result<()>> + PortableSend;
    fn firmware(
        &self,
        device: &Self::Handle,
        image: &[u8],
    ) -> impl Future<Output = Result<()>> + PortableSend;
    fn product(
        &self,
        device: &Self::Handle,
    ) -> impl Future<Output = Result<b2xx::Product>> + PortableSend;
    fn fpga(
        &self,
        device: &Self::Handle,
        image: &[u8],
    ) -> impl Future<Output = Result<()>> + PortableSend;
    fn running_hash(
        &self,
        device: &Self::Handle,
    ) -> impl Future<Output = Result<Option<u32>>> + PortableSend;
    fn initialize(
        &self,
        device: Self::Handle,
        config: RxConfig,
    ) -> impl Future<Output = Result<Self::Radio>> + PortableSend;
}
#[cfg(not(target_arch = "wasm32"))]
pub(super) use std::marker::Sync as SyncOnNative;
#[cfg(target_arch = "wasm32")]
pub(super) trait SyncOnNative {}
#[cfg(target_arch = "wasm32")]
impl<T> SyncOnNative for T {}

pub(super) async fn prepare<B: Backend>(
    backend: &B,
    info: B::Descriptor,
    images: &ImageCatalog,
    reload: bool,
    timeout: Duration,
) -> Result<B::Handle> {
    if backend.loaded(&info) && !reload {
        let device = backend.open(info).await?;
        backend.check_firmware(&device).await?;
        return Ok(device);
    }
    let image = images.get(Image::Firmware)?;
    crate::ihex::parse(&image)?;
    let key = backend.key(&info);
    let device = if backend.loaded(&info) {
        let device = backend.open(info).await?;
        backend.reset(&device).await?;
        drop(device);
        reconnect(backend, &key, false, timeout).await?
    } else {
        backend.open(info).await?
    };
    bounded(backend.firmware(&device, &image), Duration::from_secs(30)).await?;
    drop(device);
    let device = reconnect(backend, &key, true, timeout).await?;
    backend.check_firmware(&device).await?;
    Ok(device)
}
pub(super) async fn open<B: Backend>(
    backend: &B,
    info: B::Descriptor,
    images: &ImageCatalog,
    reload: bool,
    timeout: Duration,
    config: RxConfig,
) -> Result<B::Radio> {
    let device = prepare(backend, info, images, reload, timeout).await?;
    let image_id = Image::fpga(backend.product(&device).await?);
    match images.get(image_id) {
        Ok(image) => bounded(backend.fpga(&device, &image), Duration::from_secs(60)).await?,
        Err(Error::MissingImage(_))
            if backend.running_hash(&device).await? == Some(image_id.pinned_hash()) => {}
        Err(error) => return Err(error),
    }
    backend.initialize(device, config).await
}
async fn reconnect<B: Backend>(
    backend: &B,
    key: &B::Key,
    firmware: bool,
    timeout: Duration,
) -> Result<B::Handle> {
    bounded(
        async {
            loop {
                let mut candidates = backend
                    .list()
                    .await?
                    .into_iter()
                    .filter(|i| backend.matches(key, i, firmware));
                if let Some(info) = candidates.next()
                    && candidates.next().is_none()
                {
                    match backend.open(info).await {
                        Ok(device) => return Ok(device),
                        // A USB node may appear before udev finishes applying permissions.
                        Err(Error::Usb(error))
                            if matches!(
                                error.kind(),
                                nusb::ErrorKind::PermissionDenied
                                    | nusb::ErrorKind::NotFound
                                    | nusb::ErrorKind::Disconnected
                            ) => {}
                        Err(Error::Io(error))
                            if matches!(
                                error.kind(),
                                std::io::ErrorKind::PermissionDenied | std::io::ErrorKind::NotFound
                            ) => {}
                        Err(error) => return Err(error),
                    }
                }
                futures_timer::Delay::new(Duration::from_millis(50)).await;
            }
        },
        timeout,
    )
    .await
    .map_err(|error| {
        if !matches!(error, Error::Timeout) {
            return error;
        }
        #[cfg(target_arch = "wasm32")]
        {
            Error::PermissionRequired
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            Error::DeviceReenumerationTimeout {
                state: "reconnect on the same USB port",
                timeout,
            }
        }
    })
}
pub(super) struct Usb;
impl Backend for Usb {
    type Descriptor = DeviceDescriptor;
    type Handle = b2xx::B2xxDevice;
    type Key = ReconnectIdentity;
    type Radio = Radio;
    fn loaded(&self, info: &DeviceDescriptor) -> bool {
        info.firmware_loaded
    }
    fn key(&self, info: &DeviceDescriptor) -> ReconnectIdentity {
        ReconnectIdentity::new(info)
    }
    fn matches(&self, key: &ReconnectIdentity, info: &DeviceDescriptor, firmware: bool) -> bool {
        key.matches(info, firmware)
    }
    async fn list(&self) -> Result<Vec<DeviceDescriptor>> {
        b2xx::list_devices().await
    }
    async fn open(&self, info: DeviceDescriptor) -> Result<b2xx::B2xxDevice> {
        info.open().await
    }
    async fn check_firmware(&self, device: &b2xx::B2xxDevice) -> Result<()> {
        device.check_firmware_compatibility().await.map(|_| ())
    }
    async fn reset(&self, device: &b2xx::B2xxDevice) -> Result<()> {
        device.reset_fx3().await
    }
    async fn firmware(&self, device: &b2xx::B2xxDevice, image: &[u8]) -> Result<()> {
        device.load_firmware(image).await
    }
    async fn product(&self, device: &b2xx::B2xxDevice) -> Result<b2xx::Product> {
        device
            .identity()
            .await?
            .product
            .or(device.info().product)
            .ok_or(Error::Unsupported("unknown B2xx motherboard product"))
    }
    async fn fpga(&self, device: &b2xx::B2xxDevice, image: &[u8]) -> Result<()> {
        device.load_fpga(image, false).await.map(|_| ())
    }
    async fn running_hash(&self, device: &b2xx::B2xxDevice) -> Result<Option<u32>> {
        if device.fx3_state().await? == b2xx::Fx3State::Running {
            Ok(Some(device.fpga_hash().await?))
        } else {
            Ok(None)
        }
    }
    async fn initialize(&self, device: b2xx::B2xxDevice, config: RxConfig) -> Result<Radio> {
        Radio::open(device, config).await
    }
}
#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;
    use futures_lite::future::{block_on, poll_once};
    #[derive(Clone)]
    struct Info {
        loaded: bool,
        port: u8,
        serial: Option<&'static str>,
    }
    struct Handle {
        info: Info,
        events: Owner<Mutex<Vec<String>>>,
    }
    impl Drop for Handle {
        fn drop(&mut self) {
            self.events.lock().unwrap().push("release".into());
        }
    }
    struct Fake {
        events: Owner<Mutex<Vec<String>>>,
        devices: Mutex<Vec<Info>>,
        product: b2xx::Product,
        incompatible: bool,
        fpga_incompatible: bool,
        pause: bool,
        deny_open_once: std::sync::atomic::AtomicBool,
    }
    impl Fake {
        fn new(product: b2xx::Product) -> Self {
            Self {
                events: Owner::default(),
                devices: Mutex::new(Vec::new()),
                product,
                incompatible: false,
                fpga_incompatible: false,
                pause: false,
                deny_open_once: std::sync::atomic::AtomicBool::new(false),
            }
        }
        fn event(&self, s: impl Into<String>) {
            self.events.lock().unwrap().push(s.into());
        }
    }
    impl Backend for Fake {
        type Descriptor = Info;
        type Handle = Handle;
        type Key = Info;
        type Radio = ();
        fn loaded(&self, i: &Info) -> bool {
            i.loaded
        }
        fn key(&self, i: &Info) -> Info {
            i.clone()
        }
        fn matches(&self, key: &Info, i: &Info, loaded: bool) -> bool {
            key.port == i.port
                && i.loaded == loaded
                && (!loaded || key.serial.is_none() || key.serial == i.serial)
        }
        async fn list(&self) -> Result<Vec<Info>> {
            Ok(self.devices.lock().unwrap().clone())
        }
        async fn open(&self, info: Info) -> Result<Handle> {
            if self.deny_open_once.swap(false, Ordering::Relaxed) {
                return Err(std::io::Error::from(std::io::ErrorKind::PermissionDenied).into());
            }
            self.event(if info.loaded {
                "open-running"
            } else {
                "open-boot"
            });
            Ok(Handle {
                info,
                events: self.events.clone(),
            })
        }
        async fn check_firmware(&self, _: &Handle) -> Result<()> {
            self.event("compatibility");
            if self.incompatible {
                Err(Error::FirmwareCompatibility {
                    expected_major: 8,
                    expected_minor: 0,
                    actual_major: 7,
                    actual_minor: 0,
                })
            } else {
                Ok(())
            }
        }
        async fn reset(&self, d: &Handle) -> Result<()> {
            self.event("reset");
            *self.devices.lock().unwrap() = vec![Info {
                loaded: false,
                ..d.info.clone()
            }];
            Ok(())
        }
        async fn firmware(&self, d: &Handle, image: &[u8]) -> Result<()> {
            crate::ihex::parse(image)?;
            self.event("firmware");
            if self.pause {
                std::future::pending::<()>().await;
            }
            *self.devices.lock().unwrap() = vec![Info {
                loaded: true,
                ..d.info.clone()
            }];
            Ok(())
        }
        async fn product(&self, _: &Handle) -> Result<b2xx::Product> {
            self.event("identity");
            Ok(self.product)
        }
        async fn fpga(&self, _: &Handle, image: &[u8]) -> Result<()> {
            self.event(format!("fpga:{}", image[0]));
            Ok(())
        }
        async fn running_hash(&self, _: &Handle) -> Result<Option<u32>> {
            Ok(None)
        }
        async fn initialize(&self, _: Handle, _: RxConfig) -> Result<()> {
            self.event("initialize");
            if self.fpga_incompatible {
                Err(Error::FpgaCompatibility {
                    expected: 16,
                    actual: 15,
                })
            } else {
                Ok(())
            }
        }
    }
    fn catalog() -> ImageCatalog {
        let mut c = ImageCatalog::default();
        c.insert(
            Image::Firmware,
            b":020000040001F9\n:0400100001020304E2\n:0400000500010010E6\n:00000001FF\n".to_vec(),
        );
        for (n, image) in [Image::B200, Image::B210, Image::B200Mini, Image::B205Mini]
            .into_iter()
            .enumerate()
        {
            c.insert(image, vec![n as u8]);
        }
        c
    }
    fn info(loaded: bool) -> Info {
        Info {
            loaded,
            port: 4,
            serial: Some("radio"),
        }
    }
    #[test]
    fn cold_and_warm_open_select_all_product_images() {
        for (index, product) in [
            b2xx::Product::B200,
            b2xx::Product::B210,
            b2xx::Product::B200Mini,
            b2xx::Product::B205Mini,
        ]
        .into_iter()
        .enumerate()
        {
            for warm in [false, true] {
                let b = Fake::new(product);
                block_on(open(
                    &b,
                    info(warm),
                    &catalog(),
                    false,
                    Duration::from_secs(1),
                    DeviceBuilder::default().config,
                ))
                .unwrap();
                let events = b.events.lock().unwrap();
                assert_eq!(events.contains(&"firmware".into()), !warm);
                assert!(events.contains(&format!("fpga:{index}")));
                assert_eq!(events.last().unwrap(), "release");
            }
        }
    }
    #[test]
    fn incompatible_firmware_fails_before_fpga_or_radio() {
        let mut b = Fake::new(b2xx::Product::B200);
        b.incompatible = true;
        assert!(matches!(
            block_on(open(
                &b,
                info(true),
                &catalog(),
                false,
                Duration::from_secs(1),
                DeviceBuilder::default().config
            )),
            Err(Error::FirmwareCompatibility { .. })
        ));
        assert_eq!(
            *b.events.lock().unwrap(),
            vec!["open-running", "compatibility", "release"]
        );
    }
    #[test]
    fn malformed_firmware_does_not_reset_working_device() {
        let b = Fake::new(b2xx::Product::B200);
        let mut c = catalog();
        c.insert(Image::Firmware, vec![1, 2]);
        assert!(matches!(
            block_on(prepare(&b, info(true), &c, true, Duration::from_secs(1))),
            Err(Error::IntelHex { .. })
        ));
        assert!(b.events.lock().unwrap().is_empty());
    }
    #[test]
    fn explicit_reload_and_fpga_compatibility_failure_release_resources() {
        let mut b = Fake::new(b2xx::Product::B200);
        b.fpga_incompatible = true;
        assert!(matches!(
            block_on(open(
                &b,
                info(true),
                &catalog(),
                true,
                Duration::from_secs(1),
                DeviceBuilder::default().config
            )),
            Err(Error::FpgaCompatibility { .. })
        ));
        assert!(b.events.lock().unwrap().contains(&"reset".into()));
        assert_eq!(b.events.lock().unwrap().last().unwrap(), "release");
    }
    #[test]
    fn reconnect_never_uses_unrelated_device_or_wrong_serial() {
        let b = Fake::new(b2xx::Product::B200);
        *b.devices.lock().unwrap() = vec![
            Info {
                port: 8,
                ..info(true)
            },
            Info {
                serial: Some("unrelated"),
                ..info(true)
            },
        ];
        assert!(block_on(reconnect(&b, &info(true), true, Duration::from_millis(1))).is_err());
    }
    #[test]
    fn cancelled_firmware_load_releases_open_handle() {
        let mut b = Fake::new(b2xx::Product::B200);
        b.pause = true;
        let c = catalog();
        let mut future = Box::pin(prepare(&b, info(false), &c, false, Duration::from_secs(1)));
        assert!(block_on(poll_once(future.as_mut())).is_none());
        drop(future);
        assert_eq!(b.events.lock().unwrap().last().unwrap(), "release");
    }
    #[test]
    fn reconnect_retries_permission_race_on_the_matched_device() {
        let backend = Fake::new(b2xx::Product::B210);
        *backend.devices.lock().unwrap() = vec![info(true)];
        backend.deny_open_once.store(true, Ordering::Relaxed);
        let device = block_on(reconnect(
            &backend,
            &info(true),
            true,
            Duration::from_secs(1),
        ))
        .unwrap();
        assert_eq!(device.info.port, 4);
    }
}
