//! Explicit opt-in; run one hardware test at a time with --ignored --test-threads=1.
#![cfg(all(feature = "hardware-tests", not(target_arch = "wasm32")))]
use std::time::Duration;
use uhd_rs::{
    Complex32, Device, MaybeFuture,
    b2xx::{self},
    images::{Image, ImageCatalog},
};
fn descriptor() -> b2xx::B2xxDeviceInfo {
    let mut devices = Device::list().wait().unwrap();
    if let Ok(serial) = std::env::var("UHD_RS_SERIAL") {
        devices.retain(|d| d.serial_number.as_deref() == Some(&serial));
    }
    assert_eq!(devices.len(), 1, "connect one B2xx or set UHD_RS_SERIAL");
    devices.pop().unwrap()
}
#[test]
#[ignore = "loads volatile firmware/FPGA and accesses physical B2xx hardware"]
fn b2xx_startup_images_and_diagnostics() {
    futures_lite::future::block_on(async {
        let images = ImageCatalog::default();
        let device = b2xx::load_firmware_and_reconnect(
            descriptor(),
            &images,
            std::env::var_os("UHD_RS_RELOAD_FIRMWARE").is_some(),
            Duration::from_secs(10),
        )
        .await
        .unwrap();
        let identity = device.identity().await.unwrap();
        let product = identity.product.unwrap();
        println!("{identity:?}");
        let image = images.get(Image::fpga(product)).unwrap();
        let outcome = device.load_fpga(&image, false).await.unwrap();
        println!("FPGA: {outcome:?}");
        assert_eq!(
            device.load_fpga(&image, false).await.unwrap(),
            b2xx::LoadOutcome::AlreadyLoaded
        );
        let mut session = device.open_fpga_session().await.unwrap();
        println!(
            "FPGA {:?}, {} radio chains",
            session.fpga_compatibility(),
            session.radio_chains()
        );
        let compatibility = session.control_mut().peek64(0).await.unwrap();
        assert_eq!(compatibility >> 32, 0xace0_ba5e);
        drop(session);
        let reopened = descriptor().open().await.unwrap();
        reopened.check_firmware_compatibility().await.unwrap();
    });
}
#[test]
#[ignore = "requires B200 revision 5+; forces cold firmware startup, receives and restarts RX"]
fn b200_cold_receive_restart_cancel_and_reopen() {
    let mut device = Device::builder()
        .descriptor(descriptor())
        .reload_firmware(true)
        .open()
        .wait()
        .unwrap();
    let mut rx = device.rx_stream().unwrap();
    rx.start().wait().unwrap();
    let mut samples = [Complex32::default(); 4096];
    assert!(
        rx.read(&mut samples, Some(Duration::from_secs(3)))
            .wait()
            .unwrap()
            > 0
    );
    rx.stop().wait().unwrap();
    rx.start().wait().unwrap();
    assert!(
        rx.read(&mut samples, Some(Duration::from_secs(3)))
            .wait()
            .unwrap()
            > 0
    );
    rx.close().wait().unwrap();
    device.shutdown().wait().unwrap();
    let mut device = Device::builder()
        .descriptor(descriptor())
        .open()
        .wait()
        .unwrap();
    let mut rx = device.rx_stream().unwrap();
    rx.start().wait().unwrap();
    {
        use std::future::IntoFuture;
        let mut read = Box::pin(rx.read(&mut samples, None).into_future());
        let _ = futures_lite::future::block_on(futures_lite::future::poll_once(read.as_mut()));
    }
    drop(rx);
    device.shutdown().wait().unwrap();
    if let Ok(program) = std::env::var("UHD_RS_HANDOFF_COMMAND") {
        assert!(
            std::process::Command::new(program)
                .status()
                .unwrap()
                .success()
        );
    }
}
#[test]
#[ignore = "requires B210; verifies the supported-radio boundary after automatic startup"]
fn b210_high_level_rejects_unsupported_radio_and_releases_usb() {
    let result = Device::builder().descriptor(descriptor()).open().wait();
    assert!(matches!(result,Err(uhd_rs::Error::Unsupported(message)) if message.contains("B200")));
    let device = descriptor().open().wait().unwrap();
    futures_lite::future::block_on(device.check_firmware_compatibility()).unwrap();
    let session = futures_lite::future::block_on(device.open_fpga_session()).unwrap();
    assert_eq!(session.product(), b2xx::Product::B210);
}
