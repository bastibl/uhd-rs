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
#[ignore = "requires B2xx; receives across the full sample-rate range and changes clocks"]
fn b2xx_receive_sample_rate_range() {
    let mut device = Device::builder()
        .descriptor(descriptor())
        .sample_rate_hz(429.6875)
        .open()
        .wait()
        .unwrap();
    let mut rx = device.rx_stream().unwrap();
    let result = (|| -> uhd_rs::Result<()> {
        let mut samples = [Complex32::default(); 4086];
        for (index, requested) in [
            429.6875,
            1e3,
            10e3,
            31_249.0,
            16e6 / 257.0,
            250e3,
            1.1e6,
            7e6,
            15.36e6,
            16e6,
            19e6,
            20e6,
            21e6,
            23e6,
            30.72e6,
            40e6,
            41e6,
            56e6,
            58e6,
            59e6,
            61.44e6,
            1e6,
        ]
        .into_iter()
        .enumerate()
        {
            let actual = device.set_sample_rate(requested).wait()?;
            assert!((actual - requested).abs() < 1.0, "{requested} -> {actual}");
            device
                .set_gain(if index % 2 == 0 {
                    uhd_rs::RxGain::Manual(30.0)
                } else {
                    uhd_rs::RxGain::Automatic
                })
                .wait()?;
            rx.start().wait()?;
            // Reapplying a rate must not try to recalibrate a quantized clock.
            assert_eq!(device.set_sample_rate(requested).wait()?, actual);
            let timeout = Duration::from_secs_f64(2.0 * samples.len() as f64 / actual + 3.0);
            let count = rx.read(&mut samples, Some(timeout)).wait()?;
            assert!(count > 0);
            assert!(
                samples[..count]
                    .iter()
                    .all(|s| s.re.is_finite() && s.im.is_finite())
            );
            assert!(samples[..count].iter().any(|s| s.norm_sqr() > 0.0));
            println!(
                "Requested {requested:.6} S/s, actual {actual:.6} S/s: received {count} samples"
            );
            rx.stop().wait()?;
        }
        rx.start().wait()?;
        assert!(matches!(
            device.set_sample_rate(7e6).wait(),
            Err(uhd_rs::Error::Busy)
        ));
        assert!(rx.read(&mut samples, Some(Duration::from_secs(3))).wait()? > 0);
        rx.stop().wait()?;
        device
            .tune(uhd_rs::RxTuneRequest::with_lo_offset(100e6, 3.6e6))
            .wait()?;
        assert!(matches!(
            device.set_sample_rate(429.6875).wait(),
            Err(uhd_rs::Error::InvalidArgument(_))
        ));
        // An invalid clock/LO combination must leave the previous configuration usable.
        rx.start().wait()?;
        assert!(rx.read(&mut samples, Some(Duration::from_secs(3))).wait()? > 0);
        rx.stop().wait()?;
        device.set_center_frequency(100e6).wait()?;
        Ok(())
    })();
    let close = rx.close().wait();
    let shutdown = device.shutdown().wait();
    result.unwrap();
    close.unwrap();
    shutdown.unwrap();
}

#[test]
#[ignore = "requires B2xx on USB 3; measures sustained RX up to 61.44 MS/s"]
fn b2xx_receive_sample_rate_throughput() {
    let mut device = Device::builder()
        .descriptor(descriptor())
        .open()
        .wait()
        .unwrap();
    let mut rx = device.rx_stream().unwrap();
    let result = (|| -> uhd_rs::Result<()> {
        let mut samples = [Complex32::default(); 4086];
        for requested in [1.1e6, 7e6, 19e6, 20e6, 30.72e6, 40e6, 56e6, 61.44e6] {
            let actual = device.set_sample_rate(requested).wait()?;
            rx.start().wait()?;
            let warmup = std::time::Instant::now();
            while warmup.elapsed() < Duration::from_secs(1) {
                rx.read(&mut samples, Some(Duration::from_secs(3))).wait()?;
            }
            let before = rx.stats();
            let start = std::time::Instant::now();
            while start.elapsed() < Duration::from_secs(2) {
                rx.read(&mut samples, Some(Duration::from_secs(3))).wait()?;
            }
            let elapsed = start.elapsed().as_secs_f64();
            let after = rx.stats();
            let measured = (after.samples - before.samples) as f64 / elapsed;
            println!(
                "Requested {:.3} MS/s, measured {:.3} MS/s, {} overflows",
                requested / 1e6,
                measured / 1e6,
                after.overflows - before.overflows
            );
            rx.stop().wait()?;
            assert!(
                measured >= actual * 0.95,
                "{requested}: measured {measured} S/s"
            );
        }
        Ok(())
    })();
    let close = rx.close().wait();
    let shutdown = device.shutdown().wait();
    result.unwrap();
    close.unwrap();
    shutdown.unwrap();
}

#[test]
#[ignore = "requires one B2xx on USB 3; measures sustained 20 MS/s RX"]
fn b2xx_wlan_receive_throughput() {
    let mut device = Device::builder()
        .descriptor(descriptor())
        .sample_rate_hz(20e6)
        .open()
        .wait()
        .unwrap();
    let mut rx = device.rx_stream().unwrap();
    let measured = (|| -> uhd_rs::Result<f64> {
        rx.start().wait()?;
        let mut samples = [Complex32::default(); 4086];
        let warmup = std::time::Instant::now();
        while warmup.elapsed() < Duration::from_secs(1) {
            rx.read(&mut samples, Some(Duration::from_secs(2))).wait()?;
        }
        let before = rx.stats();
        let start = std::time::Instant::now();
        while start.elapsed() < Duration::from_secs(5) {
            rx.read(&mut samples, Some(Duration::from_secs(2))).wait()?;
        }
        let after = rx.stats();
        let rate = (after.samples - before.samples) as f64 / start.elapsed().as_secs_f64();
        println!(
            "RX: {:.3} MS/s, {} transfers, {} overflows",
            rate / 1e6,
            after.transfers - before.transfers,
            after.overflows - before.overflows
        );
        Ok(rate)
    })();
    let close = rx.close().wait();
    let shutdown = device.shutdown().wait();
    close.unwrap();
    shutdown.unwrap();
    assert!(
        measured.unwrap() >= 19e6,
        "RX did not sustain the WLAN sample rate"
    );
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
#[ignore = "requires B2xx; forces cold firmware startup, receives and restarts RX"]
fn b2xx_cold_receive_restart_cancel_and_reopen() {
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
#[ignore = "requires B210; verifies channel-zero routing and releases USB"]
fn b210_radio_initialization_and_release() {
    futures_lite::future::block_on(async {
        let images = ImageCatalog::default();
        let device = b2xx::load_firmware_and_reconnect(
            descriptor(),
            &images,
            false,
            Duration::from_secs(10),
        )
        .await
        .unwrap();
        let image = images.get(Image::fpga(b2xx::Product::B210)).unwrap();
        device.load_fpga(&image, false).await.unwrap();
        let mut session = device.open_session().await.unwrap();
        assert_eq!(session.product(), b2xx::Product::B210);
        assert_eq!(session.radio_chains(), 2);
        assert!(session.radio_initialized());
        // B210 channel zero (A) is wired to AD9361 frontend two.
        assert_eq!(
            session.ad9361().read_register(0x003).await.unwrap() & 0xc0,
            0x80
        );
        assert_eq!(
            session.ad9361().read_register(0x002).await.unwrap() & 0xc0,
            0
        );
        assert_eq!(session.ad9361().read_register(0x10c).await.unwrap(), 0);
        drop(session);
        let mut device = Device::builder()
            .descriptor(descriptor())
            .open()
            .await
            .unwrap();
        assert_eq!(
            device.identity().await.unwrap().product,
            Some(b2xx::Product::B210)
        );
        device.shutdown().await.unwrap();
        descriptor()
            .open()
            .await
            .unwrap()
            .check_firmware_compatibility()
            .await
            .unwrap();
    });
}
