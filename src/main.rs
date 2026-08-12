#[cfg(not(target_arch = "wasm32"))]
fn main() {
    if let Err(error) = futures_lite::future::block_on(run()) {
        eprintln!("error: {error}");
        std::process::exit(1);
    }
}

#[cfg(target_arch = "wasm32")]
fn main() {}

#[cfg(not(target_arch = "wasm32"))]
async fn run() -> uhd_pure::Result<()> {
    use uhd_pure::b2xx::{self, StreamId};

    let mut arguments = std::env::args().skip(1);
    let command = arguments.next().unwrap_or_else(|| "list".into());
    let arguments: Vec<String> = arguments.collect();
    if command == "list" {
        let devices = b2xx::list_devices().await?;
        if devices.is_empty() {
            println!("No USRP B2xx USB device found.");
        }
        for (index, device) in devices.iter().enumerate() {
            println!(
                "{index}: {:04x}:{:04x} {} serial={} firmware={}",
                device.vendor_id,
                device.product_id,
                device.product_string.as_deref().unwrap_or("B2xx"),
                device.serial_number.as_deref().unwrap_or("unknown"),
                if device.firmware_loaded {
                    "loaded"
                } else {
                    "bootloader"
                }
            );
        }
        return Ok(());
    }
    if matches!(command.as_str(), "help" | "--help" | "-h") {
        print_help();
        return Ok(());
    }
    if !matches!(
        command.as_str(),
        "probe" | "load-firmware" | "load-fpga" | "peek" | "poke" | "ad9361-read" | "loopback"
    ) {
        return Err(uhd_pure::Error::InvalidArgument(format!(
            "unknown command {command:?}; run `uhd-pure help`"
        )));
    }
    if command == "load-firmware" {
        return load_firmware(&arguments).await;
    }

    let serial = match command.as_str() {
        "probe" => arguments.first(),
        "peek" | "ad9361-read" => arguments.get(1),
        "load-fpga" => arguments.get(1).filter(|value| value.as_str() != "--force"),
        "poke" => arguments.get(2),
        "loopback" => arguments.get(1),
        _ => None,
    };
    let info = select_device(serial.map(String::as_str)).await?;
    let device = info.open().await?;
    match command.as_str() {
        "probe" => {
            let compatibility = device.check_firmware_compatibility().await?;
            let speed = device.usb_speed().await?;
            let state = device.fx3_state().await?;
            let identity = device.identity().await?;
            println!(
                "USRP {} serial={} name={:?} revision={} USB {} firmware={}.{} FX3={}",
                identity.product.map_or_else(
                    || format!("code 0x{:04x}", identity.product_code),
                    |p| p.to_string()
                ),
                identity.serial,
                identity.name,
                identity.revision,
                speed.major_version(),
                compatibility.major,
                compatibility.minor,
                state
            );
        }
        "load-fpga" => {
            let path = required_argument(&arguments, 0, "FPGA .bin path")?;
            let image = std::fs::read(path)?;
            let force = arguments.iter().skip(1).any(|value| value == "--force");
            let outcome = device.load_fpga(&image, force).await?;
            device.reset_gpif().await?;
            println!("FPGA result: {outcome:?}; GPIF reset complete.");
        }
        "peek" => {
            let address = parse_number(required_argument(&arguments, 0, "byte address")?)?;
            require_running(&device).await?;
            let transport = device.open_transport().await?;
            let mut control = transport.into_radio_control(StreamId::LocalControl);
            println!("0x{:08x}", control.peek32(address).await?);
        }
        "poke" => {
            let address = parse_number(required_argument(&arguments, 0, "byte address")?)?;
            let value = parse_number(required_argument(&arguments, 1, "32-bit value")?)?;
            require_running(&device).await?;
            let transport = device.open_transport().await?;
            let mut control = transport.into_radio_control(StreamId::LocalControl);
            control.poke32(address, value).await?;
            println!("wrote 0x{value:08x} to 0x{address:08x}");
        }
        "ad9361-read" => {
            let register = parse_u16(required_argument(&arguments, 0, "register address")?)?;
            require_running(&device).await?;
            let transport = device.open_transport().await?;
            let mut control = transport.into_radio_control(StreamId::LocalControl);
            let spi = b2xx::B2xxSpi::new(&mut control);
            let mut codec = b2xx::Ad9361Io::new(spi);
            println!("0x{:02x}", codec.read_register(register).await?);
        }
        "loopback" => {
            let channel = arguments
                .first()
                .map_or(Ok(0), |value| parse_u8(value, "radio channel"))?;
            if channel > 1 {
                return Err(uhd_pure::Error::InvalidArgument(
                    "radio channel must be 0 or 1".into(),
                ));
            }
            require_running(&device).await?;
            let transport = device.open_transport().await?;
            let mut control = transport.into_radio_control(StreamId::RadioControl(channel));
            for pattern in [0x0000_0000, 0xffff_ffff, 0xa5a5_5a5a, 0x0123_4567] {
                control.poke32(0x54, pattern).await?;
                let actual = control.peek32(0).await?;
                if actual != pattern {
                    return Err(uhd_pure::Error::InvalidArgument(format!(
                        "loopback mismatch: wrote 0x{pattern:08x}, read 0x{actual:08x}"
                    )));
                }
            }
            println!("radio {channel} register loopback passed");
        }
        _ => unreachable!("command validated above"),
    }
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
async fn load_firmware(arguments: &[String]) -> uhd_pure::Result<()> {
    use std::time::Duration;

    const REENUMERATION_TIMEOUT: Duration = Duration::from_secs(10);
    const POLL_INTERVAL: Duration = Duration::from_millis(50);

    if arguments.len() > 2 {
        return Err(uhd_pure::Error::InvalidArgument(
            "usage: uhd-pure load-firmware <usrp_b200_fw.hex> [serial]".into(),
        ));
    }
    let path = required_argument(arguments, 0, "firmware Intel HEX path")?;
    let image = std::fs::read(path)?;
    // Validate the complete image before resetting a working device.
    uhd_pure::ihex::parse(&image)?;

    let info = select_device(arguments.get(1).map(String::as_str)).await?;
    let bus_id = info.bus_id().to_owned();
    let port_chain = info.port_chain().to_vec();
    let expected_serial = info
        .firmware_loaded
        .then(|| info.serial_number.clone())
        .flatten();
    let bootloader_info = if info.firmware_loaded {
        println!("Resetting running FX3 firmware to the bootloader...");
        let device = info.open().await?;
        device.reset_fx3().await?;
        drop(device);
        wait_for_device(
            &bus_id,
            &port_chain,
            false,
            None,
            "enter its FX3 bootloader",
            REENUMERATION_TIMEOUT,
            POLL_INTERVAL,
        )
        .await?
    } else {
        info
    };

    println!("Loading FX3 firmware from {path}...");
    let bootloader = bootloader_info.open().await?;
    bootloader.load_firmware(&image).await?;
    drop(bootloader);

    let loaded_info = wait_for_device(
        &bus_id,
        &port_chain,
        true,
        expected_serial.as_deref(),
        "re-enumerate with firmware",
        REENUMERATION_TIMEOUT,
        POLL_INTERVAL,
    )
    .await?;
    let loaded = loaded_info.open().await?;
    let compatibility = loaded.check_firmware_compatibility().await?;
    let identity = loaded.identity().await?;
    println!(
        "Firmware {}.{} running on {} serial={} name={:?}.",
        compatibility.major,
        compatibility.minor,
        identity.product.map_or("B2xx", |product| product.name()),
        identity.serial,
        identity.name
    );
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
async fn wait_for_device(
    bus_id: &str,
    port_chain: &[u8],
    firmware_loaded: bool,
    expected_serial: Option<&str>,
    state: &'static str,
    timeout: std::time::Duration,
    poll_interval: std::time::Duration,
) -> uhd_pure::Result<uhd_pure::b2xx::B2xxDeviceInfo> {
    let deadline = std::time::Instant::now() + timeout;
    loop {
        let mut candidates: Vec<_> = uhd_pure::b2xx::list_devices()
            .await?
            .into_iter()
            .filter(|info| info.firmware_loaded == firmware_loaded)
            .collect();
        let position = candidates
            .iter()
            .position(|info| info.bus_id() == bus_id && info.port_chain() == port_chain)
            .or_else(|| {
                expected_serial.and_then(|serial| {
                    candidates
                        .iter()
                        .position(|info| info.serial_number.as_deref() == Some(serial))
                })
            })
            .or_else(|| (candidates.len() == 1).then_some(0));
        if let Some(position) = position {
            let info = candidates.swap_remove(position);
            return Ok(info);
        }
        if std::time::Instant::now() >= deadline {
            return Err(uhd_pure::Error::DeviceReenumerationTimeout { state, timeout });
        }
        futures_timer::Delay::new(poll_interval).await;
    }
}

#[cfg(not(target_arch = "wasm32"))]
async fn require_running(device: &uhd_pure::b2xx::B2xxDevice) -> uhd_pure::Result<()> {
    let state = device.fx3_state().await?;
    if state != uhd_pure::b2xx::Fx3State::Running {
        return Err(uhd_pure::Error::Fx3State(state));
    }
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
async fn select_device(serial: Option<&str>) -> uhd_pure::Result<uhd_pure::b2xx::B2xxDeviceInfo> {
    let devices = uhd_pure::b2xx::list_devices().await?;
    if let Some(serial) = serial {
        devices
            .into_iter()
            .find(|device| device.serial_number.as_deref() == Some(serial))
            .ok_or(uhd_pure::Error::DeviceNotFound)
    } else if devices.len() == 1 {
        Ok(devices.into_iter().next().expect("one device"))
    } else if devices.is_empty() {
        Err(uhd_pure::Error::DeviceNotFound)
    } else {
        Err(uhd_pure::Error::InvalidArgument(
            "multiple B2xx devices found; pass the serial number after the command arguments"
                .into(),
        ))
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn required_argument<'a>(
    arguments: &'a [String],
    index: usize,
    description: &str,
) -> uhd_pure::Result<&'a str> {
    arguments
        .get(index)
        .map(String::as_str)
        .ok_or_else(|| uhd_pure::Error::InvalidArgument(format!("missing required {description}")))
}

#[cfg(not(target_arch = "wasm32"))]
fn parse_number(value: &str) -> uhd_pure::Result<u32> {
    let parsed = if let Some(hex) = value.strip_prefix("0x") {
        u32::from_str_radix(hex, 16)
    } else {
        value.parse()
    };
    parsed.map_err(|_| uhd_pure::Error::InvalidArgument(format!("invalid number {value:?}")))
}

#[cfg(not(target_arch = "wasm32"))]
fn parse_u16(value: &str) -> uhd_pure::Result<u16> {
    u16::try_from(parse_number(value)?).map_err(|_| {
        uhd_pure::Error::InvalidArgument(format!("number does not fit in 16 bits: {value:?}"))
    })
}

#[cfg(not(target_arch = "wasm32"))]
fn parse_u8(value: &str, description: &str) -> uhd_pure::Result<u8> {
    u8::try_from(parse_number(value)?).map_err(|_| {
        uhd_pure::Error::InvalidArgument(format!("{description} does not fit in 8 bits: {value:?}"))
    })
}

#[cfg(not(target_arch = "wasm32"))]
fn print_help() {
    println!(
        "uhd-pure commands:\n  list\n  probe [serial]\n  load-firmware <usrp_b200_fw.hex> [serial]\n  load-fpga <usrp_b2xx_fpga.bin> [serial] [--force]\n  peek <byte-address> [serial]\n  poke <byte-address> <value> [serial]\n  ad9361-read <register> [serial]\n  loopback [channel] [serial]"
    );
}
