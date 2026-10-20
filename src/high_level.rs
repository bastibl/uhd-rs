//! Owned channel-zero RX API.
use crate::{
    Complex32, Error, Result, RxAntenna, RxConfig, RxGain, RxTuneRequest, RxTuneResult,
    b2xx::{
        self,
        rx::{B2xxReceiver, RxSequenceState, SequenceDisposition},
    },
    images::{Image, ImageCatalog},
    operation::{bounded, operation},
};
use async_lock::Mutex as AsyncMutex;
use nusb::{
    MaybeFuture,
    transfer::{Buffer, Bulk, In},
};
use std::{
    sync::{
        Mutex,
        atomic::{AtomicU32, Ordering},
    },
    time::Duration,
};

#[cfg(target_arch = "wasm32")]
use std::rc::Rc as Owner;
#[cfg(not(target_arch = "wasm32"))]
use std::sync::Arc as Owner;

mod io;
mod startup;
use io::{Endpoint, Radio};

pub type DeviceDescriptor = b2xx::B2xxDeviceInfo;
const CLEANUP_TIMEOUT: Duration = Duration::from_secs(2);
// A 64-packet queue covers only 13 ms at 20 MS/s. Browser rendering and GC
// can delay WebUSB completion callbacks longer than that. Keep about 105 ms
// queued on wasm (34 ms at 61.44 MS/s) without increasing read batch latency.
const QUEUE_DEPTH: usize = if cfg!(target_arch = "wasm32") {
    512
} else {
    64
};
const TRANSFER_BYTES: usize = 16_384;
// Bound work per call while amortizing downstream scheduling across ready packets.
const READ_BATCH_PACKETS: usize = 4;

/// Cumulative statistics for this stream, including restarts.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct StreamingStats {
    pub samples: u64,
    pub transfers: u64,
    pub overflows: u64,
}
#[derive(Default)]
struct Life {
    claimed: bool,
    terminal: bool,
    complete: bool,
    reopen: bool,
}
struct Shared {
    radio: AsyncMutex<Option<Radio>>,
    life: Mutex<Life>,
    scale: AtomicU32,
}
impl Shared {
    fn active(&self) -> Result<()> {
        if self.life.lock().unwrap().terminal {
            Err(Error::Shutdown)
        } else {
            Ok(())
        }
    }
}
impl Drop for Shared {
    fn drop(&mut self) {
        if let Some(mut radio) = self.radio.get_mut().take() {
            #[cfg(not(target_arch = "wasm32"))]
            {
                let _ = crate::operation::blocking(bounded(
                    async {
                        radio.stop().await?;
                        radio.terminal_close().await
                    },
                    CLEANUP_TIMEOUT,
                ));
            }
            #[cfg(target_arch = "wasm32")]
            wasm_bindgen_futures::spawn_local(async move {
                let _ = bounded(radio.stop(), CLEANUP_TIMEOUT).await;
                let _ = radio.terminal_close().await;
            });
        }
    }
}

/// A prepared B2xx channel-zero receiver. Dropping it does not invalidate an owned stream.
pub struct Device {
    shared: Owner<Shared>,
}
impl Device {
    pub fn builder() -> DeviceBuilder {
        DeviceBuilder::default()
    }
    pub fn list() -> impl MaybeFuture<Output = Result<Vec<DeviceDescriptor>>> {
        operation(b2xx::list_devices())
    }
    /// On wasm, poll this from a user gesture before opening.
    pub fn request_permission() -> impl MaybeFuture<Output = Result<Option<DeviceDescriptor>>> {
        operation(b2xx::request_device())
    }
    /// Return the motherboard EEPROM identity, independent of USB product labels.
    pub fn identity(&self) -> impl MaybeFuture<Output = Result<b2xx::B2xxIdentity>> + '_ {
        operation(async move {
            let radio = self.shared.radio.lock().await;
            self.shared.active()?;
            radio.as_ref().ok_or(Error::Shutdown)?.identity()
        })
    }
    /// Claim the single sample endpoint. No transfers are submitted until start.
    pub fn rx_stream(&mut self) -> Result<RxStream> {
        let mut radio = self.shared.radio.try_lock().ok_or(Error::Busy)?;
        let mut life = self.shared.life.lock().unwrap();
        if life.terminal {
            return Err(Error::Shutdown);
        }
        if life.claimed {
            return Err(Error::Busy);
        }
        if life.reopen {
            return Err(Error::ReopenRequired);
        }
        let endpoint = radio.as_mut().ok_or(Error::Shutdown)?.take_endpoint()?;
        life.claimed = true;
        Ok(RxStream {
            shared: self.shared.clone(),
            endpoint: Some(endpoint),
            buffer: SampleBuffer::default(),
            pending_error: None,
            sequence: RxSequenceState::Synchronizing,
            running: false,
            failed: false,
            submitted: false,
            claimed: true,
            stats: StreamingStats::default(),
        })
    }
    pub fn tune(
        &mut self,
        request: RxTuneRequest,
    ) -> impl MaybeFuture<Output = Result<RxTuneResult>> + '_ {
        operation(async move {
            let mut radio = self.shared.radio.lock().await;
            self.shared.active()?;
            radio.as_mut().ok_or(Error::Shutdown)?.tune(request).await
        })
    }
    pub fn set_center_frequency(&mut self, hz: f64) -> impl MaybeFuture<Output = Result<f64>> + '_ {
        operation(async move {
            Ok(self
                .tune(RxTuneRequest::new(hz))
                .await?
                .actual_center_frequency_hz)
        })
    }
    /// Set the receive rate from 429.6875 S/s through 61.44 MS/s, returning the
    /// actual quantized rate. Stop the stream before changing the master clock,
    /// which is selected automatically for the requested rate.
    pub fn set_sample_rate(&mut self, hz: f64) -> impl MaybeFuture<Output = Result<f64>> + '_ {
        operation(async move {
            let mut guard = self.shared.radio.lock().await;
            self.shared.active()?;
            let radio = guard.as_mut().ok_or(Error::Shutdown)?;
            let rate = radio.set_sample_rate(hz).await?;
            self.shared
                .scale
                .store(radio.host_scale().to_bits(), Ordering::Relaxed);
            Ok(rate)
        })
    }
    pub fn set_gain(&mut self, gain: RxGain) -> impl MaybeFuture<Output = Result<()>> + '_ {
        operation(async move {
            let mut radio = self.shared.radio.lock().await;
            self.shared.active()?;
            radio.as_mut().ok_or(Error::Shutdown)?.set_gain(gain).await
        })
    }

    /// Return the selected receive connector.
    pub fn antenna(&self) -> impl MaybeFuture<Output = Result<RxAntenna>> + '_ {
        operation(async move {
            let guard = self.shared.radio.lock().await;
            self.shared.active()?;
            guard.as_ref().ok_or(Error::Shutdown)?.antenna()
        })
    }

    /// Select RX2 or the shared TX/RX connector for reception.
    pub fn set_antenna(
        &mut self,
        antenna: RxAntenna,
    ) -> impl MaybeFuture<Output = Result<()>> + '_ {
        operation(async move {
            let mut guard = self.shared.radio.lock().await;
            self.shared.active()?;
            guard
                .as_mut()
                .ok_or(Error::Shutdown)?
                .set_antenna(antenna)
                .await
        })
    }

    /// Return the configured analog receive bandwidth in Hz.
    pub fn bandwidth(&self) -> impl MaybeFuture<Output = Result<f64>> + '_ {
        operation(async move {
            let guard = self.shared.radio.lock().await;
            self.shared.active()?;
            guard.as_ref().ok_or(Error::Shutdown)?.bandwidth()
        })
    }

    /// Set analog receive bandwidth (200 kHz through 56 MHz), limited by the
    /// master clock, and return the applied value. Zero restores automatic
    /// bandwidth following the master clock. Manual requests persist across
    /// clock changes and are reapplied within the new clock's limit.
    pub fn set_bandwidth(&mut self, hz: f64) -> impl MaybeFuture<Output = Result<f64>> + '_ {
        operation(async move {
            let mut guard = self.shared.radio.lock().await;
            self.shared.active()?;
            guard
                .as_mut()
                .ok_or(Error::Shutdown)?
                .set_bandwidth(hz)
                .await
        })
    }

    /// Return whether automatic baseband DC offset correction is enabled.
    pub fn dc_offset_enabled(&self) -> impl MaybeFuture<Output = Result<bool>> + '_ {
        operation(async move {
            let guard = self.shared.radio.lock().await;
            self.shared.active()?;
            guard.as_ref().ok_or(Error::Shutdown)?.dc_offset_enabled()
        })
    }

    /// Toggle baseband DC tracking, retaining static RF and BB calibration.
    pub fn set_dc_offset_enabled(
        &mut self,
        enabled: bool,
    ) -> impl MaybeFuture<Output = Result<()>> + '_ {
        operation(async move {
            let mut guard = self.shared.radio.lock().await;
            self.shared.active()?;
            guard
                .as_mut()
                .ok_or(Error::Shutdown)?
                .set_dc_offset_enabled(enabled)
                .await
        })
    }
    /// Terminal once polled. Busy rejection is nonterminal. Failures may be retried.
    pub fn shutdown(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        operation(async move {
            {
                let mut life = self.shared.life.lock().unwrap();
                if life.claimed {
                    return Err(Error::Busy);
                }
                if life.complete {
                    return Ok(());
                }
                life.terminal = true;
            }
            let mut radio = self.shared.radio.lock().await;
            if let Some(radio) = radio.as_mut() {
                radio.stop().await?;
                radio.terminal_close().await?;
            }
            *radio = None;
            self.shared.life.lock().unwrap().complete = true;
            Ok(())
        })
    }
}

/// Image and radio options for lazy startup. Defaults: 100 MHz, 1 MS/s, 30 dB.
#[derive(Clone, Debug)]
pub struct DeviceBuilder {
    descriptor: Option<DeviceDescriptor>,
    serial: Option<String>,
    config: RxConfig,
    lo_offset: f64,
    antenna: RxAntenna,
    bandwidth_hz: f64,
    dc_offset_enabled: bool,
    images: ImageCatalog,
    reload_firmware: bool,
    reconnect_timeout: Duration,
}
impl Default for DeviceBuilder {
    fn default() -> Self {
        Self {
            descriptor: None,
            serial: None,
            config: RxConfig {
                center_frequency_hz: 100e6,
                sample_rate_hz: 1e6,
                gain: RxGain::Manual(30.0),
            },
            lo_offset: 0.0,
            antenna: RxAntenna::Rx2,
            bandwidth_hz: 0.0,
            dc_offset_enabled: true,
            images: ImageCatalog::default(),
            reload_firmware: false,
            reconnect_timeout: Duration::from_secs(10),
        }
    }
}
impl DeviceBuilder {
    pub fn descriptor(mut self, descriptor: DeviceDescriptor) -> Self {
        self.descriptor = Some(descriptor);
        self
    }
    pub fn serial(mut self, serial: impl Into<String>) -> Self {
        self.serial = Some(serial.into());
        self
    }
    pub fn frequency_hz(mut self, hz: f64) -> Self {
        self.config.center_frequency_hz = hz;
        self
    }
    pub fn sample_rate_hz(mut self, hz: f64) -> Self {
        self.config.sample_rate_hz = hz;
        self
    }
    pub fn gain(mut self, gain: RxGain) -> Self {
        self.config.gain = gain;
        self
    }
    pub fn lo_offset_hz(mut self, hz: f64) -> Self {
        self.lo_offset = hz;
        self
    }
    /// Select the receive connector; defaults to RX2.
    pub fn antenna(mut self, antenna: RxAntenna) -> Self {
        self.antenna = antenna;
        self
    }
    /// Select analog bandwidth in Hz; zero (the default) follows the master clock.
    pub fn bandwidth_hz(mut self, hz: f64) -> Self {
        self.bandwidth_hz = hz;
        self
    }
    /// Enable or disable automatic DC offset correction; enabled by default.
    pub fn dc_offset_enabled(mut self, enabled: bool) -> Self {
        self.dc_offset_enabled = enabled;
        self
    }
    pub fn image(mut self, image: Image, bytes: impl Into<Vec<u8>>) -> Self {
        self.images.insert(image, bytes);
        self
    }
    pub fn images(mut self, catalog: ImageCatalog) -> Self {
        self.images = catalog;
        self
    }
    pub fn reload_firmware(mut self, reload: bool) -> Self {
        self.reload_firmware = reload;
        self
    }
    pub fn reconnect_timeout(mut self, timeout: Duration) -> Self {
        self.reconnect_timeout = timeout;
        self
    }
    pub fn open(self) -> impl MaybeFuture<Output = Result<Device>> {
        operation(self.open_async())
    }
    async fn open_async(self) -> Result<Device> {
        self.config.validate()?;
        b2xx::rx::validate_bandwidth(self.bandwidth_hz)?;
        let tune = RxTuneRequest::with_lo_offset(self.config.center_frequency_hz, self.lo_offset)
            .validate_for_clock(b2xx::rx::clock_for_rate(self.config.sample_rate_hz))?;
        let info = match self.descriptor {
            Some(info) => info,
            None => {
                let mut devices = b2xx::list_devices().await?.into_iter().filter(|d| {
                    self.serial
                        .as_ref()
                        .is_none_or(|s| d.serial_number.as_ref() == Some(s))
                });
                let info = devices.next().ok_or(Error::DeviceNotFound)?;
                if devices.next().is_some() {
                    return Err(Error::InvalidArgument(
                        "multiple B2xx devices; select a serial or descriptor".into(),
                    ));
                }
                info
            }
        };
        let radio = startup::open(
            &startup::Usb,
            info,
            &self.images,
            self.reload_firmware,
            self.reconnect_timeout,
            self.config,
        )
        .await?;
        let scale = radio.host_scale();
        let mut device = Device {
            shared: Owner::new(Shared {
                radio: AsyncMutex::new(Some(radio)),
                life: Mutex::new(Life::default()),
                scale: AtomicU32::new(scale.to_bits()),
            }),
        };
        if self.lo_offset != 0.0 {
            device.tune(tune).await?;
        }
        if self.antenna != RxAntenna::Rx2 {
            device.set_antenna(self.antenna).await?;
        }
        if self.bandwidth_hz != 0.0 {
            device.set_bandwidth(self.bandwidth_hz).await?;
        }
        if !self.dc_offset_enabled {
            device.set_dc_offset_enabled(false).await?;
        }
        Ok(device)
    }
}

/// Load/reload firmware and reconnect to the same physical device.
pub(crate) async fn prepare_firmware(
    info: DeviceDescriptor,
    images: &ImageCatalog,
    reload: bool,
    timeout: Duration,
) -> Result<b2xx::B2xxDevice> {
    startup::prepare(&startup::Usb, info, images, reload, timeout).await
}

struct ReconnectIdentity {
    #[cfg(target_os = "linux")]
    connector: Option<std::path::PathBuf>,
    serial: Option<String>,
    #[cfg(not(target_arch = "wasm32"))]
    bus: String,
    #[cfg(not(target_arch = "wasm32"))]
    ports: Vec<u8>,
}
impl ReconnectIdentity {
    fn new(info: &DeviceDescriptor) -> Self {
        Self {
            #[cfg(target_os = "linux")]
            connector: info.physical_port_key(),
            // Native backends identify the physical port across firmware load.
            // WebUSB only exposes serials, including the bootloader's serial.
            serial: (cfg!(target_arch = "wasm32") || info.firmware_loaded)
                .then(|| info.serial_number.clone())
                .flatten()
                .filter(|s| !s.is_empty()),
            #[cfg(not(target_arch = "wasm32"))]
            bus: info.bus_id().to_owned(),
            #[cfg(not(target_arch = "wasm32"))]
            ports: info.port_chain().to_vec(),
        }
    }
    fn matches(&self, info: &DeviceDescriptor, firmware: bool) -> bool {
        if info.firmware_loaded != firmware {
            return false;
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let same_port = self.bus == info.bus_id() && self.ports == info.port_chain();
            #[cfg(target_os = "linux")]
            let same_port = same_port
                || self
                    .connector
                    .as_ref()
                    .is_some_and(|key| info.physical_port_key().as_ref() == Some(key));
            same_port
                && (!firmware
                    || self
                        .serial
                        .as_ref()
                        .is_none_or(|s| info.serial_number.as_ref() == Some(s)))
        }
        #[cfg(target_arch = "wasm32")]
        {
            self.serial
                .as_ref()
                .is_some_and(|s| info.serial_number.as_ref() == Some(s))
        }
    }
}

#[derive(Default)]
struct SampleBuffer {
    samples: Vec<Complex32>,
    offset: usize,
}
impl SampleBuffer {
    fn clear(&mut self) {
        self.samples.clear();
        self.offset = 0;
    }
    fn copy(&mut self, output: &mut [Complex32]) -> usize {
        let count = output.len().min(self.samples.len() - self.offset);
        output[..count].copy_from_slice(&self.samples[self.offset..self.offset + count]);
        self.offset += count;
        if self.offset == self.samples.len() {
            self.clear();
        }
        count
    }
}
/// Owned RX queue. Close explicitly to observe cleanup errors.
pub struct RxStream {
    shared: Owner<Shared>,
    endpoint: Option<Endpoint>,
    buffer: SampleBuffer,
    pending_error: Option<Error>,
    sequence: RxSequenceState,
    running: bool,
    failed: bool,
    submitted: bool,
    claimed: bool,
    stats: StreamingStats,
}
impl RxStream {
    pub fn stats(&self) -> StreamingStats {
        self.stats
    }
    pub fn start(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        operation(self.start_async())
    }
    async fn start_async(&mut self) -> Result<()> {
        if self.failed {
            return Err(Error::StreamClosed);
        }
        if self.running {
            return Ok(());
        }
        self.buffer.clear();
        self.pending_error = None;
        self.sequence = RxSequenceState::Synchronizing;
        let mut radio = self.shared.radio.lock().await;
        self.shared.active()?;
        let radio = radio.as_mut().ok_or(Error::Shutdown)?;
        // Retire completed stale packets before resetting the framer sequence.
        let ep = self.endpoint.as_mut().ok_or(Error::StreamClosed)?;
        while ep.pending() > 0 {
            match futures_lite::future::poll_once(ep.next_complete()).await {
                Some(_) => {}
                None => break,
            }
        }
        while ep.pending() < QUEUE_DEPTH {
            ep.submit(Buffer::new(TRANSFER_BYTES));
        }
        self.submitted = true;
        self.failed = true; // A cancelled partial start requires cleanup.
        radio.start().await?;
        self.failed = false;
        self.running = true;
        Ok(())
    }
    pub fn stop(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        operation(self.stop_async())
    }
    async fn stop_async(&mut self) -> Result<()> {
        self.running = false;
        self.buffer.clear();
        self.pending_error = None;
        let mut guard = self.shared.radio.lock().await;
        guard.as_mut().ok_or(Error::Shutdown)?.stop().await
    }
    /// Return available samples promptly; retain the unread part of a packet.
    /// After the first samples, include up to four already-completed packets
    /// without waiting for another USB transfer. Short reads remain valid.
    /// A timeout leaves pending transfers queued and the stream usable.
    pub fn read<'a>(
        &'a mut self,
        output: &'a mut [Complex32],
        timeout: Option<Duration>,
    ) -> impl MaybeFuture<Output = Result<usize>> + 'a {
        operation(self.read_async(output, timeout))
    }
    async fn read_async(
        &mut self,
        output: &mut [Complex32],
        timeout: Option<Duration>,
    ) -> Result<usize> {
        if !self.running || self.failed {
            return Err(Error::StreamClosed);
        }
        if output.is_empty() {
            return Ok(0);
        }
        if let Some(error) = self.pending_error.take() {
            self.receive_error(error).await?;
        }
        let mut count = self.buffer.copy(output);
        let mut packets = 0;
        let start = web_time::Instant::now();
        while count < output.len() && (count == 0 || packets < READ_BATCH_PACKETS) {
            let remaining = timeout
                .map(|t| t.saturating_sub(start.elapsed()))
                .unwrap_or(Duration::from_secs(86400));
            let ep = self.endpoint.as_mut().ok_or(Error::StreamClosed)?;
            let completion = if count == 0 {
                ep.complete(remaining).await
            } else {
                // Never suspend after copying samples into the caller's buffer.
                // Returning now also keeps low-rate streams responsive.
                let Some(completion) = futures_lite::future::poll_once(ep.next_complete()).await
                else {
                    break;
                };
                Ok(completion)
            };
            let completion = match completion {
                Ok(c) => c,
                Err(Error::Timeout) if timeout.is_none() => continue,
                Err(e) => return Err(e),
            };
            if let Err(error) = completion.status {
                if count != 0 {
                    self.pending_error = Some(error.into());
                    break;
                }
                self.receive_error(error.into()).await?;
            }
            packets += 1;
            self.stats.transfers += 1;
            let bytes = &completion.buffer[..completion.actual_len];
            let decoded = self.decode(bytes, &mut output[count..]);
            let mut buffer = completion.buffer;
            buffer.clear();
            buffer.set_requested_len(TRANSFER_BYTES);
            self.endpoint.as_mut().unwrap().submit(buffer);
            match decoded {
                Ok(n) => count += n,
                Err(error) if count != 0 => {
                    // Deliver the samples already copied. Report/recover the
                    // following error on the next read, without losing that data
                    // or suspending halfway through an overflow restart.
                    self.pending_error = Some(error);
                    break;
                }
                Err(error) => self.receive_error(error).await?,
            }
            if timeout.is_some_and(|t| start.elapsed() >= t) {
                if count == 0 {
                    return Err(Error::Timeout);
                }
                break;
            }
        }
        self.stats.samples += count as u64;
        Ok(count)
    }
    async fn receive_error(&mut self, error: Error) -> Result<()> {
        match error {
            Error::DeviceReceiveOverflow { .. } => {
                self.stats.overflows += 1;
                log::warn!(
                    "UHD RX FIFO overflow; restarting reception ({} total)",
                    self.stats.overflows
                );
                self.running = false;
                self.start_async().await
            }
            Error::ReceiveOverflow { expected, actual } => {
                self.stats.overflows += 1;
                log::warn!(
                    "UHD RX packet loss: expected sequence {expected}, got {actual} ({} overflows total)",
                    self.stats.overflows
                );
                Ok(())
            }
            error => {
                self.failed = true;
                self.running = false;
                Err(error)
            }
        }
    }
    fn decode(&mut self, bytes: &[u8], output: &mut [Complex32]) -> Result<usize> {
        use b2xx::rx::{
            RX_CONTEXT_OVERFLOW, RX_DATA_STREAM_ID, receive_context_code, sc16_samples,
        };
        let packet = crate::chdr::parse(bytes)?;
        if packet.stream_id != RX_DATA_STREAM_ID {
            return Err(Error::Chdr(format!(
                "unexpected receive stream ID 0x{:08x}",
                packet.stream_id
            )));
        }
        if packet.context {
            let code = receive_context_code(packet.payload)?;
            return Err(if code == RX_CONTEXT_OVERFLOW {
                Error::DeviceReceiveOverflow {
                    sequence: packet.sequence,
                }
            } else {
                Error::ReceiveContext {
                    code,
                    sequence: packet.sequence,
                }
            });
        }
        match self.sequence.observe(packet.sequence) {
            SequenceDisposition::Discard => Ok(0),
            SequenceDisposition::Overflow { expected, actual } => {
                Err(Error::ReceiveOverflow { expected, actual })
            }
            SequenceDisposition::Accept => {
                let samples = sc16_samples(
                    packet.payload,
                    f32::from_bits(self.shared.scale.load(Ordering::Relaxed)),
                )?;
                let n = samples.len();
                if n <= output.len() {
                    // Decode straight into the caller's stream buffer. Only
                    // partial packets need the intermediate sample buffer.
                    for (out, sample) in output.iter_mut().zip(samples) {
                        *out = sample;
                    }
                    Ok(n)
                } else {
                    self.buffer.samples.clear();
                    self.buffer.samples.extend(samples);
                    Ok(self.buffer.copy(output))
                }
            }
        }
    }
    /// Consuming, owned and lazy. Dropping this operation also attempts cleanup.
    pub fn close(mut self) -> impl MaybeFuture<Output = Result<StreamingStats>> + 'static {
        operation(async move {
            self.cleanup().await?;
            Ok(self.stats)
        })
    }
    async fn cleanup(&mut self) -> Result<()> {
        if !self.claimed {
            return Ok(());
        }
        let mut radio = self.shared.radio.lock().await;
        let receiver = radio.as_mut().ok_or(Error::Shutdown)?;
        receiver.stop().await?;
        if self.endpoint.is_some() {
            #[cfg(not(target_arch = "wasm32"))]
            {
                let endpoint = self.endpoint.as_mut().unwrap();
                endpoint.cancel_all();
                let deadline = web_time::Instant::now() + CLEANUP_TIMEOUT;
                while endpoint.pending() > 0 {
                    endpoint
                        .complete(deadline.saturating_duration_since(web_time::Instant::now()))
                        .await?;
                }
                receiver.return_endpoint(self.endpoint.take().unwrap());
            }
            #[cfg(target_arch = "wasm32")]
            {
                if self.submitted {
                    self.shared.life.lock().unwrap().reopen = true;
                } else {
                    receiver.return_endpoint(self.endpoint.take().unwrap());
                }
            }
        }
        self.shared.life.lock().unwrap().claimed = false;
        self.claimed = false;
        Ok(())
    }
}
impl Drop for RxStream {
    fn drop(&mut self) {
        if !self.claimed {
            return;
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let failed =
                crate::operation::blocking(bounded(self.cleanup(), CLEANUP_TIMEOUT)).is_err();
            let mut life = self.shared.life.lock().unwrap();
            life.claimed = false;
            life.terminal |= failed;
            self.claimed = false;
        }
        #[cfg(target_arch = "wasm32")]
        {
            // Move ownership into the task; the final owner cannot close USB early.
            let mut cleanup = Self {
                shared: self.shared.clone(),
                endpoint: self.endpoint.take(),
                buffer: SampleBuffer::default(),
                pending_error: None,
                sequence: self.sequence,
                running: self.running,
                failed: self.failed,
                submitted: self.submitted,
                claimed: true,
                stats: self.stats,
            };
            self.claimed = false;
            wasm_bindgen_futures::spawn_local(async move {
                let failed = bounded(cleanup.cleanup(), CLEANUP_TIMEOUT).await.is_err();
                let mut life = cleanup.shared.life.lock().unwrap();
                life.claimed = false;
                life.terminal |= failed;
                life.reopen |= cleanup.submitted;
                drop(life);
                cleanup.claimed = false;
                drop(cleanup);
            });
        }
    }
}
#[cfg(all(test, target_arch = "wasm32"))]
mod browser_tests;
#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests;
