use std::time::Duration;

use nusb::transfer::{Buffer, Bulk, In, Out};

use crate::{Error, Result, chdr};

const DATA_OUT_INTERFACE: u8 = 1;
const DATA_IN_INTERFACE: u8 = 2;
const CONTROL_OUT_INTERFACE: u8 = 3;
const CONTROL_IN_INTERFACE: u8 = 4;
const DATA_OUT_ENDPOINT: u8 = 0x02;
const DATA_IN_ENDPOINT: u8 = 0x86;
const CONTROL_OUT_ENDPOINT: u8 = 0x04;
const CONTROL_IN_ENDPOINT: u8 = 0x88;
const CONTROL_RESPONSE_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(target_arch = "wasm32")]
// Preposted receives drain acknowledgements while the FPGA's SPI-ready signal
// backpressures incoming commands. 64 writes amortize browser completion latency.
const CONTROL_WRITE_WINDOW: usize = 64;

/// Well-known B2xx CHDR stream identifiers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StreamId {
    LocalControl,
    RadioControl(u8),
    TxData(u8),
    RxData(u8),
    GpsUart,
}

impl StreamId {
    #[must_use]
    pub const fn value(self) -> u32 {
        match self {
            Self::LocalControl => 0x0000_0040,
            Self::RadioControl(0) => 0x0000_0010,
            Self::RadioControl(_) => 0x0000_0020,
            Self::TxData(0) => 0x0000_0050,
            Self::TxData(_) => 0x0000_0060,
            Self::RxData(0) => 0x0000_00a0,
            Self::RxData(_) => 0x0000_00b0,
            Self::GpsUart => 0x0000_0030,
        }
    }

    #[must_use]
    pub const fn response_value(self) -> u32 {
        self.value().rotate_left(16)
    }
}

/// Exclusive raw access to the two control and two sample bulk endpoints.
pub struct B2xxTransport {
    control_in: nusb::Endpoint<Bulk, In>,
    control_out: nusb::Endpoint<Bulk, Out>,
    data_in: Option<nusb::Endpoint<Bulk, In>>,
    data_out: nusb::Endpoint<Bulk, Out>,
}

impl std::fmt::Debug for B2xxTransport {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("B2xxTransport")
            .field("control_in_packet_size", &self.control_in.max_packet_size())
            .field(
                "data_in_packet_size",
                &self.data_in.as_ref().map(|ep| ep.max_packet_size()),
            )
            .finish_non_exhaustive()
    }
}

impl B2xxTransport {
    pub(crate) async fn open(device: &nusb::Device) -> Result<Self> {
        let data_out_interface =
            crate::operation::usb(device.detach_and_claim_interface(DATA_OUT_INTERFACE)).await?;
        let data_in_interface =
            crate::operation::usb(device.detach_and_claim_interface(DATA_IN_INTERFACE)).await?;
        let control_out_interface =
            crate::operation::usb(device.detach_and_claim_interface(CONTROL_OUT_INTERFACE)).await?;
        let control_in_interface =
            crate::operation::usb(device.detach_and_claim_interface(CONTROL_IN_INTERFACE)).await?;

        Ok(Self {
            data_out: data_out_interface.endpoint::<Bulk, Out>(DATA_OUT_ENDPOINT)?,
            data_in: Some(data_in_interface.endpoint::<Bulk, In>(DATA_IN_ENDPOINT)?),
            control_out: control_out_interface.endpoint::<Bulk, Out>(CONTROL_OUT_ENDPOINT)?,
            control_in: control_in_interface.endpoint::<Bulk, In>(CONTROL_IN_ENDPOINT)?,
        })
    }

    /// Send one complete transfer to the FPGA control path.
    pub async fn send_control(&mut self, bytes: Vec<u8>) -> Result<()> {
        transfer_out(&mut self.control_out, bytes).await
    }

    /// Receive from the FPGA control path.
    pub async fn receive_control(&mut self) -> Result<Vec<u8>> {
        let size = self.control_in.max_packet_size() * 2;
        transfer_in(&mut self.control_in, size).await
    }

    fn cancel_control_receive(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        self.control_in.cancel_all();
    }

    /// Send CHDR sample data to the FPGA.
    pub async fn send_data(&mut self, bytes: Vec<u8>) -> Result<()> {
        transfer_out(&mut self.data_out, bytes).await
    }

    /// Receive CHDR sample data from the FPGA.
    ///
    /// `nusb` 0.2 requires `requested_length` to be a multiple of the endpoint
    /// packet size. B2xx's established optimal receive lengths (8176 or 16360)
    /// intentionally are not multiples, so callers must currently choose an
    /// aligned size here. See the repository README for the resulting streaming
    /// limitation.
    pub async fn receive_data(&mut self, requested_length: usize) -> Result<Vec<u8>> {
        let endpoint = self.data_in.as_mut().ok_or(Error::Busy)?;
        let packet_size = endpoint.max_packet_size();
        if requested_length == 0 || !requested_length.is_multiple_of(packet_size) {
            return Err(Error::InvalidArgument(format!(
                "nusb requires the IN length to be a nonzero multiple of endpoint packet size {packet_size}"
            )));
        }
        transfer_in(endpoint, requested_length).await
    }

    pub(crate) fn take_sample_endpoint(&mut self) -> Result<nusb::Endpoint<Bulk, In>> {
        self.data_in.take().ok_or(Error::Busy)
    }
    pub(crate) fn return_sample_endpoint(&mut self, endpoint: nusb::Endpoint<Bulk, In>) {
        self.data_in = Some(endpoint);
    }

    #[must_use]
    pub const fn into_radio_control(self, stream: StreamId) -> RadioControl {
        RadioControl {
            transport: self,
            stream,
            sequence: 0,
            command_time: None,
            #[cfg(target_arch = "wasm32")]
            poisoned: false,
        }
    }
}

/// Ordered Wishbone register transactions over the B2xx control stream.
pub struct RadioControl {
    transport: B2xxTransport,
    stream: StreamId,
    sequence: u16,
    command_time: Option<u64>,
    #[cfg(target_arch = "wasm32")]
    poisoned: bool,
}

impl RadioControl {
    /// Consecutive settings writes, fully acknowledged before returning.
    pub(crate) async fn poke32_many(&mut self, writes: &[(u32, u32)]) -> Result<()> {
        if writes.iter().any(|(address, _)| !address.is_multiple_of(4)) {
            return Err(Error::InvalidArgument(
                "poke32 address must be 4-byte aligned".into(),
            ));
        }
        #[cfg(not(target_arch = "wasm32"))]
        for &(address, value) in writes {
            self.poke32(address, value).await?;
        }
        #[cfg(target_arch = "wasm32")]
        {
            self.poke32_pipeline(writes, CONTROL_WRITE_WINDOW).await?;
        }
        Ok(())
    }

    #[cfg(target_arch = "wasm32")]
    async fn poke32_pipeline(&mut self, writes: &[(u32, u32)], window: usize) -> Result<()> {
        if self.poisoned {
            return Err(Error::ReopenRequired);
        }
        if writes.is_empty() {
            return Ok(());
        }
        assert!(window > 0);
        let response_size = self.transport.control_in.max_packet_size() * 2;
        let mut pending = std::collections::VecDeque::with_capacity(window);
        let mut submitted = 0;
        let profiling = log::log_enabled!(log::Level::Debug);
        let clock = || profiling.then(web_time::Instant::now);
        let elapsed = |start: Option<web_time::Instant>| {
            start.map_or(Duration::ZERO, |start| start.elapsed())
        };
        let started = clock();
        let mut submission_time = Duration::ZERO;
        let mut out_latency = Duration::ZERO;
        let mut ack_latency = Duration::ZERO;
        // Cancellation or a failed transfer leaves WebUSB transfers queued.
        // Further I/O requires reopening unless every acknowledgement is drained.
        self.poisoned = true;
        while submitted < writes.len() || !pending.is_empty() {
            let submitting = clock();
            while submitted < writes.len() && pending.len() < window {
                let (address, value) = writes[submitted];
                let sequence = self.sequence & 0xfff;
                self.sequence = self.sequence.wrapping_add(1) & 0xfff;
                let packet = chdr::encode_control(
                    self.stream.value(),
                    sequence,
                    address / 4,
                    value,
                    self.command_time,
                );
                let sent = clock();
                pending.push_back((sequence, packet.len(), sent));
                self.transport.control_in.submit(Buffer::new(response_size));
                self.transport.control_out.submit(packet.into());
                submitted += 1;
            }
            submission_time += elapsed(submitting);
            let (sequence, length, sent) = pending.pop_front().expect("submitted control write");
            let transport = &mut self.transport;
            // Poll both directions: an ACK need not wait for the OUT callback.
            let (out_elapsed, (response, ack_elapsed)) = futures_lite::future::try_zip(
                async {
                    let completion = crate::operation::completion(
                        &mut transport.control_out,
                        CONTROL_RESPONSE_TIMEOUT,
                    )
                    .await?;
                    completion.status?;
                    if completion.actual_len != length {
                        return Err(Error::ShortTransfer {
                            expected: length,
                            actual: completion.actual_len,
                        });
                    }
                    Ok(elapsed(sent))
                },
                async {
                    let response = transfer_in(&mut transport.control_in, response_size).await?;
                    Ok((response, elapsed(sent)))
                },
            )
            .await?;
            out_latency += out_elapsed;
            ack_latency += ack_elapsed;
            let packet = chdr::parse(&response)?;
            if packet.stream_id != self.stream.response_value()
                || packet.sequence != sequence
                || !packet.context
                || packet.payload.len() != 8
            {
                return Err(Error::Chdr(
                    "unexpected register pipeline acknowledgement".into(),
                ));
            }
            // Refill immediately instead of draining the entire window.
        }
        self.poisoned = false;
        log::debug!(
            "Control pipeline: {} writes, window {}, total {:.3}s, submit {:.3}s, observed OUT {:.2}ms/ACK {:.2}ms average",
            writes.len(),
            window,
            elapsed(started).as_secs_f64(),
            submission_time.as_secs_f64(),
            out_latency.as_secs_f64() * 1000.0 / writes.len() as f64,
            ack_latency.as_secs_f64() * 1000.0 / writes.len() as f64,
        );
        Ok(())
    }

    #[must_use]
    pub const fn stream(&self) -> StreamId {
        self.stream
    }

    pub fn set_stream(&mut self, stream: StreamId) {
        self.stream = stream;
        self.sequence = 0;
    }

    pub fn set_command_time(&mut self, ticks: Option<u64>) {
        self.command_time = ticks;
    }

    #[must_use]
    pub const fn transport_mut(&mut self) -> &mut B2xxTransport {
        &mut self.transport
    }

    #[must_use]
    pub fn into_transport(self) -> B2xxTransport {
        self.transport
    }

    pub async fn poke32(&mut self, byte_address: u32, value: u32) -> Result<()> {
        if !byte_address.is_multiple_of(4) {
            return Err(Error::InvalidArgument(
                "poke32 address must be 4-byte aligned".into(),
            ));
        }
        self.transaction(byte_address / 4, value).await.map(|_| ())
    }

    pub async fn peek32(&mut self, byte_address: u32) -> Result<u32> {
        if !byte_address.is_multiple_of(4) {
            return Err(Error::InvalidArgument(
                "peek32 address must be 4-byte aligned".into(),
            ));
        }
        let value = self.transaction(32, byte_address / 8).await?;
        let words = value.to_le_bytes();
        Ok(if (byte_address / 4) & 1 == 0 {
            u32::from_le_bytes([words[0], words[1], words[2], words[3]])
        } else {
            u32::from_le_bytes([words[4], words[5], words[6], words[7]])
        })
    }

    pub async fn peek64(&mut self, byte_address: u32) -> Result<u64> {
        if !byte_address.is_multiple_of(8) {
            return Err(Error::InvalidArgument(
                "peek64 address must be 8-byte aligned".into(),
            ));
        }
        self.transaction(32, byte_address / 8).await
    }

    async fn transaction(&mut self, address: u32, value: u32) -> Result<u64> {
        #[cfg(target_arch = "wasm32")]
        if self.poisoned {
            return Err(Error::ReopenRequired);
        }
        let sequence = self.sequence & 0x0fff;
        self.sequence = self.sequence.wrapping_add(1) & 0x0fff;
        let request = chdr::encode_control(
            self.stream.value(),
            sequence,
            address,
            value,
            self.command_time,
        );
        self.transport.send_control(request).await?;

        let deadline = web_time::Instant::now() + CONTROL_RESPONSE_TIMEOUT;
        loop {
            let remaining = deadline.saturating_duration_since(web_time::Instant::now());
            let response = crate::operation::bounded(self.transport.receive_control(), remaining)
                .await
                .map_err(|error| {
                    if matches!(error, Error::Timeout) {
                        Error::ControlTimeout {
                            timeout: CONTROL_RESPONSE_TIMEOUT,
                        }
                    } else {
                        error
                    }
                });
            let response = match response {
                Ok(response) => response,
                Err(error @ Error::ControlTimeout { .. }) => {
                    self.transport.cancel_control_receive();
                    return Err(error);
                }
                Err(error) => return Err(error),
            };
            let packet = chdr::parse(&response)?;
            if packet.stream_id != self.stream.response_value() || packet.sequence != sequence {
                continue;
            }
            if !packet.context || packet.payload.len() != 8 {
                return Err(Error::Chdr(
                    "register response is not an eight-byte context payload".into(),
                ));
            }
            let high = u64::from(u32::from_le_bytes(
                packet.payload[0..4]
                    .try_into()
                    .expect("four-byte high word"),
            ));
            let low = u64::from(u32::from_le_bytes(
                packet.payload[4..8].try_into().expect("four-byte low word"),
            ));
            return Ok((high << 32) | low);
        }
    }
}

async fn transfer_out(endpoint: &mut nusb::Endpoint<Bulk, Out>, bytes: Vec<u8>) -> Result<()> {
    let expected = bytes.len();
    endpoint.submit(bytes.into());
    let completion = crate::operation::completion(endpoint, CONTROL_RESPONSE_TIMEOUT).await?;
    completion.status?;
    if completion.actual_len != expected {
        return Err(Error::ShortTransfer {
            expected,
            actual: completion.actual_len,
        });
    }
    Ok(())
}

async fn transfer_in(
    endpoint: &mut nusb::Endpoint<Bulk, In>,
    requested_length: usize,
) -> Result<Vec<u8>> {
    if endpoint.pending() == 0 {
        endpoint.submit(Buffer::new(requested_length));
    }
    let completion = crate::operation::completion(endpoint, CONTROL_RESPONSE_TIMEOUT).await?;
    completion.status?;
    Ok(completion.buffer.into_vec())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stream_ids_match_b2xx_routing() {
        assert_eq!(StreamId::LocalControl.value(), 0x40);
        assert_eq!(StreamId::LocalControl.response_value(), 0x0040_0000);
        assert_eq!(StreamId::RadioControl(1).value(), 0x20);
        assert_eq!(StreamId::RxData(0).value(), 0xa0);
    }
}

#[cfg(all(test, target_arch = "wasm32"))]
mod browser_tests {
    use super::*;
    use wasm_bindgen::prelude::*;
    use wasm_bindgen_test::*;

    #[wasm_bindgen(inline_js = "
export function install_register_usb(failure) {
 const descriptor = [9,2,73,0,4,1,0,128,50];
 for (const [i, ep] of [[1,2],[2,0x86],[3,4],[4,0x88]]) {
  descriptor.push(9,4,i,0,1,255,0,0,0,7,5,ep,2,0,4,0);
 }
 const cfg = {configurationValue:1, interfaces:[1,2,3,4].map(i=>({interfaceNumber:i,claimed:false,alternates:[{alternateSetting:0,interfaceClass:255,interfaceSubclass:0,interfaceProtocol:0,interfaceName:'control'}]}))};
 const d = {vendorId:0x2500,productId:0x20,serialNumber:'register-test',
  deviceVersionMajor:1,deviceVersionMinor:0,deviceVersionSubminor:0,
  usbVersionMajor:3,usbVersionMinor:0,usbVersionSubminor:0,deviceClass:0,deviceSubclass:0,deviceProtocol:0,
  manufacturerName:'Ettus Research LLC',productName:'B210',configuration:cfg,configurations:[cfg],
  opened:true,waiting:[],active:0,peak:0,requests:[],refilled:false,
  open:()=>Promise.resolve(),claimInterface:()=>Promise.resolve(),releaseInterface:()=>Promise.resolve(),
  close:()=>Promise.resolve(),
  controlTransferIn:(setup, length)=> {
   const bytes = (setup.value>>8)===1 ? [18,1,0,3,0,0,0,9,0,0x25,0x20,0,0,0,0,0,0,1] : descriptor;
   return Promise.resolve({status:'ok',data:new DataView(new Uint8Array(bytes.slice(0,length)).buffer)});
  },
  transferIn:(ep, length)=>new Promise(resolve=>d.waiting.push(resolve)),
  transferOut:(ep, data)=> {
   if (ep!==4) throw new Error('unexpected endpoint');
   const packet = new DataView(data.buffer,data.byteOffset,data.byteLength);
   const sequence=(packet.getUint32(0,true)>>16)&4095;
   const resolveIn=d.waiting.shift();
   const index=d.requests.length;
   if (index===3 && d.active>0) d.refilled=true;
   d.requests.push([sequence,packet.getUint32(data.byteLength-8,true),packet.getUint32(data.byteLength-4,true)]);
   d.active++; d.peak=Math.max(d.peak,d.active);
   if (d.active>64) throw new Error('control response queue overflow');
   return new Promise(resolve=>setTimeout(()=> {
    d.active--;
    const response = new DataView(new ArrayBuffer(16));
    response.setUint32(0,0x80000010|(((sequence+(failure===1?1:0))&4095)<<16),true);
    const sid=packet.getUint32(4,true);
    response.setUint32(4,((sid<<16)|(sid>>>16))+(failure===4?1:0),true);
    if (failure===5) response.setUint32(0,0x8000000c|(sequence<<16),true);
    response.setUint32(12,0x12345678,true);
    const result={status:'ok',data:response};
    if (resolveIn) resolveIn(result);
    else d.waiting.push(result);
    resolve({status:'ok',bytesWritten:data.byteLength-(failure===2?1:0)});
   },failure===3 && (index===1 || index===2) ? 30 : 2));
  }
 };
 // Scalar transactions submit their IN transfer after their OUT completes.
 const transferIn=d.transferIn;
 d.transferIn=(ep,length)=> {
  if (d.waiting.length && typeof d.waiting[0]!=='function') return Promise.resolve(d.waiting.shift());
  return transferIn(ep,length);
 };
 Object.setPrototypeOf(d,USBDevice.prototype);
 Object.defineProperty(navigator,'usb',{configurable:true,value:{getDevices:()=>Promise.resolve([d])}});
 return d;
}
export function register_peak(d) { return d.peak; }
export function register_requests(d) { return new Uint32Array(d.requests.flat()); }
export function register_pending(d) { return d.waiting.length+d.active; }
export function register_refilled(d) { return d.refilled; }
")]
    extern "C" {
        fn install_register_usb(failure: u32) -> web_sys::UsbDevice;
        fn register_peak(device: &web_sys::UsbDevice) -> u32;
        fn register_requests(device: &web_sys::UsbDevice) -> js_sys::Uint32Array;
        fn register_pending(device: &web_sys::UsbDevice) -> u32;
        fn register_refilled(device: &web_sys::UsbDevice) -> bool;
    }

    async fn control() -> (RadioControl, super::super::B2xxDevice) {
        let info = crate::Device::list().await.unwrap().pop().unwrap();
        let device = info.open().await.unwrap();
        let control = device
            .open_transport()
            .await
            .unwrap()
            .into_radio_control(StreamId::LocalControl);
        (control, device)
    }

    #[wasm_bindgen_test]
    async fn register_pipeline_is_bounded_ordered_and_drained_before_readback() {
        for window in [1, 3, 8, 16, 32, 64] {
            let device = install_register_usb(0);
            let (mut control, _owner) = control().await;
            control.sequence = 4094;
            let writes = (0..70).map(|i| (i * 4, i + 100)).collect::<Vec<_>>();
            control.poke32_pipeline(&writes, window).await.unwrap();
            assert_eq!(register_peak(&device), window as u32);
            assert_eq!(register_pending(&device), 0);
            let expected = (0..70)
                .flat_map(|i| [(4094 + i) & 4095, i, i + 100])
                .collect::<Vec<_>>();
            assert_eq!(register_requests(&device).to_vec(), expected);
            assert_eq!(control.peek32(0).await.unwrap(), 0x12345678);
            assert_eq!(register_pending(&device), 0);
        }
    }

    #[wasm_bindgen_test]
    async fn register_pipeline_refills_before_the_old_window_drains() {
        let device = install_register_usb(3);
        let (mut control, _owner) = control().await;
        control.poke32_pipeline(&[(0, 1); 10], 3).await.unwrap();
        assert!(register_refilled(&device));
        assert_eq!(register_pending(&device), 0);
    }

    #[wasm_bindgen_test]
    async fn failed_register_batch_requires_reopen() {
        for failure in [1, 2, 4, 5] {
            install_register_usb(failure);
            let (mut control, _owner) = control().await;
            assert!(
                control
                    .poke32_many(&[(0, 1), (4, 2), (8, 3)])
                    .await
                    .is_err()
            );
            assert!(matches!(
                control.peek32(0).await,
                Err(Error::ReopenRequired)
            ));
        }
    }

    #[wasm_bindgen_test]
    async fn cancelled_register_batch_requires_reopen() {
        install_register_usb(0);
        let (mut control, _owner) = control().await;
        assert!(
            futures_lite::future::poll_once(control.poke32_many(&[(0, 1), (4, 2)]))
                .await
                .is_none()
        );
        assert!(matches!(
            control.peek32(0).await,
            Err(Error::ReopenRequired)
        ));
    }

    #[wasm_bindgen_test]
    async fn ad9361_batch_preserves_spi_commands_and_cache() {
        let device = install_register_usb(0);
        let (mut control, _owner) = control().await;
        let spi = super::super::B2xxSpi::new(&mut control);
        let mut io = super::super::Ad9361Io::new(spi);
        io.write_registers(&[(0x123, 0x5a), (0x123, 0x5a)])
            .await
            .unwrap();
        io.write_registers(&[(0x321, 0xa5)]).await.unwrap();
        assert_eq!(
            register_requests(&device).to_vec(),
            vec![
                0, 8, 49, 1, 9, 0x98000001, 2, 10, 0x81235a00, 3, 10, 0x81235a00, 4, 10,
                0x8321a500,
            ]
        );
        assert_eq!(register_pending(&device), 0);
    }
}
