//! USB substitution boundary for lifecycle tests. Radio algorithms stay in b2xx.
use super::*;
use nusb::transfer::Completion;
pub(super) enum Radio {
    Hardware(Box<B2xxReceiver>),
    #[cfg(test)]
    Fake(FakeRadio),
}
impl Radio {
    pub async fn open(device: b2xx::B2xxDevice, config: RxConfig) -> Result<Self> {
        Ok(Self::Hardware(Box::new(
            B2xxReceiver::open(device, config).await?,
        )))
    }
    pub fn host_scale(&self) -> f32 {
        match self {
            Self::Hardware(r) => r.host_scale(),
            #[cfg(test)]
            Self::Fake(_) => 1.0,
        }
    }
    pub fn take_endpoint(&mut self) -> Result<Endpoint> {
        match self {
            Self::Hardware(r) => Ok(Endpoint::Hardware(
                r.transport_mut().take_sample_endpoint()?,
            )),
            #[cfg(test)]
            Self::Fake(r) => Ok(Endpoint::Fake(r.endpoint.take().ok_or(Error::Busy)?)),
        }
    }
    pub fn return_endpoint(&mut self, ep: Endpoint) {
        match (self, ep) {
            (Self::Hardware(r), Endpoint::Hardware(ep)) => {
                r.transport_mut().return_sample_endpoint(ep)
            }
            #[cfg(test)]
            (Self::Fake(r), Endpoint::Fake(ep)) => r.endpoint = Some(ep),
            #[cfg(test)]
            _ => unreachable!(),
        }
    }
    pub async fn start(&mut self) -> Result<()> {
        match self {
            Self::Hardware(r) => r.start().await,
            #[cfg(test)]
            Self::Fake(r) => {
                r.running = true;
                r.io.event("start").await
            }
        }
    }
    pub async fn stop(&mut self) -> Result<()> {
        match self {
            Self::Hardware(r) => r.stop().await,
            #[cfg(test)]
            Self::Fake(r) => {
                if r.running {
                    r.io.event("stop").await?;
                    r.running = false;
                }
                Ok(())
            }
        }
    }
    pub async fn terminal_close(&mut self) -> Result<()> {
        match self {
            Self::Hardware(r) => r.terminal_close().await,
            #[cfg(test)]
            Self::Fake(r) => r.io.event("close").await,
        }
    }
    pub async fn tune(&mut self, request: RxTuneRequest) -> Result<RxTuneResult> {
        match self {
            Self::Hardware(r) => r.tune(request).await,
            #[cfg(test)]
            Self::Fake(r) => {
                r.io.event("tune").await?;
                Ok(RxTuneResult {
                    requested_center_frequency_hz: request.center_frequency_hz,
                    target_rf_frequency_hz: request.center_frequency_hz,
                    actual_rf_frequency_hz: request.center_frequency_hz,
                    target_dsp_frequency_hz: 0.0,
                    actual_dsp_frequency_hz: 0.0,
                    actual_center_frequency_hz: request.center_frequency_hz,
                })
            }
        }
    }
    pub async fn set_sample_rate(&mut self, hz: f64) -> Result<f64> {
        match self {
            Self::Hardware(r) => r.set_sample_rate(hz).await,
            #[cfg(test)]
            Self::Fake(r) => {
                r.io.event("rate").await?;
                Ok(hz)
            }
        }
    }
    pub async fn set_gain(&mut self, gain: RxGain) -> Result<()> {
        match self {
            Self::Hardware(r) => r.set_gain(gain).await,
            #[cfg(test)]
            Self::Fake(r) => r.io.event("gain").await,
        }
    }
}
pub(super) enum Endpoint {
    Hardware(nusb::Endpoint<Bulk, In>),
    #[cfg(test)]
    Fake(FakeEndpoint),
}
impl Endpoint {
    pub fn submit(&mut self, buffer: Buffer) {
        match self {
            Self::Hardware(ep) => ep.submit(buffer),
            #[cfg(test)]
            Self::Fake(ep) => {
                assert_eq!(buffer.requested_len(), TRANSFER_BYTES);
                ep.pending += 1;
                ep.io.0.lock().unwrap().submitted += 1;
            }
        }
    }
    pub fn pending(&self) -> usize {
        match self {
            Self::Hardware(ep) => ep.pending(),
            #[cfg(test)]
            Self::Fake(ep) => ep.pending,
        }
    }
    #[cfg(not(target_arch = "wasm32"))]
    pub fn cancel_all(&mut self) {
        match self {
            Self::Hardware(ep) => ep.cancel_all(),
            #[cfg(test)]
            Self::Fake(ep) => ep.cancelled = true,
        }
    }
    pub async fn next_complete(&mut self) -> Completion {
        match self {
            Self::Hardware(ep) => ep.next_complete().await,
            #[cfg(test)]
            Self::Fake(ep) => ep.next_complete().await,
        }
    }
    pub async fn complete(&mut self, timeout: Duration) -> Result<Completion> {
        match self {
            Self::Hardware(ep) => crate::operation::completion(ep, timeout).await,
            #[cfg(test)]
            Self::Fake(ep) => bounded(async { Ok(ep.next_complete().await) }, timeout).await,
        }
    }
}
#[cfg(test)]
#[derive(Clone, Default)]
pub(super) struct TestIo(pub Owner<Mutex<FakeState>>);
#[cfg(test)]
#[derive(Default)]
pub(super) struct FakeState {
    pub events: Vec<&'static str>,
    pub packets: std::collections::VecDeque<Result<Vec<u8>, nusb::transfer::TransferError>>,
    pub fail: Option<&'static str>,
    pub pause: Option<&'static str>,
    pub submitted: usize,
}
#[cfg(test)]
impl TestIo {
    async fn event(&self, event: &'static str) -> Result<()> {
        let (fail, pause) = {
            let mut state = self.0.lock().unwrap();
            state.events.push(event);
            (state.fail == Some(event), state.pause == Some(event))
        };
        if fail {
            return Err(Error::Timeout);
        }
        if pause {
            std::future::pending::<()>().await;
        }
        Ok(())
    }
    pub fn radio(&self) -> Radio {
        Radio::Fake(FakeRadio {
            io: self.clone(),
            running: false,
            endpoint: Some(FakeEndpoint {
                io: self.clone(),
                pending: 0,
                cancelled: false,
            }),
        })
    }
}
#[cfg(test)]
pub(super) struct FakeRadio {
    io: TestIo,
    running: bool,
    endpoint: Option<FakeEndpoint>,
}
#[cfg(test)]
pub(super) struct FakeEndpoint {
    io: TestIo,
    pending: usize,
    cancelled: bool,
}
#[cfg(test)]
impl FakeEndpoint {
    async fn next_complete(&mut self) -> Completion {
        std::future::poll_fn(|_| {
            let packet = if self.cancelled {
                Some(Err(nusb::transfer::TransferError::Cancelled))
            } else {
                self.io.0.lock().unwrap().packets.pop_front()
            };
            match packet {
                None => std::task::Poll::Pending,
                Some(packet) => {
                    assert!(self.pending > 0);
                    self.pending -= 1;
                    if self.pending == 0 {
                        self.cancelled = false;
                    }
                    let (bytes, status) = match packet {
                        Ok(b) => (b, Ok(())),
                        Err(e) => (Vec::new(), Err(e)),
                    };
                    let mut buffer = Buffer::new(TRANSFER_BYTES);
                    buffer.extend_from_slice(&bytes);
                    std::task::Poll::Ready(Completion {
                        actual_len: bytes.len(),
                        buffer,
                        status,
                    })
                }
            }
        })
        .await
    }
}
