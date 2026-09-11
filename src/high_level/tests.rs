use super::*;
use futures_lite::future::{block_on, poll_once};
use io::TestIo;
use std::future::IntoFuture;
fn device() -> (Device, TestIo) {
    let io = TestIo::default();
    (
        Device {
            shared: Owner::new(Shared {
                radio: AsyncMutex::new(Some(io.radio())),
                life: Mutex::new(Life::default()),
                scale: AtomicU32::new(1f32.to_bits()),
            }),
        },
        io,
    )
}
fn packet(io: &TestIo, seq: u16, samples: &[f32]) {
    let payload: Vec<u8> = samples
        .iter()
        .flat_map(|x| [x.to_le_bytes(), (-x).to_le_bytes()].concat())
        .collect();
    io.0.lock()
        .unwrap()
        .packets
        .push_back(Ok(crate::chdr::encode_data(
            0xa0, seq, &payload, None, false,
        )
        .unwrap()));
}
#[test]
fn claim_and_unpolled_close_own_device() {
    let (mut d, io) = device();
    drop(d.shutdown());
    let rx = d.rx_stream().unwrap();
    assert!(matches!(d.rx_stream(), Err(Error::Busy)));
    let close = rx.close();
    assert!(matches!(d.shutdown().wait(), Err(Error::Busy)));
    assert!(io.0.lock().unwrap().events.is_empty());
    drop(close);
    d.rx_stream().unwrap().close().wait().unwrap();
    d.shutdown().wait().unwrap();
}
#[test]
fn read_partial_buffers_and_no_control_lock_on_reads() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    assert_eq!(io.0.lock().unwrap().submitted, 16);
    packet(&io, 0, &[1., 2., 3.]);
    let control = d.shared.radio.try_lock().unwrap();
    let mut out = [Complex32::default(); 2];
    assert_eq!(
        rx.read(&mut out, Some(Duration::from_millis(10)))
            .wait()
            .unwrap(),
        2
    );
    assert_eq!(out[1], Complex32::new(2., -2.));
    assert_eq!(rx.read(&mut out, Some(Duration::ZERO)).wait().unwrap(), 1);
    assert_eq!(out[0], Complex32::new(3., -3.));
    assert_eq!(io.0.lock().unwrap().submitted, 17);
    drop(control);
    assert_eq!(rx.close().wait().unwrap().samples, 3);
    d.shutdown().wait().unwrap();
}
#[test]
fn pending_read_timeout_and_cancellation_reuse_queue() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    let mut out = [Complex32::default(); 2];
    assert!(matches!(
        rx.read(&mut out, Some(Duration::from_millis(1))).wait(),
        Err(Error::Timeout)
    ));
    let mut read = Box::pin(rx.read(&mut out, None).into_future());
    assert!(block_on(poll_once(read.as_mut())).is_none());
    drop(read);
    assert_eq!(io.0.lock().unwrap().submitted, 16);
    packet(&io, 0, &[7.]);
    assert_eq!(rx.read(&mut out, None).wait().unwrap(), 1);
    rx.close().wait().unwrap();
}
#[test]
fn stop_restart_discards_buffer_and_old_sequences() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    packet(&io, 0, &[1., 2.]);
    let mut out = [Complex32::default(); 1];
    rx.read(&mut out, None).wait().unwrap();
    rx.stop().wait().unwrap();
    assert!(matches!(
        rx.read(&mut out, None).wait(),
        Err(Error::StreamClosed)
    ));
    rx.start().wait().unwrap();
    packet(&io, 9, &[9.]);
    packet(&io, 0, &[3.]);
    rx.read(&mut out, None).wait().unwrap();
    assert_eq!(out[0].re, 3.);
    rx.close().wait().unwrap();
}
#[test]
fn sequence_loss_recovers_without_invalidating_stream() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    packet(&io, 0, &[1.]);
    let mut out = [Complex32::default(); 1];
    rx.read(&mut out, None).wait().unwrap();
    packet(&io, 2, &[2.]);
    packet(&io, 3, &[3.]);
    rx.read(&mut out, None).wait().unwrap();
    assert_eq!(out[0].re, 3.);
    assert_eq!(rx.stats().overflows, 1);
    rx.close().wait().unwrap();
}
#[test]
fn malformed_packet_invalidates_stream() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    io.0.lock().unwrap().packets.push_back(Ok(vec![0; 3]));
    assert!(matches!(
        rx.read(&mut [Complex32::default()], None).wait(),
        Err(Error::Chdr(_))
    ));
    assert!(matches!(rx.start().wait(), Err(Error::StreamClosed)));
    rx.close().wait().unwrap();
}
#[test]
fn surviving_stream_performs_final_owner_cleanup() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    drop(d);
    packet(&io, 0, &[1.]);
    rx.read(&mut [Complex32::default()], None).wait().unwrap();
    rx.close().wait().unwrap();
    assert_eq!(io.0.lock().unwrap().events, vec!["start", "stop", "close"]);
}
#[test]
fn shutdown_is_terminal_retryable_and_idempotent() {
    let (mut d, io) = device();
    io.0.lock().unwrap().fail = Some("close");
    assert!(d.shutdown().wait().is_err());
    assert!(matches!(d.rx_stream(), Err(Error::Shutdown)));
    assert!(matches!(
        d.set_gain(RxGain::Manual(4.)).wait(),
        Err(Error::Shutdown)
    ));
    io.0.lock().unwrap().fail = None;
    d.shutdown().wait().unwrap();
    d.shutdown().wait().unwrap();
    assert_eq!(io.0.lock().unwrap().events, vec!["close", "close"]);
}
#[test]
fn cancelled_shutdown_can_retry() {
    let (mut d, io) = device();
    io.0.lock().unwrap().pause = Some("close");
    let mut op = Box::pin(d.shutdown().into_future());
    assert!(block_on(poll_once(op.as_mut())).is_none());
    drop(op);
    assert!(matches!(d.rx_stream(), Err(Error::Shutdown)));
    io.0.lock().unwrap().pause = None;
    d.shutdown().wait().unwrap();
}
#[test]
fn cancelled_start_and_close_have_drop_cleanup() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    io.0.lock().unwrap().pause = Some("start");
    let mut start = Box::pin(rx.start().into_future());
    assert!(block_on(poll_once(start.as_mut())).is_none());
    drop(start);
    io.0.lock().unwrap().pause = Some("stop");
    let mut close = Box::pin(rx.close().into_future());
    assert!(block_on(poll_once(close.as_mut())).is_none());
    assert!(matches!(d.shutdown().wait(), Err(Error::Busy)));
    io.0.lock().unwrap().pause = None;
    drop(close);
    d.shutdown().wait().unwrap();
    assert_eq!(
        io.0.lock().unwrap().events,
        vec!["start", "stop", "stop", "close"]
    );
}
#[test]
fn native_operations_and_owned_close_are_send() {
    fn send<T: Send>(_: &T) {}
    fn owned<T: Send + 'static>(_: T) {}
    let (mut d, _) = device();
    send(&d.set_gain(RxGain::Automatic).into_future());
    let mut rx = d.rx_stream().unwrap();
    send(&rx.start().into_future());
    owned(rx.close().into_future());
    send(&Device::builder().open().into_future());
}
#[test]
fn defaults_and_invalid_open_are_lazy() {
    let b = Device::builder();
    assert_eq!(b.config.sample_rate_hz, 1e6);
    assert_eq!(b.config.gain, RxGain::Manual(30.));
    let op = b.frequency_hz(f64::NAN).open();
    assert!(matches!(op.wait(), Err(Error::InvalidArgument(_))));
}
#[test]
fn device_overflow_restarts_and_fatal_usb_error_requires_close() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    io.0.lock()
        .unwrap()
        .packets
        .push_back(Ok(crate::chdr::encode_control(0xa0, 0, 8, 0, None)));
    assert!(matches!(
        rx.read(&mut [Complex32::default()], Some(Duration::from_millis(1)))
            .wait(),
        Err(Error::Timeout)
    ));
    assert_eq!(rx.stats().overflows, 1);
    packet(&io, 0, &[5.]);
    rx.read(&mut [Complex32::default()], None).wait().unwrap();
    io.0.lock()
        .unwrap()
        .packets
        .push_back(Err(nusb::transfer::TransferError::Disconnected));
    assert!(matches!(
        rx.read(&mut [Complex32::default()], None).wait(),
        Err(Error::Transfer(_))
    ));
    assert!(matches!(rx.start().wait(), Err(Error::StreamClosed)));
    rx.close().wait().unwrap();
    d.rx_stream().unwrap().close().wait().unwrap();
    d.shutdown().wait().unwrap();
}
#[test]
fn failed_close_requires_terminal_cleanup_retry() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    io.0.lock().unwrap().fail = Some("stop");
    assert!(rx.close().wait().is_err());
    assert!(matches!(d.rx_stream(), Err(Error::Shutdown)));
    io.0.lock().unwrap().fail = None;
    d.shutdown().wait().unwrap();
}
