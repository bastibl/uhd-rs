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
        .flat_map(|x| [(-*x as i16).to_le_bytes(), (*x as i16).to_le_bytes()].concat())
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
    assert_eq!(io.0.lock().unwrap().submitted, QUEUE_DEPTH);
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
    assert_eq!(io.0.lock().unwrap().submitted, QUEUE_DEPTH + 1);
    drop(control);
    assert_eq!(rx.close().wait().unwrap().samples, 3);
    d.shutdown().wait().unwrap();
}
#[test]
fn ready_packets_are_batched_without_waiting_for_another_transfer() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    packet(&io, 0, &[1., 2.]);
    packet(&io, 1, &[3., 4.]);
    let mut out = [Complex32::default(); 16];
    let read = rx
        .read(&mut out, Some(Duration::from_secs(1)))
        .into_future();
    // The output isn't full and the next transfer never completes. This poll
    // must still return the already received samples immediately.
    assert_eq!(block_on(poll_once(read)).unwrap().unwrap(), 4);
    assert_eq!(
        out[..4].iter().map(|x| x.re).collect::<Vec<_>>(),
        [1., 2., 3., 4.]
    );
    assert_eq!(rx.stats().samples, 4);
    assert_eq!(rx.stats().transfers, 2);
    assert!(
        rx.buffer.samples.is_empty(),
        "whole packets decode into the caller's buffer"
    );
    assert_eq!(rx.buffer.samples.capacity(), 0);
    assert_eq!(io.0.lock().unwrap().submitted, QUEUE_DEPTH + 2);
    rx.close().wait().unwrap();
}
#[test]
fn ready_packet_batch_is_bounded_and_retains_partial_packets() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    for seq in 0..6 {
        packet(&io, seq, &[f32::from(seq)]);
    }
    let mut out = [Complex32::default(); 16];
    assert_eq!(rx.read(&mut out, None).wait().unwrap(), READ_BATCH_PACKETS);
    assert_eq!(
        out[..4].iter().map(|x| x.re).collect::<Vec<_>>(),
        [0., 1., 2., 3.]
    );
    assert_eq!(rx.read(&mut out, None).wait().unwrap(), 2);
    packet(&io, 6, &[6., 7., 8.]);
    packet(&io, 7, &[9., 10.]);
    assert_eq!(rx.read(&mut out[..2], None).wait().unwrap(), 2);
    assert_eq!(rx.read(&mut out, None).wait().unwrap(), 3);
    assert_eq!(
        out[..3].iter().map(|x| x.re).collect::<Vec<_>>(),
        [8., 9., 10.]
    );
    assert_eq!(rx.stats().samples, 11);
    rx.close().wait().unwrap();
}
#[test]
fn error_after_valid_samples_is_reported_on_the_following_read() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    packet(&io, 0, &[1., 2.]);
    io.0.lock().unwrap().packets.push_back(Ok(vec![0; 3]));
    let mut out = [Complex32::default(); 16];
    assert_eq!(rx.read(&mut out, None).wait().unwrap(), 2);
    assert_eq!(out[1].re, 2.);
    assert!(matches!(
        rx.read(&mut out, None).wait(),
        Err(Error::Chdr(_))
    ));
    assert!(matches!(
        rx.read(&mut out, None).wait(),
        Err(Error::StreamClosed)
    ));
    assert_eq!(rx.stats().samples, 2);
    rx.close().wait().unwrap();
}
#[test]
fn batch_ends_at_sequence_gap_and_recovers_on_following_read() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    packet(&io, 0, &[1., 2.]);
    packet(&io, 2, &[3.]);
    packet(&io, 3, &[4., 5.]);
    let mut out = [Complex32::default(); 16];
    assert_eq!(rx.read(&mut out, None).wait().unwrap(), 2);
    assert_eq!(out[0].re, 1.);
    assert_eq!(rx.read(&mut out, None).wait().unwrap(), 2);
    assert_eq!(out[0].re, 4.);
    assert_eq!(rx.stats().overflows, 1);
    assert_eq!(rx.stats().samples, 4);
    rx.close().wait().unwrap();
}
#[test]
fn fifo_overflow_after_samples_is_restarted_on_the_following_read() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().wait().unwrap();
    packet(&io, 0, &[1., 2.]);
    io.0.lock()
        .unwrap()
        .packets
        .push_back(Ok(crate::chdr::encode_control(0xa0, 1, 8, 0, None)));
    let mut out = [Complex32::default(); 16];
    assert_eq!(rx.read(&mut out, None).wait().unwrap(), 2);
    assert!(matches!(
        rx.read(&mut out, Some(Duration::from_millis(1))).wait(),
        Err(Error::Timeout)
    ));
    assert_eq!(rx.stats().overflows, 1);
    assert_eq!(io.0.lock().unwrap().events, ["start", "start"]);
    packet(&io, 0, &[3.]);
    assert_eq!(rx.read(&mut out, None).wait().unwrap(), 1);
    assert_eq!(out[0].re, 3.);
    rx.close().wait().unwrap();
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
    assert_eq!(io.0.lock().unwrap().submitted, QUEUE_DEPTH);
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

#[test]
fn rx_controls_are_lazy_and_cancelled_configuration_is_unknown() {
    let (mut d, io) = device();
    drop(d.set_antenna(RxAntenna::TxRx));
    drop(d.set_bandwidth(2e6));
    drop(d.set_dc_offset_enabled(false));
    assert!(io.0.lock().unwrap().events.is_empty());
    assert_eq!(d.antenna().wait().unwrap(), RxAntenna::Rx2);
    assert!(d.dc_offset_enabled().wait().unwrap());
    io.0.lock().unwrap().pause = Some("bandwidth");
    let mut change = Box::pin(d.set_bandwidth(2e6).into_future());
    assert!(block_on(poll_once(change.as_mut())).is_none());
    drop(change);
    assert!(matches!(d.bandwidth().wait(), Err(Error::ReopenRequired)));
    io.0.lock().unwrap().pause = None;
    assert_eq!(d.set_bandwidth(2e6).wait().unwrap(), 2e6);
    d.set_antenna(RxAntenna::TxRx).wait().unwrap();
    d.set_dc_offset_enabled(false).wait().unwrap();
    assert_eq!(d.antenna().wait().unwrap(), RxAntenna::TxRx);
    assert!(!d.dc_offset_enabled().wait().unwrap());
    assert!(matches!(
        d.set_bandwidth(f64::NAN).wait(),
        Err(Error::InvalidArgument(_))
    ));
    assert_eq!(d.bandwidth().wait().unwrap(), 2e6);
    d.shutdown().wait().unwrap();
    assert!(matches!(d.antenna().wait(), Err(Error::Shutdown)));
}

#[test]
fn builder_rejects_invalid_bandwidth_before_discovery() {
    for hz in [f64::NAN, f64::INFINITY, -1.0, 199_999.0, 56_000_001.0] {
        assert!(matches!(
            Device::builder().bandwidth_hz(hz).open().wait(),
            Err(Error::InvalidArgument(_))
        ));
    }
    assert!(matches!(
        Device::builder()
            .sample_rate_hz(16e6)
            .lo_offset_hz(9e6)
            .open()
            .wait(),
        Err(Error::InvalidArgument(_))
    ));
}
