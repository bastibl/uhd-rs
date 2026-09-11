use super::*;
use futures_lite::future::poll_once;
use io::TestIo;
use std::future::IntoFuture;
use wasm_bindgen_test::*;
wasm_bindgen_test_configure!(run_in_browser);
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
async fn tick() {
    futures_timer::Delay::new(Duration::from_millis(20)).await;
}
#[wasm_bindgen_test]
async fn deferred_drop_keeps_claim_until_cleanup_finishes() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().await.unwrap();
    drop(rx);
    assert!(matches!(d.shutdown().await, Err(Error::Busy)));
    tick().await;
    assert!(matches!(d.rx_stream(), Err(Error::ReopenRequired)));
    d.shutdown().await.unwrap();
    assert_eq!(io.0.lock().unwrap().events, vec!["start", "stop", "close"]);
}
#[wasm_bindgen_test]
async fn stop_retains_pending_queue_and_close_requires_reopen() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().await.unwrap();
    rx.stop().await.unwrap();
    rx.start().await.unwrap();
    assert_eq!(io.0.lock().unwrap().submitted, 16);
    let mut out = [Complex32::default()];
    let mut pending = Box::pin(rx.read(&mut out, None).into_future());
    assert!(poll_once(pending.as_mut()).await.is_none());
    drop(pending);
    rx.close().await.unwrap();
    assert!(matches!(d.rx_stream(), Err(Error::ReopenRequired)));
    d.shutdown().await.unwrap();
}
#[wasm_bindgen_test]
async fn final_owner_closes_after_deferred_stream_stop() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().await.unwrap();
    drop(d);
    drop(rx);
    tick().await;
    tick().await;
    assert_eq!(io.0.lock().unwrap().events, vec!["start", "stop", "close"]);
}
#[wasm_bindgen_test]
async fn dormant_close_permits_replacement_and_is_lazy() {
    let (mut d, io) = device();
    let close = d.rx_stream().unwrap().close();
    assert!(io.0.lock().unwrap().events.is_empty());
    assert!(matches!(d.shutdown().await, Err(Error::Busy)));
    close.await.unwrap();
    d.rx_stream().unwrap().close().await.unwrap();
    d.shutdown().await.unwrap();
}
#[wasm_bindgen_test]
async fn cancelled_close_keeps_claim_until_fallback_finishes() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().await.unwrap();
    io.0.lock().unwrap().pause = Some("stop");
    let mut close = Box::pin(rx.close().into_future());
    assert!(poll_once(close.as_mut()).await.is_none());
    assert!(matches!(d.shutdown().await, Err(Error::Busy)));
    io.0.lock().unwrap().pause = None;
    drop(close);
    tick().await;
    assert!(matches!(d.rx_stream(), Err(Error::ReopenRequired)));
    d.shutdown().await.unwrap();
}
#[wasm_bindgen_test]
async fn failed_close_disables_configuration_until_shutdown_retry() {
    let (mut d, io) = device();
    let mut rx = d.rx_stream().unwrap();
    rx.start().await.unwrap();
    io.0.lock().unwrap().fail = Some("stop");
    assert!(rx.close().await.is_err());
    tick().await;
    assert!(matches!(
        d.set_gain(RxGain::Automatic).await,
        Err(Error::Shutdown)
    ));
    io.0.lock().unwrap().fail = None;
    d.shutdown().await.unwrap();
}
#[cfg(feature = "embedded-images")]
#[wasm_bindgen_test]
fn all_images_decompress_in_wasm_without_fetching_files() {
    for image in Image::ALL {
        let bytes = image.embedded().unwrap();
        assert_eq!(b2xx::image_hash(&bytes), image.pinned_hash());
    }
}
