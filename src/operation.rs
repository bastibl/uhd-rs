//! Execution adapter. Blocking calls use nusb's native blocking entry points;
//! radio algorithms remain shared. The mode is scoped to each poll, never held
//! across a yield, so native async futures may migrate between executor threads.
use nusb::MaybeFuture;
#[cfg(not(target_arch = "wasm32"))]
pub(crate) use std::marker::Send as PortableSend;
use std::{future::Future, time::Duration};
#[cfg(target_arch = "wasm32")]
pub(crate) trait PortableSend {}
#[cfg(target_arch = "wasm32")]
impl<T> PortableSend for T {}

#[must_use = "operations are lazy; call .wait() or .await"]
pub(crate) struct Operation<F>(F);
pub(crate) fn operation<F: Future + PortableSend>(future: F) -> Operation<F> {
    Operation(future)
}
impl<F: Future> std::future::IntoFuture for Operation<F> {
    type Output = F::Output;
    type IntoFuture = F;
    fn into_future(self) -> F {
        self.0
    }
}
impl<F: Future + PortableSend> MaybeFuture for Operation<F> {
    #[cfg(not(target_arch = "wasm32"))]
    fn wait(self) -> F::Output {
        blocking(self.0)
    }
}
#[cfg(not(target_arch = "wasm32"))]
thread_local! { static BLOCKING: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
static DEADLINE: std::cell::Cell<Option<web_time::Instant>> = const { std::cell::Cell::new(None) }; }
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn is_blocking() -> bool {
    BLOCKING.get()
}
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn blocking<F: Future>(future: F) -> F::Output {
    struct Reset(bool);
    impl Drop for Reset {
        fn drop(&mut self) {
            BLOCKING.set(self.0);
        }
    }
    let mut future = std::pin::pin!(future);
    futures_lite::future::block_on(std::future::poll_fn(|cx| {
        let _reset = Reset(BLOCKING.replace(true));
        future.as_mut().poll(cx)
    }))
}
pub(crate) async fn usb<M: MaybeFuture>(op: M) -> M::Output {
    #[cfg(not(target_arch = "wasm32"))]
    if is_blocking() || !cfg!(any(feature = "smol", feature = "tokio")) {
        return op.wait();
    }
    op.await
}
pub(crate) async fn completion<D: nusb::transfer::EndpointDirection>(
    endpoint: &mut nusb::Endpoint<nusb::transfer::Bulk, D>,
    timeout: Duration,
) -> crate::Result<nusb::transfer::Completion> {
    let timeout = remaining(timeout)?;
    #[cfg(not(target_arch = "wasm32"))]
    if is_blocking() {
        return endpoint
            .wait_next_complete(timeout)
            .ok_or(crate::Error::Timeout);
    }
    futures_lite::future::race(async { Ok(endpoint.next_complete().await) }, async {
        futures_timer::Delay::new(timeout).await;
        Err(crate::Error::Timeout)
    })
    .await
}
pub(crate) fn remaining(timeout: Duration) -> crate::Result<Duration> {
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(deadline) = DEADLINE.get() {
        let left = deadline.saturating_duration_since(web_time::Instant::now());
        if left.is_zero() {
            return Err(crate::Error::Timeout);
        }
        return Ok(timeout.min(left));
    }
    Ok(timeout)
}
pub(crate) async fn bounded<T>(
    future: impl Future<Output = crate::Result<T>>,
    timeout: Duration,
) -> crate::Result<T> {
    let deadline = web_time::Instant::now() + timeout;
    let mut future = std::pin::pin!(future);
    let timed = std::future::poll_fn(|cx| {
        if web_time::Instant::now() >= deadline {
            return std::task::Poll::Ready(Err(crate::Error::Timeout));
        }
        #[cfg(not(target_arch = "wasm32"))]
        let _reset = {
            struct Reset(Option<web_time::Instant>);
            impl Drop for Reset {
                fn drop(&mut self) {
                    DEADLINE.set(self.0);
                }
            }
            let old = DEADLINE.get();
            DEADLINE.set(Some(old.map_or(deadline, |d| d.min(deadline))));
            Reset(old)
        };
        future.as_mut().poll(cx)
    });
    futures_lite::future::race(timed, async {
        futures_timer::Delay::new(timeout).await;
        Err(crate::Error::Timeout)
    })
    .await
}
#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;
    #[test]
    fn native_mode_is_scoped_to_each_poll() {
        operation(async {
            assert!(is_blocking());
            futures_timer::Delay::new(Duration::from_millis(1)).await;
            assert!(is_blocking());
        })
        .wait();
        assert!(!is_blocking());
    }
    #[test]
    fn actual_elapsed_time_expires_nested_blocking_deadlines() {
        let result = blocking(bounded(
            async {
                std::thread::sleep(Duration::from_millis(3));
                remaining(Duration::from_secs(10))
            },
            Duration::from_millis(1),
        ));
        assert!(matches!(result, Err(crate::Error::Timeout)));
        assert!(remaining(Duration::from_secs(10)).is_ok());
    }
}
