//! This module provides an abstraction to work with reactive data flow.
//!
//! Most useful is the method [`WatchedExt::next_satisfying`] on [`WatchedWatchable`],
//! which is returns once the underlying value satisfies some property.

use crate::util::time::MissionTime;
use std::marker::PhantomData;
use std::time::Duration;
use tokio::sync::mpsc;

/// An abstraction for watching a changing value.
#[async_trait::async_trait] // Otherwise the lifetimes don't work out
pub trait Watched<T> {
    /// This needs to be implemented cancellation safe!!
    ///
    /// When the underlying data is changed, this must return a new value.
    /// This should also return a value directly after construction or calling [`Watched::reset`].
    /// (not every value must be observed, the newest suffices. Spurious wake ups are also allowed.)
    async fn next(&mut self) -> T
    where
        T: Clone + Send;

    /// Returns the current value of the [`Watched`].
    fn current(&self) -> T
    where
        T: Clone;

    /// Calling this must ensure that the next call to [`Watched::next`] will immediately return.
    fn reset(&mut self);
}

/// Extension trait for [`Watched`] providing `next_satisfying`.
#[async_trait::async_trait]
pub trait WatchedExt<T>: Watched<T> {
    /// When the underlying data is changed such that the predicate is true,
    /// this must return the new value.
    ///
    /// If the underlying data only satisfies the predicate for a short time,
    /// this doesn't need to guarantee the value is picked up.
    async fn next_satisfying(&mut self, predicate: impl (for<'a> Fn(&'a T) -> bool) + Send) -> T
    where
        T: Clone + Send,
    {
        // This future cancellation safe, as it can only be cancelled during the `next().await`
        // when there is no state held and the underlying future is cancellation safe by contract.
        loop {
            let x = self.next().await;
            if predicate(&x) {
                return x;
            }
        }
    }

    /// Maps the watched value to a different type using the provided function.
    fn map<U, F>(self, f: F) -> impl Watched<U>
    where
        Self: Sized + Send,
        T: Clone + Send,
        F: Fn(T) -> U + Send + Sync,
    {
        WatchedMapped {
            inner: self,
            inner_phantom: PhantomData,
            f,
        }
    }
}

/// A trait to construct a `Watched`.
///
/// Note that this is implemented for `&Watchable` and is thus consuming.
pub trait IntoWatched<T> {
    type Watched: Watched<T> + Send;

    /// This must return a fresh `Watched`, one that will instantly return with the current value.
    fn watched(self) -> Self::Watched;
}

impl<T, W: Watched<T>> WatchedExt<T> for W {}

/// A [`Watched`] implementor for [`watchable::Watchable`].
pub struct WatchedWatchable<T> {
    has_read_value: bool,
    inner: watchable::Watcher<T>,
}

impl<T> WatchedWatchable<T> {
    /// Create a fresh [`Watched`] for the watchable, by using the given [`watchable::Watcher`].
    pub fn new(watcher: watchable::Watcher<T>) -> Self {
        Self {
            has_read_value: false,
            inner: watcher,
        }
    }
}

#[async_trait::async_trait]
impl<T> Watched<T> for WatchedWatchable<T>
where
    T: Sync,
{
    async fn next(&mut self) -> T
    where
        T: Clone + Send,
    {
        // make sure the first read directly returns.
        if !self.has_read_value {
            self.has_read_value = true;
            return self.inner.get();
        }
        // Wait for an update.
        // If the last update was read and the watchable dropped, then wait forever,
        // because otherwise this would be a busy loop.
        // This should not happen in production where the input tree always exists.
        if self.inner.watch_async().await.is_err() {
            tracing::trace!(
                "stalling WatchedWatchable::<{}>::next() due to dropped watchables",
                std::any::type_name::<T>()
            );
            std::future::pending::<()>().await;
        }
        self.inner.get()
    }

    fn current(&self) -> T
    where
        T: Clone,
    {
        self.inner.get()
    }

    fn reset(&mut self) {
        self.has_read_value = false;
    }
}

/// Easily create a [`Watched`] from a [`watchable::Watchable`].
impl<T: Send + Sync> IntoWatched<T> for &watchable::Watchable<T> {
    type Watched = WatchedWatchable<T>;

    fn watched(self) -> Self::Watched {
        WatchedWatchable::new(self.watch())
    }
}

/// A wrapper around a [`tokio::sync::mpsc::UnboundedReceiver`] that implements [`Watched`].
pub struct WatchedChannel<T> {
    receiver: mpsc::UnboundedReceiver<T>,
    current: T,
    // When false, the next call to `next` returns immediately with the current value.
    has_read_value: bool,
}

impl<T> WatchedChannel<T> {
    /// Creates a new [`WatchedChannel`].
    ///
    /// The `initial` value is used as the current value until a new one is received.
    pub fn new(receiver: mpsc::UnboundedReceiver<T>, initial: T) -> Self {
        Self {
            receiver,
            current: initial,
            has_read_value: false,
        }
    }
}

#[async_trait::async_trait]
impl<T> Watched<T> for WatchedChannel<T> {
    /// Returns a new value whenever one is available. If a reset was requested,
    /// returns the current value immediately.
    async fn next(&mut self) -> T
    where
        T: Clone + Send,
    {
        // If a reset was requested, set the flag and return the current value.
        if !self.has_read_value {
            self.has_read_value = true;
            return self.current.clone();
        }

        // Wait for the next message. Note that `recv` is cancellation-safe.
        if let Some(msg) = self.receiver.recv().await {
            self.current.clone_from(&msg);
            msg
        } else {
            // If the channel were to be closed, we will wait forever to prevent busy looping.
            // This should not happen in production where the channel always exists.
            tracing::trace!(
                "stalling WatchedChannel::<{}>::next() due to closed channel",
                std::any::type_name::<T>()
            );
            std::future::pending().await
        }
    }

    /// Returns the last observed value.
    fn current(&self) -> T
    where
        T: Clone,
    {
        self.current.clone()
    }

    /// Causes the next call to [`Watched::next`] to immediately return the current value.
    fn reset(&mut self) {
        self.has_read_value = false;
    }
}

/// A wrapper that maps a [`Watched`] value to another type.
pub struct WatchedMapped<W, T, F> {
    inner: W,
    inner_phantom: PhantomData<T>,
    f: F,
}

#[async_trait::async_trait]
impl<T, U, F, W> Watched<U> for WatchedMapped<W, T, F>
where
    T: Clone + Send,
    F: Fn(T) -> U + Send + Sync,
    W: Watched<T> + Send,
{
    async fn next(&mut self) -> U
    where
        U: Clone + Send,
    {
        // Because `f`` is synchronous, cancellation safety is lifted from the inner.
        (self.f)(self.inner.next().await)
    }

    fn current(&self) -> U
    where
        U: Clone,
    {
        (self.f)(self.inner.current())
    }

    fn reset(&mut self) {
        self.inner.reset();
    }
}

/// [`Repeatedly`] is a [`Watched`] implementation useful for running some logic periodically.
/// [`Watched::next`] returns once for every increment of `loop_period` of the inner [`Watched`].
/// Will return immediately when called for the first time after construction with [`Repeatedly::new`] or after [`Watched::reset`].
/// (Unenforced) constraint: W must watch some value that ticks up (increases), never decreases.
pub struct Repeatedly<W> {
    time_tick: W,
    next_iteration: MissionTime,
    loop_period: Duration,
}

impl<W> Repeatedly<W>
where
    W: Watched<MissionTime> + Send,
{
    /// Create a [`Repeatedly`] from an existing [`Watched`].
    pub fn new(time_tick: W, loop_period: Duration) -> Self {
        let next_iteration = time_tick.current();
        Self {
            time_tick,
            next_iteration,
            loop_period,
        }
    }
}

#[async_trait::async_trait]
impl<W> Watched<MissionTime> for Repeatedly<W>
where
    W: Watched<MissionTime> + Send,
{
    /// Returns after the value of the inner `time_tick` has exceeded the value of `next_iteration`.
    /// [`Repeatedly::next`] is cancellation safe since it does not alter any shared state when canceled.
    /// The `next_iteration` is only updated after the await operation.
    /// If the future is canceled during the await, the state remains unchanged, and the next invocation
    /// of [`Watched::next`] will correctly continue from the previous `next_iteration` value.
    async fn next(&mut self) -> MissionTime {
        let now = self
            .time_tick
            .next_satisfying(|t| *t >= self.next_iteration)
            .await;
        self.next_iteration += self.loop_period;
        now
    }

    fn current(&self) -> MissionTime {
        self.time_tick.current()
    }

    fn reset(&mut self) {
        self.next_iteration = self.time_tick.current();
    }
}

// Note: these test are mostly AI generated
#[cfg(test)]
mod tests {
    use super::*;
    use tokio::time::{Duration, sleep, timeout};
    use watchable::Watchable;

    /// This test ensures that `WatchedWatchable::next()` correctly returns the initial value immediately
    /// and updates timely when the watchable value changes (also on cloned).
    #[tokio::test]
    async fn test_watched_watchable_next() {
        let watchable = Watchable::new(42);
        let mut watched = watchable.clone().watched();

        // First call should return immediately with initial value
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 42);

        // Next call should timeout as there is no update
        assert!(
            timeout(Duration::from_millis(10), watched.next())
                .await
                .is_err()
        );

        // Update value and check if `next` picks it up
        let _ = watchable.update(100);
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 100);

        // Next call should timeout as there is no update
        assert!(
            timeout(Duration::from_millis(10), watched.next())
                .await
                .is_err()
        );
        // Update value and check if `next` picks it up
        let _ = watchable.update(1337);
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 1337);
    }

    #[tokio::test]
    async fn test_watched_watchable_next_satisfying() {
        let watchable = Watchable::new(10);
        let mut watched = watchable.watched();

        // Spawn a task to update the value after a delay
        tokio::spawn(async move {
            sleep(Duration::from_millis(30)).await;
            let _ = watchable.update(20);
            sleep(Duration::from_millis(30)).await;
            let _ = watchable.update(30);
        });

        // Waiting just 50 ms should timeout as 30 is still not reached
        assert!(
            timeout(
                Duration::from_millis(50),
                watched.next_satisfying(|&x| x >= 25)
            )
            .await
            .is_err()
        );

        // After enough time, next_satisfying should return the expected value
        let Ok(value) = timeout(
            Duration::from_millis(100),
            watched.next_satisfying(|&x| x >= 25),
        )
        .await
        else {
            panic!("next_satisfying() timed out unexpectedly");
        };
        assert_eq!(value, 30);
    }

    #[tokio::test]
    async fn test_watched_watchable_reset() {
        let watchable = Watchable::new(1);
        let mut watched = watchable.watched();

        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 1);

        let _ = watchable.update(2);
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 2);

        watched.reset();
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 2);
    }

    #[tokio::test]
    async fn test_watched_channel_next() {
        let (sender, receiver) = mpsc::unbounded_channel();
        let mut watched = WatchedChannel::new(receiver, 0);

        // First call should return immediately with initial value
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 0);

        // Next call should timeout as there is no update
        assert!(
            timeout(Duration::from_millis(10), watched.next())
                .await
                .is_err()
        );

        // Send a new value and check if `next` picks it up
        let _ = sender.send(42);
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 42);

        // Next call should timeout as there is no update
        assert!(
            timeout(Duration::from_millis(10), watched.next())
                .await
                .is_err()
        );

        // Send another two values and check if `next` picks both up.
        let _ = sender.send(100);
        let _ = sender.send(200);
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 100);
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 200);
    }

    #[tokio::test]
    async fn test_watched_channel_memory() {
        let (sender, receiver) = mpsc::unbounded_channel();
        let mut watched = WatchedChannel::new(receiver, 0);

        // First call should return immediately with initial value
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 0);

        // Verify that `current()` returns the initial value
        assert_eq!(watched.current(), 0);

        // Send a new value and check if `next` picks it up
        let _ = sender.send(42);
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 42);

        // Verify that `current()` returns the updated value
        assert_eq!(watched.current(), 42);

        // Reset and ensure the next call returns the current value immediately
        watched.reset();
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 42);

        // Verify that `current()` still returns the same value after reset
        assert_eq!(watched.current(), 42);

        // Send another value and check if `next` picks it up
        let _ = sender.send(100);
        let Ok(value) = timeout(Duration::from_millis(10), watched.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 100);

        // Verify that `current()` returns the latest value
        assert_eq!(watched.current(), 100);
    }

    #[tokio::test]
    async fn test_watched_mapped() {
        // Create a Watchable and its WatchedWatchable
        let watchable = Watchable::<i16>::new(42);
        let watched = watchable.watched();

        // Map the WatchedWatchable to a new type using a mapping function
        let mut mapped = watched.map(|x| i32::from(x) * 2);

        // First call should return immediately with the mapped initial value
        let Ok(value) = timeout(Duration::from_millis(10), mapped.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 84); // 42 * 2

        // Verify that `current()` returns the mapped initial value
        assert_eq!(mapped.current(), 84);

        // Update the watchable value and check if `next` picks it up and maps it
        let _ = watchable.update(100);
        let Ok(value) = timeout(Duration::from_millis(10), mapped.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 200); // 100 * 2

        // Verify that `current()` returns the mapped updated value
        assert_eq!(mapped.current(), 200);

        // Reset and ensure the next call returns the current mapped value immediately
        mapped.reset();
        let Ok(value) = timeout(Duration::from_millis(10), mapped.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 200);

        // Verify that `current()` still returns the same mapped value after reset
        assert_eq!(mapped.current(), 200);

        // Update the watchable value again and check if `next` picks it up and maps it
        let _ = watchable.update(1337);
        let Ok(value) = timeout(Duration::from_millis(10), mapped.next()).await else {
            panic!("next() timed out unexpectedly");
        };
        assert_eq!(value, 2674); // 1337 * 2

        // Verify that `current()` returns the latest mapped value
        assert_eq!(mapped.current(), 2674);
    }
}
