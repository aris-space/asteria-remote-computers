use lib_mission::reactive::{Watched, WatchedExt};
use lib_mission::util::time::MissionTime;
use std::time::Duration;

/// [`Timer`] is a helper construction to wait for given duration.
///
/// It is initialized with [`Self::new`], giving the clock and the duration to wait for.
/// Awaiting [`Self::wait()`] will then guarantee to return no earlier than the specified duration since the initialization.
/// The returned future is cancellation safe.
pub struct Timer<W> {
    time_watcher: W,
    start_time: MissionTime,
    duration: Duration,
}

impl<W> Timer<W>
where
    W: Watched<MissionTime> + Send,
{
    /// Create a [`Timer`] from an existing [`Watched`].
    pub fn new(time_watcher: W, duration: Duration) -> Self {
        Self {
            start_time: time_watcher.current(),
            time_watcher,
            duration,
        }
    }

    /// Start waiting for `deadline` amount of time before moving on
    pub async fn wait(&mut self) {
        self.time_watcher
            .next_satisfying(|t| *t > self.start_time + self.duration)
            .await;
    }
}
