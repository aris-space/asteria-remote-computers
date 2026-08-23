use core::panic;
use datatypes::units::ValueWithUnit;
use lib_mission::{
    reactive::{IntoWatched, Watched, WatchedWatchable},
    state_machine::StateName,
};
use std::{
    ops::ControlFlow,
    time::{Duration, Instant},
};
use watchable::Watchable;

enum DetectorState {
    /// The detector is waiting for its detection condition to be fulfilled.
    Waiting,
    /// The detector's detection condition is fulfilled but it needs to be
    /// held until the specified [`Instant`].
    Holding(Instant),
    /// The detector's detection condition wasn't fulfilled (long enough)
    /// before the detection deadline exceeded.
    Aborted(StateName),
    /// The detector's detection condition was fulfilled (long enough).
    Finished,
}

pub struct SensorDetector<'a, T: ValueWithUnit<f32>> {
    /// Watcher of the underlying sensor.
    watcher: WatchedWatchable<T>,
    /// Threshold above/below we trigger detection.
    threshold: f32,
    /// Boolean whether we want to detect values above ([`true`]) or below ([`false`]) the threshold.
    rising: bool,
    /// Name of the abort state.
    abort: &'a StateName,
    /// Instant after which we transition into the abort state.
    deadline: Instant,
    /// Optional duration for how long the value needs to stay above/below the threshold.
    duration: Option<Duration>,
    /// Current state of the sensor detector's state machine.
    state: DetectorState,
}

impl<'a, T: Send + Sync + Clone + ValueWithUnit<f32>> SensorDetector<'a, T> {
    pub fn new(
        watchable: &Watchable<T>,
        value: f32,
        rising: bool,
        abort_state: &'a StateName,
        patience: Duration,
        duration: Option<Duration>,
    ) -> Self {
        Self {
            watcher: watchable.watched(),
            threshold: value,
            rising,
            abort: abort_state,
            deadline: Instant::now() + patience,
            duration,
            state: DetectorState::Waiting,
        }
    }

    /// Detect sensor values according to the definition of this [`SensorDetector`].
    ///
    /// This method waits until one of the following things occur:
    /// - The underlying sensor contains a value which fulfills the detection condition
    ///   of this [`SensorDetector`] instance, and if configured, the condition stays
    ///   fulfilled for at least `duration`.
    ///   In this case [`ControlFlow::Continue`] is returned.
    /// - The condition is not fulfilled (or not long enough) until the [`SensorDetector`]s
    ///   deadline exceeds.
    ///   In this case [`ControlFlow::Break`] is returned with the [`StateName`] of the abort
    ///   state into which the state machine should transition.
    /// - The underlying [`Future`] is cancelled because some other [`Future`] inside the
    ///   [`tokio::select!`] completes earlier.
    pub async fn detect(&mut self) -> ControlFlow<StateName> {
        loop {
            // Duration to sleep until the detector is finished.
            let duration_until_finished = match &self.state {
                // The condition was not fulfilled, so the "sleep until finished" sleep
                // should never complete.
                DetectorState::Waiting => Duration::MAX,
                // If `instant` is in the past we return [`Duration::ZERO`] such that the
                // sleep completes immediately. Otherwise, it's the duration from
                // [`Instant::now`] to `instant`.
                DetectorState::Holding(instant) => instant
                    .checked_duration_since(Instant::now())
                    .unwrap_or(Duration::ZERO),
                // The detector previously aborted; we simply abort "again".
                DetectorState::Aborted(state) => return ControlFlow::Break(state.clone()),
                // The detector previously finished; we simply finish "again".
                DetectorState::Finished => return ControlFlow::Continue(()),
            };

            // Duration until the patience of the detection is exceeded and we need to
            // transition into the abort state.
            let duration_until_abort = self
                .deadline
                .checked_duration_since(Instant::now())
                .unwrap_or(Duration::ZERO);

            // Boolean whether the detection condition is fulfilled after either the sensor
            // returned a new value or after a sleep finished.
            let fulfills = tokio::select! {
                biased;
                value = self.watcher.next() => {
                    let inner = value.value();
                    (self.rising && inner > self.threshold) || (!self.rising && inner < self.threshold)
                },
                _ = tokio::time::sleep(duration_until_finished) => {
                    self.state = DetectorState::Finished;
                    continue;
                },
                _ = tokio::time::sleep(duration_until_abort) => {
                    self.state = DetectorState::Aborted(self.abort.clone());
                    continue;
                },
            };

            if fulfills {
                // The condition is fulfilled.
                if let Some(duration) = self.duration {
                    // The condition must be fulfilled for `duration`; we thus need
                    // to wait until the condition was held for long enough.
                    self.state = DetectorState::Holding(Instant::now() + duration);
                } else {
                    // The condition must not hold for a certain duration; the
                    // detector is thus immediately finished.
                    self.state = DetectorState::Finished;
                }
            } else {
                // The condition is not fulfilled; we transition into the
                // [`DetectorState::Waiting`] state.
                self.state = DetectorState::Waiting;
            }
        }
    }
}
