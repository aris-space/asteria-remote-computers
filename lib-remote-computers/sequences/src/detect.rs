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
    /// If a `duration` is specified and the detection condition is currently fulfilled,
    /// this field contains the instant until which the value needs to stay above the
    /// detection threshold until the execution can continue. Otherwise, this field
    /// is [`None`].
    duration_deadline: Option<Instant>,
    /// Boolean condition whether the detector is finished. Calling
    /// [`SensorDetector::detect`] on a finished detector will panic.
    finished: bool,
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
            duration_deadline: None,
            finished: false,
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
    ///
    /// After the method returned [`ControlFlow::Continue`] or [`ControlFlow::Break`] the
    /// [`SensorDetector`] instance should be treated as "consumed". This means any subsequent
    /// calls to [`SensorDetector::detect`] will panic.
    pub async fn detect(&mut self) -> ControlFlow<StateName> {
        assert!(
            !self.finished,
            "called `detect` on a finished sensor detector"
        );

        loop {
            // If the condition is currently fulfilled, this is the duration to sleep
            // until the condition is met for at least `duration`. Otherwise, this is
            // the [`Duration::MAX`], meaning we wait until the sensor changes and the
            // new value fulfills the detection condition.
            let duration_until_finished = self
                .duration_deadline
                .and_then(|instant| instant.checked_duration_since(Instant::now()))
                .unwrap_or(Duration::MAX);

            // Duration until the patience of the detection is exceeded and we need to
            // transition into the abort state.
            let duration_until_abort = self
                .deadline
                .checked_duration_since(Instant::now())
                .unwrap_or(Duration::ZERO);

            // Boolean whether the dection condition is fulfilled after either the sensor
            // returned a new value or after a sleep finished.
            let fulfills = tokio::select! {
                biased;
                value = self.watcher.next() => {
                    let inner = value.value();
                    (self.rising && inner > self.threshold) || (!self.rising && inner < self.threshold)
                },
                // `detection_until` is only `Some` when the detection condition is currently
                // fulfilled.
                _ = tokio::time::sleep(duration_until_finished) => self.duration_deadline.is_some(),
                _ = tokio::time::sleep(duration_until_abort) => {
                    self.finished = true;
                    return ControlFlow::Break(self.abort.clone())
                },
            };

            match self.duration_deadline {
                // The condition was already fulfilled; we need to check whether
                // it was fulfilled for long enough.
                Some(goal) if fulfills => {
                    if goal <= Instant::now() {
                        // The duration deadline is exceeded and we can continue
                        // the sequence execution.
                        self.finished = true;
                        return ControlFlow::Continue(());
                    }
                }
                // The condition is freshly fulfilled.
                None if fulfills => {
                    if let Some(duration) = self.duration {
                        // The condition must be fulfilled for `duration`; we thus
                        // cannot return from here now.
                        self.duration_deadline = Some(Instant::now() + duration);
                    } else {
                        self.finished = true;
                        return ControlFlow::Continue(());
                    }
                }
                // The condition is not fulfilled; we need to reset the detection
                // deadline.
                _ => self.duration_deadline = None,
            }
        }
    }
}
