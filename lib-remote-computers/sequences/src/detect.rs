use core::panic;
use datatypes::units::ValueWithUnit;
use lib_mission::{
    reactive::{IntoWatched, Watched, WatchedWatchable},
    state_machine::StateName,
};
use std::{
    fmt::Debug,
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

pub struct SensorDetector<'a, S: Debug, T: ValueWithUnit<f32>> {
    /// Name of the sensor.
    name: &'a S,
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

impl<'a, S: Debug, T: Send + Sync + Clone + ValueWithUnit<f32>> SensorDetector<'a, S, T> {
    pub fn new(
        name: &'a S,
        watchable: &Watchable<T>,
        value: f32,
        rising: bool,
        abort_state: &'a StateName,
        patience: Duration,
        duration: Option<Duration>,
    ) -> Self {
        Self {
            name,
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
                DetectorState::Aborted(state) => {
                    tracing::error!(
                        "detector for sensor {:?} aborted; the detection condition of {} {}{} was not {}; transitioning into {state}",
                        self.name,
                        self.threshold,
                        T::UNIT,
                        if self.rising { "above" } else { "below" },
                        if self.duration.is_some() {
                            "fulfilled long enough"
                        } else {
                            "fulfilled"
                        }
                    );
                    return ControlFlow::Break(state.clone());
                }
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

#[cfg(test)]
mod tests {
    use std::{
        ops::ControlFlow,
        time::{Duration, Instant},
    };

    use datatypes::units::Celsius;
    use lib_mission::state_machine::StateName;
    use tokio::sync::oneshot;
    use watchable::Watchable;

    use crate::detect::SensorDetector;

    #[tokio::test]
    async fn test_abort() {
        let sensor = Watchable::<Celsius>::default();
        let abort_state = StateName::new_unchecked("asdf").expect("state name should be valid");
        let mut detector = SensorDetector::<&'static str, Celsius>::new(
            &"sensor",
            &sensor,
            10.0,
            true,
            &abort_state,
            Duration::from_millis(100),
            None,
        );

        let result = detector.detect().await;
        assert!(matches!(result, ControlFlow::Break(state) if state == abort_state));
    }

    #[tokio::test]
    async fn test_finished_immediately() {
        let sensor = Watchable::<Celsius>::default();
        let abort_state = StateName::new_unchecked("asdf").expect("state name should be valid");
        let mut detector = SensorDetector::<&'static str, Celsius>::new(
            &"sensor",
            &sensor,
            10.0,
            true,
            &abort_state,
            Duration::from_millis(1000),
            None,
        );

        // A value of 4.0 should not trigger a detection.
        sensor.replace(Celsius(4.0));
        tokio::select! {
            _ = detector.detect() => panic!("should not detect"),
            _ = tokio::time::sleep(Duration::from_millis(10)) => { /* fall-through to below */ },
        }

        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(100)).await;
            // A value of 12.0 should trigger a detection in the main thread.
            sensor.replace(Celsius(12.0));
        });

        let result = detector.detect().await;
        assert_eq!(result, ControlFlow::Continue(()));
    }

    #[tokio::test]
    async fn test_finished_with_duration() {
        let sensor = Watchable::<Celsius>::default();
        let abort_state = StateName::new_unchecked("asdf").expect("state name should be valid");
        let mut detector = SensorDetector::<&'static str, Celsius>::new(
            &"sensor",
            &sensor,
            10.0,
            true,
            &abort_state,
            Duration::from_millis(1000),
            Some(Duration::from_millis(250)),
        );

        let (tx, rx) = oneshot::channel::<Instant>();

        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(100)).await;
            // A value of 12.0 should trigger a detection in the main thread.
            sensor.replace(Celsius(12.0));
            tx.send(Instant::now())
                .expect("should transmit write instant");
        });

        let result = detector.detect().await;
        assert_eq!(result, ControlFlow::Continue(()));

        let finished_at = Instant::now();
        let written_at = rx.await.expect("should receive write instant");
        // Ensure that the detection waited for at least `duration` until the hold
        // timeout of the detector exceeded.
        assert!(finished_at >= written_at + Duration::from_millis(250));
    }

    #[tokio::test]
    async fn test_abort_during_hold() {
        let sensor = Watchable::<Celsius>::default();
        let abort_state = StateName::new_unchecked("asdf").expect("state name should be valid");
        let mut detector = SensorDetector::<&'static str, Celsius>::new(
            &"sensor",
            &sensor,
            10.0,
            true,
            &abort_state,
            Duration::from_millis(100),
            Some(Duration::from_millis(25)),
        );

        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(80)).await;
            // A value of 12.0 should trigger a detection in the main thread.
            sensor.replace(Celsius(12.0));
        });

        let result = detector.detect().await;
        // The detection should abort because 80ms (sleep) + 25ms (hold time) > 100ms (patience).
        assert_eq!(result, ControlFlow::Break(abort_state));
    }

    #[tokio::test]
    async fn test_abort_after_hold_violation() {
        let sensor = Watchable::<Celsius>::default();
        let abort_state = StateName::new_unchecked("asdf").expect("state name should be valid");
        let mut detector = SensorDetector::<&'static str, Celsius>::new(
            &"sensor",
            &sensor,
            10.0,
            true,
            &abort_state,
            Duration::from_millis(100),
            Some(Duration::from_millis(25)),
        );

        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(10)).await;
            // A value of 12.0 should trigger a detection in the main thread.
            sensor.replace(Celsius(12.0));

            // 5ms (sleep) < 25ms (hold time), thus the detector should not
            // finish.
            tokio::time::sleep(Duration::from_millis(5)).await;
            sensor.replace(Celsius(5.0));
        });

        let result = detector.detect().await;
        assert_eq!(result, ControlFlow::Break(abort_state));
    }
}
