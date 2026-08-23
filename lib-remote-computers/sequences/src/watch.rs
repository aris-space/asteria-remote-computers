use std::{
    collections::HashMap,
    ops::{ControlFlow, RangeInclusive},
};

use datatypes::units::ValueWithUnit;
use futures::{FutureExt, StreamExt, stream::FuturesUnordered};
use lib_mission::{
    reactive::{IntoWatched, WatchedExt, WatchedWatchable},
    state_machine::StateName,
};
use pss_drivers::input::SensorName;
use watchable::Watchable;

struct WatchedSensor<T: ValueWithUnit<f32>> {
    /// Optional key of the watched sensor used to
    /// ignore it later on.
    key: Option<String>,
    /// Watcher of the underlying sensor.
    watcher: WatchedWatchable<T>,
    /// Range of allowed values of the sensor.
    range: RangeInclusive<f32>,
    /// State into which the state machine should transition
    /// when the underlying sensor contains a value outside
    /// its allowed range.
    abort_state: StateName,
}

/// Map of sensors which are currently watched.
pub struct SensorWatcher<T: ValueWithUnit<f32>>(HashMap<SensorName, Vec<WatchedSensor<T>>>);

impl<T: Send + Sync + Clone + ValueWithUnit<f32>> SensorWatcher<T> {
    pub fn new() -> Self {
        Self(HashMap::new())
    }

    /// Add a new sensor which should be watched.
    pub fn add_watcher(
        &mut self,
        name: SensorName,
        key: Option<String>,
        watchable: &Watchable<T>,
        min: f32,
        max: f32,
        abort_state: StateName,
    ) {
        if let Some(watchers) = self.0.get_mut(&name) {
            watchers.push(WatchedSensor {
                key,
                watcher: watchable.watched(),
                range: min..=max,
                abort_state,
            });
        } else {
            self.0.insert(
                name,
                vec![WatchedSensor {
                    key,
                    watcher: watchable.watched(),
                    range: min..=max,
                    abort_state,
                }],
            );
        }
    }

    /// Remove a sensor from being watched. When `key` is [`Some`], only a specific watched
    /// sensor will be removed; otherwise all watchers of the `name` sensor are removed.
    pub fn remove_watcher(&mut self, name: SensorName, key: Option<&String>) {
        let Some(key) = key else {
            self.0.remove(&name);
            return;
        };
        let Some(watchers) = self.0.get_mut(&name) else {
            return;
        };
        watchers.retain(|watched| watched.key.as_ref().is_none_or(|k| k != key));
    }

    /// Check whether any sensor contains a value which isn't in the range of allowed values.
    ///
    /// If no sensor is registered at the [`SensorWatcher`] instance, [`ControlFlow::Continue`]
    /// is returned. Otherwise, this method waits until one of the following things occur:
    /// - At least one of the registered sensors contains a value outside of the allowed
    ///   range.
    ///   In this case [`ControlFlow::Break`] is returned with the [`StateName`] of the abort
    ///   state into which the state machine should transition.
    /// - The underlying [`Future`] is cancelled because some other [`Future`] inside the
    ///   [`tokio::select!`] completes earlier.
    pub async fn check(&mut self) -> ControlFlow<StateName> {
        // "Stream" of abort states which yields as soon as a registered watcher contains
        // a value outside its allowed range.
        let iter = self.0.values_mut().flat_map(|watchers| {
            watchers.iter_mut().map(|watched| {
                // For each registered watcher we return a "stream" containing the abort state
                // for every disallowed value its sensor contains.

                let WatchedSensor {
                    watcher,
                    range,
                    abort_state,
                    ..
                } = watched;
                watcher
                    .next_satisfying(|value| !range.contains(&value.value()))
                    .map(|_| abort_state.clone())
            })
        });

        // We wait for the first watcher to produce a value outside its allowed range.
        let aborted = iter.collect::<FuturesUnordered<_>>().next().await;
        match aborted {
            Some(abort_state) => ControlFlow::Break(abort_state),
            None => ControlFlow::Continue(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{ops::ControlFlow, time::Duration};

    use datatypes::units::Celsius;
    use lib_mission::state_machine::StateName;
    use pss_drivers::input::SensorName;
    use watchable::Watchable;

    use crate::watch::SensorWatcher;

    #[tokio::test]
    async fn test_continue_when_empty() {
        let mut watcher = SensorWatcher::<Celsius>::new();
        for _ in 0..5 {
            tokio::time::sleep(Duration::from_millis(50)).await;
            assert_eq!(watcher.check().await, ControlFlow::Continue(()));
        }
    }

    #[tokio::test]
    async fn test_abort_when_violated() {
        let mut watcher = SensorWatcher::<Celsius>::new();
        let sensor = Watchable::<Celsius>::default();
        let abort_state = StateName::new_unchecked("asdf").expect("state name should be valid");

        watcher.add_watcher(
            SensorName::eng_cc_p,
            None,
            &sensor,
            10.0,
            15.0,
            abort_state.clone(),
        );

        tokio::time::sleep(Duration::from_millis(50)).await;
        // A value of 12.0 should not violate the watcher.
        sensor.replace(Celsius(12.0));
        tokio::select! {
            _ = watcher.check() => panic!("should not finish"),
            _ = tokio::time::sleep(Duration::from_millis(10)) => { /* fall-through to below */ },
        }

        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(100)).await;
            // A value of 5.0 should violate the watcher and the `watcher.check()` on
            // the main thread should return `ControlFlow::Break(abort)`.
            sensor.replace(Celsius(5.0));
        });

        let result = watcher.check().await;
        assert!(matches!(result, ControlFlow::Break(state) if state == abort_state));
    }
}
