use std::{collections::HashMap, fmt::Debug, hash::Hash, ops::RangeInclusive};

use datatypes::units::ValueWithUnit;
use futures::{FutureExt, StreamExt, stream::FuturesUnordered};
use lib_mission::{
    reactive::{IntoWatched, WatchedExt, WatchedWatchable},
    state_machine::StateName,
};
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
pub struct SensorWatcher<S: Hash + Eq + Clone + Debug, T: ValueWithUnit<f32>>(
    HashMap<S, Vec<WatchedSensor<T>>>,
);

impl<S: Hash + Eq + Clone + Debug, T: Send + Sync + Clone + ValueWithUnit<f32>>
    SensorWatcher<S, T>
{
    pub fn new() -> Self {
        Self(HashMap::new())
    }

    /// Add a new sensor which should be watched.
    pub fn add_watcher(
        &mut self,
        name: S,
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
    pub fn remove_watcher(&mut self, name: &S, key: Option<&String>) {
        let Some(key) = key else {
            self.0.remove(name);
            return;
        };
        let Some(watchers) = self.0.get_mut(name) else {
            return;
        };
        watchers.retain(|watched| watched.key.as_ref().is_none_or(|k| k != key));
    }

    /// What all configured sensors for a value which isn't in the range of allowed values.
    ///
    /// If no sensor is registered at the [`SensorWatcher`] instance, this waits indefinitely.
    /// Otherwise, this method waits until one of the following things occur:
    /// - At least one of the registered sensors contains a value outside of the allowed range.
    ///   In this case the [`StateName`] of the abort state into which the state machine should transition is returned.
    /// - The underlying [`Future`] is cancelled because some other [`Future`] inside a
    ///   [`tokio::select!`] completes earlier.
    pub async fn watch(&mut self) -> StateName {
        // "Stream" of abort states which yields as soon as a registered watcher contains
        // a value outside its allowed range.
        let iter = self.0.iter_mut().flat_map(|(name, watchers)| {
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
                    .map(|value| (name.clone(), value, range.clone(), abort_state.clone()))
            })
        });

        // We wait for the first watcher to produce a value outside its allowed range.
        let aborted = iter.collect::<FuturesUnordered<_>>().next().await;
        match aborted {
            Some((name, value, range, abort_state)) => {
                tracing::error!(
                    "sensor {name:?} has a value of {value} which is outside its allowed range of [{}{}, {}{}]; transitioning into {abort_state}",
                    range.start(),
                    T::UNIT,
                    range.end(),
                    T::UNIT
                );
                abort_state
            }
            // Nothing to watch for, so no violation will ever happen
            None => std::future::pending().await,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use datatypes::units::Celsius;
    use lib_mission::state_machine::StateName;
    use watchable::Watchable;

    use crate::watch::SensorWatcher;

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    enum DummySensorName {
        ChamberPressure,
    }

    #[tokio::test]
    async fn test_pending_forever_when_empty() {
        let mut watcher = SensorWatcher::<DummySensorName, Celsius>::new();
        for _ in 0..5 {
            tokio::select! {
                _ = watcher.watch() => panic!("should never resolve without any watched sensor"),
                () = tokio::time::sleep(Duration::from_millis(50)) => { /* fall-through to below */ },
            }
        }
    }

    #[tokio::test]
    async fn test_abort_when_violated() {
        let mut watcher = SensorWatcher::<DummySensorName, Celsius>::new();
        let sensor = Watchable::<Celsius>::default();
        let abort_state = StateName::new_unchecked("asdf").expect("state name should be valid");

        watcher.add_watcher(
            DummySensorName::ChamberPressure,
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
            _ = watcher.watch() => panic!("should not finish"),
            _ = tokio::time::sleep(Duration::from_millis(10)) => { /* fall-through to below */ },
        }

        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(100)).await;
            // A value of 5.0 should violate the watcher and the `watcher.watch()` on
            // the main thread should resolve with `abort_state`.
            sensor.replace(Celsius(5.0));
        });

        let result = watcher.watch().await;
        assert_eq!(result, abort_state);
    }
}
