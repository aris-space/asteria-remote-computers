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
                // for every unallowed value its sensor contains.

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
