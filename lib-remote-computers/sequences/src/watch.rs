use std::{collections::HashMap, ops::RangeInclusive};

use datatypes::units::ValueWithUnit;
use futures::{FutureExt, StreamExt, stream::FuturesUnordered};
use lib_mission::{
    reactive::{IntoWatched, WatchedExt, WatchedWatchable},
    state_machine::StateName,
};
use pss_drivers::input::SensorName;
use watchable::Watchable;

struct WatchedSensor<T: ValueWithUnit<f32>> {
    key: Option<String>,
    watcher: WatchedWatchable<T>,
    range: RangeInclusive<f32>,
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

    /// Check whether any sensor watcher received a value which isn't in the range of allowed
    /// values. If [`None`] is returned either no sensor received a value or all values are
    /// within the specified ranges. Otherwise, [`Some`] with the name of the abort state is
    /// returned.
    pub async fn check(&mut self) -> Option<StateName> {
        let iter = self.0.values_mut().flat_map(|watchers| {
            watchers.iter_mut().map(|watched| {
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

        let aborted = iter
            .collect::<FuturesUnordered<_>>()
            .collect::<Vec<_>>()
            .await;
        let first = aborted.first().cloned();
        if let Some(first) = &first
            && aborted.iter().any(|state| state != first)
        {
            tracing::warn!(
                "Multiple sensors aborted to different states at the same time. The aborted states are {} and we now aborted into '{first}'",
                aborted.join(", ")
            );
        }
        first
    }
}
