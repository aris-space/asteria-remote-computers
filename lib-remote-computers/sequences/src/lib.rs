use std::{
    collections::HashMap,
    ops::ControlFlow,
    ops::RangeInclusive,
    time::{Duration, Instant},
};

use datatypes::units::BarG;
use futures::{FutureExt, StreamExt, stream::FuturesUnordered};
use lib_mission::{
    reactive::{IntoWatched, WatchedExt, WatchedWatchable},
    state_machine::StateName,
};
use pss_drivers::{
    input::{SensorDatapointStore, SensorName},
    output::ValveName,
};
pub use sequence_parser::{Command, parse_sequence_str};
use watchable::Watchable;

/// Run a sequence of [`Command`]s on a remote computer.
#[expect(clippy::too_many_lines)] // yeeeeah a bit ugly.
pub async fn run_sequence(
    sequence: impl IntoIterator<Item = &Command<ValveName, SensorName, StateName>>,
    actuators: &pss_drivers::output::PSSValves,
    sensors: &pss_drivers::input::PSSSensors,
) -> ControlFlow<StateName> {
    let mut sensor_watcher = SensorWatcher::new();
    let mut wait_deadline = Option::<Instant>::None;
    let mut settables = actuators.as_settables();

    for (i, command) in sequence.into_iter().enumerate() {
        // Check whether any sensors triggered an abort or wait until the
        // `wait_deadline` is reached, if it's set.
        let mut should_wait = true;
        while should_wait {
            // The duration we need to sleep before executing the next command; when no wait
            // deadline is specified, we "sleep" zero millseconds.
            let duration = wait_deadline.map_or(Duration::ZERO, |deadline| {
                deadline.duration_since(Instant::now())
            });

            tokio::select! {
                biased;
                result = sensor_watcher.check() => {
                    if let Some(abort_state) = result { return ControlFlow::Break(abort_state) }
                    // We might still need to sleep to reach the wait deadline.
                    should_wait = wait_deadline.is_some_and(|deadline| deadline > Instant::now());
                },
                _ = tokio::time::sleep(duration) => break,
            };
        }

        match command {
            Command::Msg(msg) => tracing::info!("Command {i}: [LOG] {msg}"),
            Command::Warning(msg) => tracing::warn!("Command {i}: [WRN] {msg}"),
            Command::Error(msg) => tracing::error!("Command {i}: [ERR] {msg}"),
            Command::Wait(duration) => {
                tracing::info!("Command {i}: Sleeping for {:2}s", duration.as_secs_f32());
                wait_deadline = Some(Instant::now() + *duration);
            }
            Command::ToState(_state) => todo!("unsupported sequence command"),
            Command::ActuatorOpen(actuator) => {
                let Some(settable) = settables.get_mut(actuator) else {
                    // Ignore when the actuator doesn't exist.
                    tracing::info!(
                        "Command {i}: Actuator {actuator:?} doesn't exist, ignoring actuator open command"
                    );
                    return ControlFlow::Continue(());
                };
                tracing::info!("Command {i}: Opening actuator {actuator:?}");
                if let Err(err) = settable.set_value(1.0) {
                    tracing::error!("Command {i}: Could not open {actuator:?}: {err}");
                }
            }
            Command::ActuatorClose(actuator) => {
                let Some(settable) = settables.get_mut(actuator) else {
                    // Ignore when the actuator doesn't exist.
                    tracing::info!(
                        "Command {i}: Actuator {actuator:?} doesn't exist, ignoring actuator close command"
                    );
                    return ControlFlow::Continue(());
                };
                tracing::info!("Command {i}: Closing actuator {actuator:?}");
                if let Err(err) = settable.set_value(1.0) {
                    tracing::error!("Command {i}: Could not close {actuator:?}: {err}");
                }
            }
            Command::DprActivate {
                name: actuator,
                setpoint,
            } => {
                let Some(settable) = settables.get_mut(actuator) else {
                    // Ignore when the actuator doesn't exist.
                    tracing::info!(
                        "Command {i}: Actuator {actuator:?} doesn't exist, ignoring DPR activate command"
                    );
                    return ControlFlow::Continue(());
                };
                tracing::info!("Command {i}: Activating DPR actuator {actuator:?}");
                if let Err(err) = settable.set_value(*setpoint) {
                    tracing::error!("Command {i}: Could not activate {actuator:?}: {err}");
                }
            }
            Command::DprDeactivate(actuator) => {
                let Some(settable) = settables.get_mut(actuator) else {
                    // Ignore when the actuator doesn't exist.
                    tracing::info!(
                        "Command {i}: Actuator {actuator:?} doesn't exist, ignoring DPR deactivate command"
                    );
                    return ControlFlow::Continue(());
                };
                tracing::info!("Command {i}: Deactivating DPR actuator {actuator:?}");
                if let Err(err) = settable.set_value(f32::NAN) {
                    tracing::error!("Command {i}: Could not deactivate {actuator:?}: {err}");
                }
            }
            Command::SensorWatch {
                name,
                abort_state,
                max,
                min,
                key,
            } if name.is_pressure_sensor() => {
                let sensor = sensors
                    .get_pressure_datapoint(name)
                    .expect("we ensured that the name is for a pressure sensor");
                sensor_watcher.add_pressure_watcher(
                    *name,
                    key.clone(),
                    sensor,
                    *min,
                    *max,
                    abort_state.clone(),
                );
            }
            Command::SensorWatch {
                name,
                abort_state,
                max,
                min,
                key,
            } if name.is_temperature_sensor() => {
                todo!("temperature sensor sequence commands are not yet supported")
            }
            Command::SensorIgnore { name, key } => {
                sensor_watcher.remove_pressure_watcher(*name, key.as_ref());
            }
            Command::SensorDetect {
                name,
                value,
                rising,
                abort_state,
                patience,
                duration,
            } if name.is_pressure_sensor() => todo!("unsupported sequence command"),
            Command::SensorDetect {
                name,
                value,
                rising,
                abort_state,
                patience,
                duration,
            } if name.is_temperature_sensor() => {
                todo!("temperature sensor sequence commands are not yet supported")
            }
            _ => unreachable!(
                "we know that every sensor is either a temperature or a pressure sensor"
            ),
        }
    }

    ControlFlow::Continue(())
}

struct WatchedSensor {
    key: Option<String>,
    watcher: WatchedWatchable<BarG>,
    range: RangeInclusive<f32>,
    abort_state: StateName,
}

/// Map of sensors which are currently watched.
struct SensorWatcher(HashMap<SensorName, Vec<WatchedSensor>>);

impl SensorWatcher {
    pub fn new() -> Self {
        Self(HashMap::new())
    }

    /// Add a new sensor which should be watched.
    pub fn add_pressure_watcher(
        &mut self,
        name: SensorName,
        key: Option<String>,
        watchable: &Watchable<BarG>,
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
    pub fn remove_pressure_watcher(&mut self, name: SensorName, key: Option<&String>) {
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
                    .next_satisfying(|value| !range.contains(&value.0))
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
