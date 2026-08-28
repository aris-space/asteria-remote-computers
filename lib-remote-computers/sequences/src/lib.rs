use std::{
    fmt::Debug,
    hash::Hash,
    ops::ControlFlow,
    time::{Duration, Instant},
};

use datatypes::units::{BarG, Celsius};
use lib_mission::{actuator_control::ActuatorStore, state_machine::StateName};
pub use sequence_parser::{Command, parse_sequence_str};
use watchable::Watchable;

use crate::{detect::SensorDetector, watch::SensorWatcher};

mod detect;
mod watch;

/// Trait which must be implemented by any type which represents
/// sensor names. The trait allows to determine the kind of sensor
/// this name is for (e.g., pressure, temperature, ...).
pub trait SensorNameExt {
    /// Boolean whether `self` is the name of a pressure sensor.
    fn is_pressure_sensor(&self) -> bool;

    /// Boolean whether `self` is the name of a temperature sensor.
    fn is_temperature_sensor(&self) -> bool;
}

/// Trait to get typed sensor values by name.
/// The implementation must match the one of `T`.
pub trait SensorStore<T: SensorNameExt> {
    /// Get a watchable for a pressure sensor by its `name`.
    /// Returns [`None`] if the provided `name` is not the name of a pressure sensor.
    fn get_pressure_datapoint<'a>(&'a self, name: &T) -> Option<&'a Watchable<BarG>>;

    /// Get a watchable for a temperature sensor by its `name`.
    /// Returns [`None`] if the provided `name` is not the name of a temperature sensor.
    fn get_temperature_datapoint<'a>(&'a self, name: &T) -> Option<&'a Watchable<Celsius>>;
}

enum Blocker<'a> {
    /// We're blocked on a [`Command::Wait`] until the underlying
    /// deadline exceeds.
    Wait(Instant),
    /// We're blocked on a [`Command::SensorDetect`] until the
    /// underlying [`SensorDetector`] either finishes or aborts.
    Detector(SensorDetectorKind<'a>),
}

/// Run a sequence of [`Command`]s on a remote computer.
#[expect(clippy::too_many_lines)] // yeeeeah a bit ugly.
pub async fn run_sequence<ValveName, SensorName>(
    sequence: impl IntoIterator<Item = &Command<ValveName, SensorName, StateName>>,
    actuators: &impl ActuatorStore<ValveName>,
    sensors: &impl SensorStore<SensorName>,
) -> ControlFlow<StateName>
where
    ValveName: Clone + Debug + Ord + 'static,
    SensorName: SensorNameExt + Clone + Debug + Hash + Eq + 'static,
{
    let mut pressure_watcher = SensorWatcher::<SensorName, BarG>::new();
    let mut temperature_watcher = SensorWatcher::<SensorName, Celsius>::new();

    let mut blocker = Option::<Blocker>::None;
    let mut settables = actuators.as_settables();

    for (i, command) in sequence.into_iter().enumerate() {
        // Before we execute the next sequence command we first need to check whether
        // there is still a blocker. Should that be the case, we execute the blocker
        // until it's removed.
        // Additionally, we also check whether any of the active watchers was violated
        // in the meantime. If that's the case, we return an abort state.
        loop {
            match &mut blocker {
                Some(Blocker::Detector(detector)) => {
                    tokio::select! {
                        result = detector.detect() => {
                            result?;
                            // The detector finished; we can thus remove the blocker.
                            blocker = None;
                        },
                        result = pressure_watcher.check() => result?,
                        result = temperature_watcher.check() => result?,
                    };
                }
                Some(Blocker::Wait(deadline)) => {
                    // Sleep until `deadline`. If `deadline` is no longer in the future we sleep
                    // for zero milliseconds and then remove the blocker.
                    let duration = deadline
                        .checked_duration_since(Instant::now())
                        .unwrap_or(Duration::ZERO);

                    tokio::select! {
                        biased;
                        result = pressure_watcher.check() => result?,
                        result = temperature_watcher.check() => result?,
                        _ = tokio::time::sleep(duration) => {
                            // The wait is finished; we can thus remove the blocker.
                            blocker = None;
                        },
                    };
                }
                // We're not blocked on anything before being able to execute the next sequence
                // command. Since there might have been a watcher event since the last command was
                // executed, we need to check for such events before continuing the execution of
                // the next sequence command.
                None => tokio::select! {
                        biased;
                        result = pressure_watcher.check() => result?,
                        result = temperature_watcher.check() => result?,
                        // No watcher had an event; we can thus start executing the next
                        // sequence command.
                        _ = tokio::time::sleep(Duration::ZERO) => break,
                },
            }
        }

        // There is no blocker for the next sequence command. We can thus start executing the
        // next sequence command.

        match command {
            Command::Msg(msg) => tracing::info!("Command {i}: [LOG] {msg}"),
            Command::Warning(msg) => tracing::warn!("Command {i}: [WRN] {msg}"),
            Command::Error(msg) => tracing::error!("Command {i}: [ERR] {msg}"),
            Command::Wait(duration) => {
                tracing::info!("Command {i}: Sleeping for {:2}s", duration.as_secs_f32());
                blocker = Some(Blocker::Wait(Instant::now() + *duration));
            }
            Command::ToState(state) => return ControlFlow::Break(state.clone()),
            Command::ActuatorOpen(actuator) => {
                let Some(settable) = settables.get_mut(actuator) else {
                    // Ignore when the actuator doesn't exist.
                    tracing::error!(
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
                    tracing::error!(
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
                    tracing::error!(
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
                    tracing::error!(
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
                let Some(sensor) = sensors.get_pressure_datapoint(name) else {
                    tracing::error!(
                        "got a pressure sensor {name:?} whose name is valid but datapoint isn't",
                    );
                    return ControlFlow::Continue(());
                };
                pressure_watcher.add_watcher(
                    name.clone(),
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
                let Some(sensor) = sensors.get_temperature_datapoint(name) else {
                    tracing::error!(
                        "got a temperature sensor {name:?} whose name is valid but datapoint isn't",
                    );
                    return ControlFlow::Continue(());
                };
                temperature_watcher.add_watcher(
                    name.clone(),
                    key.clone(),
                    sensor,
                    *min,
                    *max,
                    abort_state.clone(),
                );
            }
            Command::SensorIgnore { name, key } => {
                temperature_watcher.remove_watcher(name, key.as_ref());
                pressure_watcher.remove_watcher(name, key.as_ref());
            }
            Command::SensorDetect {
                name,
                value,
                rising,
                abort_state,
                patience,
                duration,
            } if name.is_pressure_sensor() => {
                let Some(sensor) = sensors.get_pressure_datapoint(name) else {
                    tracing::error!(
                        "got a pressure sensor {name:?} whose name is valid but datapoint isn't",
                    );
                    return ControlFlow::Continue(());
                };
                let detector =
                    SensorDetector::new(sensor, *value, *rising, abort_state, *patience, *duration);
                blocker = Some(Blocker::Detector(SensorDetectorKind::Pressure(detector)));
            }
            Command::SensorDetect {
                name,
                value,
                rising,
                abort_state,
                patience,
                duration,
            } if name.is_temperature_sensor() => {
                let Some(sensor) = sensors.get_temperature_datapoint(name) else {
                    tracing::error!(
                        "got a temperature sensor {name:?} whose name is valid but datapoint isn't",
                    );
                    return ControlFlow::Continue(());
                };
                let detector =
                    SensorDetector::new(sensor, *value, *rising, abort_state, *patience, *duration);
                blocker = Some(Blocker::Detector(SensorDetectorKind::Temperature(detector)));
            }
            Command::SensorWatch { name, .. } | Command::SensorDetect { name, .. } => {
                tracing::error!("got a sensor {name:?} that is neither temperature nor pressure");
                return ControlFlow::Continue(());
            }
        }
    }

    ControlFlow::Continue(())
}

pub enum SensorDetectorKind<'a> {
    Temperature(SensorDetector<'a, Celsius>),
    Pressure(SensorDetector<'a, BarG>),
}

impl SensorDetectorKind<'_> {
    pub async fn detect(&mut self) -> ControlFlow<StateName> {
        match self {
            SensorDetectorKind::Temperature(detector) => detector.detect().await,
            SensorDetectorKind::Pressure(detector) => detector.detect().await,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::ops::ControlFlow;

    use datatypes::actuator::Settable;
    use datatypes::units::{BarG, Celsius};
    use lib_mission::actuator_control::ActuatorStore;
    use lib_mission::state_machine::StateName;
    use watchable::Watchable;

    use crate::{Command, SensorNameExt, SensorStore, run_sequence};

    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    enum DummyValveName {}

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    enum DummySensorName {
        ChamberPressure,
    }

    impl SensorNameExt for DummySensorName {
        fn is_pressure_sensor(&self) -> bool {
            matches!(self, DummySensorName::ChamberPressure)
        }

        fn is_temperature_sensor(&self) -> bool {
            false
        }
    }

    /// Dummy actuator store with no settables; the tested sequences don't contain
    /// actuator commands.
    #[derive(Default)]
    struct DummyActuators;

    impl ActuatorStore<DummyValveName> for DummyActuators {
        fn as_settables(&self) -> BTreeMap<DummyValveName, Box<dyn Settable + Send>> {
            BTreeMap::new()
        }
    }

    #[derive(Default)]
    struct DummySensors {
        chamber_pressure: Watchable<BarG>,
    }

    impl SensorStore<DummySensorName> for DummySensors {
        fn get_pressure_datapoint<'a>(
            &'a self,
            name: &DummySensorName,
        ) -> Option<&'a Watchable<BarG>> {
            match name {
                DummySensorName::ChamberPressure => Some(&self.chamber_pressure),
            }
        }

        fn get_temperature_datapoint<'a>(
            &'a self,
            _name: &DummySensorName,
        ) -> Option<&'a Watchable<Celsius>> {
            None
        }
    }

    // Regression test for the `Blocker::None` branch: before executing the next sequence
    // command, we must check whether a watched sensor already reports a value outside its
    // allowed range and abort immediately, rather than proceeding to the next command.
    #[tokio::test]
    #[ignore = "hangs indefinitely"]
    async fn test_aborts_on_watcher_violation_pending_before_next_command() {
        let sensors = DummySensors::default();
        let actuators = DummyActuators;
        let abort_state = StateName::new_unchecked("abort").expect("state name should be valid");

        // The chamber pressure already violates the range that's about to be watched.
        sensors.chamber_pressure.replace(BarG(20.0));

        let commands = vec![
            Command::SensorWatch {
                name: DummySensorName::ChamberPressure,
                abort_state: abort_state.clone(),
                max: 10.0,
                min: 0.0,
                key: None,
            },
            Command::Msg("should never run".to_string()),
        ];

        let result = run_sequence(&commands, &actuators, &sensors).await;
        assert!(matches!(result, ControlFlow::Break(state) if state == abort_state));
    }

    #[tokio::test]
    #[ignore = "hangs indefinitely"]
    async fn test_continues_when_no_watcher_violation_pending_before_next_command() {
        let sensors = DummySensors::default();
        let actuators = DummyActuators;
        let abort_state = StateName::new_unchecked("abort").expect("state name should be valid");

        // The chamber pressure is within the range that's about to be watched.
        sensors.chamber_pressure.replace(BarG(5.0));

        let commands = vec![
            Command::SensorWatch {
                name: DummySensorName::ChamberPressure,
                abort_state,
                max: 10.0,
                min: 0.0,
                key: None,
            },
            Command::Msg("runs fine".to_string()),
        ];

        let result = run_sequence(&commands, &actuators, &sensors).await;
        assert_eq!(result, ControlFlow::Continue(()));
    }
}
