//! This crate provides [`run_sequence`], a generic function that will read sensors, and actuate commands
//! based off a parsed sequence.

use std::fmt::Debug;
use std::hash::Hash;
use std::ops::ControlFlow;
use std::time::Duration;

use crate::{detect::SensorDetector, watch::SensorWatcher};
use datatypes::units::{BarG, Celsius};
use lib_mission::{actuator_control::ActuatorStore, state_machine::StateName};
use watchable::Watchable;

pub use sequence_parser::{Command, parse_sequence_str};

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

    let mut settables = actuators.as_settables();

    // Iterate through the commands, executing them step by step,
    // ensuring that watchers are always checked if there could pass any time, or they change.
    for (i, command) in sequence.into_iter().enumerate() {
        match command {
            Command::Msg(msg) => tracing::info!("Command {i}: [LOG] {msg}"),
            Command::Warning(msg) => tracing::warn!("Command {i}: [WRN] {msg}"),
            Command::Error(msg) => tracing::error!("Command {i}: [ERR] {msg}"),
            Command::Wait(duration) => {
                tracing::info!("Command {i}: Sleeping for {:2}s", duration.as_secs_f32());
                tokio::select! {
                    biased;
                    state = pressure_watcher.watch() => return ControlFlow::Break(state),
                    state = temperature_watcher.watch() => return ControlFlow::Break(state),
                    _ = tokio::time::sleep(*duration) => {},
                };
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
                if let Err(err) = settable.set_value(0.0) {
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
                // The new watch might currenly be violated, thus quickly check that before proceeding
                tokio::select! {
                        biased;
                        state = pressure_watcher.watch() => return ControlFlow::Break(state),
                        // No watcher had an event; we can thus start executing the next command.
                        _ = tokio::time::sleep(Duration::ZERO) => {},
                };
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
                // The new watch might currenly be violated, thus quickly check that before proceeding
                tokio::select! {
                        biased;
                        state = temperature_watcher.watch() => return ControlFlow::Break(state),
                        // No watcher had an event; we can thus start executing the next command.
                        _ = tokio::time::sleep(Duration::ZERO) => {},
                };
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
                let mut detector =
                    SensorDetector::new(sensor, *value, *rising, abort_state, *patience, *duration);
                tokio::select! {
                    biased;
                    state = pressure_watcher.watch() => return ControlFlow::Break(state),
                    state = temperature_watcher.watch() => return ControlFlow::Break(state),
                    result = detector.detect() => {
                        // We got a detection event, either an error, which then cancels the sequence,
                        // or a continuation, such that we can proceed.
                        result?;
                    },
                };
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
                let mut detector =
                    SensorDetector::new(sensor, *value, *rising, abort_state, *patience, *duration);
                tokio::select! {
                    biased;
                    state = pressure_watcher.watch() => return ControlFlow::Break(state),
                    state = temperature_watcher.watch() => return ControlFlow::Break(state),
                    result = detector.detect() => {
                        // We got a detection event, either an error, which then cancels the sequence,
                        // or a continuation, such that we can proceed.
                        result?;
                    },
                };
            }
            Command::SensorWatch { name, .. } | Command::SensorDetect { name, .. } => {
                tracing::error!("got a sensor {name:?} that is neither temperature nor pressure");
                return ControlFlow::Continue(());
            }
        }
    }

    ControlFlow::Continue(())
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::ops::ControlFlow;
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    use datatypes::actuator::{OutOfRange, Settable};
    use datatypes::units::{BarG, Celsius};
    use lib_mission::actuator_control::ActuatorStore;
    use lib_mission::state_machine::StateName;
    use watchable::Watchable;

    use crate::{Command, SensorNameExt, SensorStore, run_sequence};

    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    enum DummyValveName {
        Valve,
    }

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

    /// A [`Settable`] that records the last value it was set to, for assertions.
    #[derive(Clone, Default)]
    struct RecordingSettable(Arc<Mutex<Option<f32>>>);

    impl RecordingSettable {
        fn last_value(&self) -> Option<f32> {
            *self.0.lock().expect("lock shouldn't be poisoned")
        }
    }

    impl Settable for RecordingSettable {
        fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
            *self.0.lock().expect("lock shouldn't be poisoned") = Some(value);
            Ok(())
        }
    }

    /// Dummy actuator store with a single [`DummyValveName::Valve`] settable, whose
    /// activation can be observed through `valve`.
    #[derive(Default)]
    struct DummyActuators {
        valve: RecordingSettable,
    }

    impl ActuatorStore<DummyValveName> for DummyActuators {
        fn as_settables(&self) -> BTreeMap<DummyValveName, Box<dyn Settable + Send>> {
            let mut map: BTreeMap<DummyValveName, Box<dyn Settable + Send>> = BTreeMap::new();
            map.insert(DummyValveName::Valve, Box::new(self.valve.clone()));
            map
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

    // Regression test for the `SensorWatch` branch: before executing the next sequence
    // command, we must check whether a watched sensor already reports a value outside its
    // allowed range and abort immediately, rather than proceeding to (and executing) the
    // next command.
    #[tokio::test]
    async fn test_aborts_on_watcher_violation_pending_before_next_command() {
        let sensors = DummySensors::default();
        let actuators = DummyActuators::default();
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
            Command::ActuatorOpen(DummyValveName::Valve),
        ];

        let result = run_sequence(&commands, &actuators, &sensors).await;
        assert!(matches!(result, ControlFlow::Break(state) if state == abort_state));
        assert_eq!(
            actuators.valve.last_value(),
            None,
            "the actuator command after the violated watch should never have run"
        );
    }

    // Smoke test for a sequence of purely informational/timing commands.
    #[tokio::test]
    async fn test_msg_wait_and_state_transition() {
        let sensors = DummySensors::default();
        let actuators = DummyActuators::default();
        let target_state = StateName::new_unchecked("next").expect("state name should be valid");

        let commands = vec![
            Command::Msg("starting".to_string()),
            Command::Warning("careful".to_string()),
            Command::Error("uh oh".to_string()),
            Command::Wait(Duration::from_millis(10)),
            Command::ToState(target_state.clone()),
        ];

        let result = run_sequence(&commands, &actuators, &sensors).await;
        assert!(matches!(result, ControlFlow::Break(state) if state == target_state));
    }

    // Smoke test for a sequence of actuator commands, checking that they all run and
    // reach the end of the sequence.
    #[tokio::test]
    async fn test_actuator_commands() {
        let sensors = DummySensors::default();
        let actuators = DummyActuators::default();

        let commands = vec![
            Command::ActuatorOpen(DummyValveName::Valve),
            Command::DprActivate {
                name: DummyValveName::Valve,
                setpoint: 12.,
            },
            Command::DprDeactivate(DummyValveName::Valve),
            Command::ActuatorClose(DummyValveName::Valve),
        ];

        let result = run_sequence(&commands, &actuators, &sensors).await;
        assert_eq!(result, ControlFlow::Continue(()));
        // The sequence ends with `ActuatorClose`, so the last observed value must be 0.0,
        // not 1.0 (that would mean `close` actuated the valve instead of closing it).
        assert_eq!(actuators.valve.last_value(), Some(0.0));
    }

    // Smoke test for a sequence that watches a sensor within range and then stops
    // watching it again, reaching the end of the sequence without aborting.
    #[tokio::test]
    async fn test_sensor_watch_and_ignore() {
        let sensors = DummySensors::default();
        let actuators = DummyActuators::default();
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
            Command::SensorIgnore {
                name: DummySensorName::ChamberPressure,
                key: None,
            },
            Command::Msg("runs fine".to_string()),
        ];

        let result = run_sequence(&commands, &actuators, &sensors).await;
        assert_eq!(result, ControlFlow::Continue(()));
    }
}
