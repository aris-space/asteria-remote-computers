use std::{
    fmt::Debug,
    hash::Hash,
    ops::ControlFlow,
    time::{Duration, Instant},
};

use datatypes::units::{BarG, Celsius};
use lib_mission::state_machine::StateName;
use pss_drivers::{
    input::{SensorNameExt, SensorStore},
    output::ActuatorStore,
};
pub use sequence_parser::{Command, parse_sequence_str};

use crate::{detect::SensorDetector, watch::SensorWatcher};

mod detect;
mod watch;

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
