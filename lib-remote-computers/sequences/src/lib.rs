use std::{
    ops::ControlFlow,
    time::{Duration, Instant},
};

use datatypes::units::{BarG, Celsius};
use lib_mission::state_machine::StateName;
use pss_drivers::{
    input::{SensorDatapointStore, SensorName},
    output::ValveName,
};
pub use sequence_parser::{Command, parse_sequence_str};

use crate::watch::SensorWatcher;

mod watch;

/// Run a sequence of [`Command`]s on a remote computer.
#[expect(clippy::too_many_lines)] // yeeeeah a bit ugly.
pub async fn run_sequence(
    sequence: impl IntoIterator<Item = &Command<ValveName, SensorName, StateName>>,
    actuators: &pss_drivers::output::PSSValves,
    sensors: &pss_drivers::input::PSSSensors,
) -> ControlFlow<StateName> {
    let mut pressure_watcher = SensorWatcher::<BarG>::new();
    let mut temperature_watcher = SensorWatcher::<Celsius>::new();
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

            let result = tokio::select! {
                biased;
                result = pressure_watcher.check() => result,
                result = temperature_watcher.check() => result,
                _ = tokio::time::sleep(duration) => break,
            };

            if let Some(abort_state) = result {
                return ControlFlow::Break(abort_state);
            }
            // We might still need to sleep to reach the wait deadline.
            should_wait = wait_deadline.is_some_and(|deadline| deadline > Instant::now());
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
                pressure_watcher.add_watcher(
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
                let sensor = sensors
                    .get_temperature_datapoint(name)
                    .expect("we ensured that the name is for a temperature sensor");
                temperature_watcher.add_watcher(
                    *name,
                    key.clone(),
                    sensor,
                    *min,
                    *max,
                    abort_state.clone(),
                );
            }
            Command::SensorIgnore { name, key } => {
                temperature_watcher.remove_watcher(*name, key.as_ref());
                pressure_watcher.remove_watcher(*name, key.as_ref());
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
