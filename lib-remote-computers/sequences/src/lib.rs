use std::ops::ControlFlow;

use pss_drivers::output::ValveName;
pub use sequence_parser::{Command, parse_sequence_str};

#[derive(Debug, thiserror::Error)]
pub enum SequenceError {
    #[error("Actuator '{0:?}' is not part of the output settables")]
    MissingOutputSettable(ValveName),
}

type SensorName = String;
type StateName = String;

/// Run a sequence of [`Command`]s on a remote computer.
pub async fn run_sequence(
    sequence: impl IntoIterator<Item = &Command<ValveName, SensorName, StateName>>,
    actuators: &pss_drivers::output::PSSValves,
    _sensors: &pss_drivers::input::PSSSensors,
) -> ControlFlow<SequenceError> {
    let mut settables = actuators.as_settables();
    for (i, command) in sequence.into_iter().enumerate() {
        match command {
            Command::Msg(msg) => tracing::info!("Command {i}: [LOG] {msg}"),
            Command::Warning(msg) => tracing::warn!("Command {i}: [WRN] {msg}"),
            Command::Error(msg) => tracing::error!("Command {i}: [ERR] {msg}"),
            Command::Wait(duration) => {
                tracing::info!("Command {i}: Sleeping for {:2}s", duration.as_secs_f32());
                tokio::time::sleep(*duration).await;
            }
            Command::ToState(_state) => todo!("unsupported sequence command"),
            Command::ActuatorOpen(actuator) => {
                if let Some(settable) = settables.get_mut(actuator) {
                    tracing::info!("Command {i}: Opening actuator {actuator:?}");
                    if let Err(err) = settable.set_value(1.0) {
                        tracing::error!("Command {i}: Could not open {actuator:?}: {err}");
                    }
                } else {
                    tracing::error!(
                        "Command {i}: Could not find settable for actuator {actuator:?} to open it"
                    );
                    return ControlFlow::Break(SequenceError::MissingOutputSettable(*actuator));
                }
            }
            Command::ActuatorClose(actuator) => {
                if let Some(settable) = settables.get_mut(actuator) {
                    tracing::info!("Command {i}: Closing actuator {actuator:?}");
                    if let Err(err) = settable.set_value(0.0) {
                        tracing::error!("Command {i}: Could not close {actuator:?}: {err}");
                    }
                } else {
                    tracing::error!(
                        "Command {i}: Could not find settable for actuator {actuator:?} to close it"
                    );
                    return ControlFlow::Break(SequenceError::MissingOutputSettable(*actuator));
                }
            }
            Command::DprActivate {
                name: actuator,
                setpoint,
            } => {
                if let Some(settable) = settables.get_mut(actuator) {
                    tracing::info!("Command {i}: Activating DPR actuator {actuator:?}");
                    if let Err(err) = settable.set_value(*setpoint) {
                        tracing::error!("Command {i}: Could not activate {actuator:?}: {err}");
                    }
                } else {
                    tracing::error!(
                        "Command {i}: Could not find settable for DPR actuator {actuator:?} to activate it"
                    );
                    return ControlFlow::Break(SequenceError::MissingOutputSettable(*actuator));
                }
            }
            Command::DprDeactivate(actuator) => {
                if let Some(settable) = settables.get_mut(actuator) {
                    tracing::info!("Command {i}: Deactivating DPR actuator {actuator:?}");
                    if let Err(err) = settable.set_value(f32::NAN) {
                        tracing::error!("Command {i}: Could not deactivate {actuator:?}: {err}");
                    }
                } else {
                    tracing::error!(
                        "Command {i}: Could not find settable for DPR actuator {actuator:?} to deactivate it"
                    );
                    return ControlFlow::Break(SequenceError::MissingOutputSettable(*actuator));
                }
            }
            Command::SensorWatch { .. } => todo!("unspported sequence command"),
            Command::SensorIgnore { .. } => todo!("unsupported sequence command"),
            Command::SensorDetect { .. } => todo!("unsupported sequence command"),
        }
    }

    ControlFlow::Continue(())
}
