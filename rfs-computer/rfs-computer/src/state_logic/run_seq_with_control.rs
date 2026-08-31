use std::ops::ControlFlow;
use std::time::Duration;

use lib_mission::actuator_control::ActuatorStore;
use lib_mission::actuator_control::command::{ActuatorControl, NeverReturns};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::telemetry::await_telemetry_loss;
use lib_mission::transition::ManualTransitions;
use sequences::Command;

use crate::io::input::SensorName;
use crate::io::output::ActuatorName;

/// This state runs a sequence, but at the same time also gives manual control for the configured actuators,
/// and otherwise waits for a manual transition or telemetry loss.
///
/// This is used for Parallel Filling, where N2 is filled using a sequence, while LOX is still manually filled.
/// At this time, telemetry must also be monitored and manual aborts must be possible.
#[derive(Debug)]
pub struct RunSeqWithControl {
    /// Where we can manually transition
    pub manual_transitions: ManualTransitions,
    /// The name of the actuators and their logic which can be activated.
    pub actuator_control: ActuatorControl<ActuatorName>,
    /// If the last telemetry message is older than this, transition to `on_violation`.
    pub telemetry_loss_threshold: Duration,
    /// The state to transition to when telemetry has been lost.
    pub on_violation: StateName,
    /// The sequence to run.
    pub sequence: Vec<Command<ActuatorName, SensorName, StateName>>,
    /// The state to transition to when the sequence has been completed.
    pub on_completed: StateName,
}

#[async_trait::async_trait]
impl StateLogic for RunSeqWithControl {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::RunSeqWithControl::run", skip_all, fields(state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!(?self, "Entering a monitored manual control state");

        // Prepares a background task to listen for (and act upon) manual actuator inputs.
        let actuator_future = self.actuator_control.act(
            param.current_state().1 as usize,
            &input.actuator_command,
            output.as_settables(),
        );
        assert_future::<NeverReturns>(&actuator_future);

        // Then waits for a manual transition or violation.
        tokio::select! {
            next_state = self.manual_transitions.next(input, param) => next_state,
            _ = await_telemetry_loss(&param
                .parameters.uplink_telemetry_health, &input.time, self.telemetry_loss_threshold) => {
                tracing::warn!("telemetry loss detected");
                self.on_violation.clone()
            }
            outcome = sequences::run_sequence(&self.sequence, output, input) => match outcome {
                ControlFlow::Continue(_) => self.on_completed.clone(),
                ControlFlow::Break(abort_state) => {
                    tracing::error!("Sequence aborted, transitioning into {abort_state}");
                    abort_state
                }
            },
            _ = actuator_future => unreachable!("act cannot return normally"),
        }
    }
}

/// Just a helper function to ensure the futures we're using have the right type
fn assert_future<T>(_future: &impl Future<Output = T>) {}
