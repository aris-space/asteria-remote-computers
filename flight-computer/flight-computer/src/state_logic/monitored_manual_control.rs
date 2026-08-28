use std::sync::Arc;
use std::time::Duration;

use crate::state_logic::helpers::ThresholdWatcher;
use lib_mission::actuator_control::ActuatorStore;
use lib_mission::actuator_control::command::{ActuatorControl, NeverReturns};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::telemetry::await_telemetry_loss;
use lib_mission::transition::ManualTransitions;
use pss_drivers::output::ValveName;

/// This state gives manual control for the configured actuators,
/// and otherwise waits for a manual transition or a threshold violation
#[derive(Debug)]
pub struct MonitoredManualControl {
    /// Where we can manually transition
    pub manual_transitions: ManualTransitions,
    /// The name of the actuators and their logic which can be activated.
    pub allowed_actuators: Arc<ActuatorControl<ValveName>>,
    /// If the last telemetry message is older than this, transition to `on_violation`.
    pub telemetry_loss_threshold: Duration,
    /// Thresholds on sensors to watch during the whole operations.
    /// If any of them are violated, this will end the sequence and proceeds to `on_violation`.
    pub thresholds: ThresholdWatcher,
    /// The state to transition to when a threshold has been violated.
    pub on_violation: StateName,
}

#[async_trait::async_trait]
impl StateLogic for MonitoredManualControl {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::MonitoredManualControl::run", skip_all, fields(state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!(?self, "Entering a monitored manual control state");

        // Prepares a background task to listen for (and act upon) manual actuator inputs.
        let actuator_future = self.allowed_actuators.act(
            input.state.current_state.get().0 as usize,
            &input.actuator_command,
            output.valves.as_settables(),
        );
        assert_future::<NeverReturns>(&actuator_future);

        // Then waits for a manual transition or violation.
        tokio::select! {
            next_state = self.manual_transitions.next(input, param) => next_state,
            violation = self.thresholds.watch(input) => {
                tracing::warn!("threshold violation: {violation}");
                self.on_violation.clone()
            }
            _ = await_telemetry_loss(&input.telemetry_health, &input.time, self.telemetry_loss_threshold) => {
                tracing::warn!("telemetry loss detected");
                self.on_violation.clone()
            }
            _ = actuator_future => unreachable!("act cannot return normally"),
        }
    }
}

/// Just a helper function to ensure the futures we're using have the right type
fn assert_future<T>(_future: &impl std::future::Future<Output = T>) {}
