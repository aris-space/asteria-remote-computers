use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::telemetry::await_telemetry_loss;
use lib_mission::transition::ManualTransitions;
use std::time::Duration;

use crate::state_logic::helpers::ThresholdWatcher;

/// This state waits for a manual transition or a threshold violation.
#[derive(Debug)]
pub struct MonitoredWaiting {
    /// Where we can manually transition
    pub manual_transitions: ManualTransitions,
    /// Thresholds on sensors to watch during waiting.
    /// If any of them are violated, this will proceed to `on_violation`.
    pub thresholds: ThresholdWatcher,
    /// If the last telemetry message is older than this, transition to `on_violation`.
    pub telemetry_loss_threshold: Duration,
    /// The state to transition to when a threshold has been violated.
    pub on_violation: StateName,
    /// The state to transition to when telemetry has been lost.
    pub on_telemetry_loss: StateName,
}

#[async_trait::async_trait]
impl StateLogic for MonitoredWaiting {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::MonitoredWaiting::run", skip_all, fields(state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!(?self, "Entering a monitored waiting state");

        // Then waits for a manual transition or violation.
        tokio::select! {
            next_state = self.manual_transitions.next(input, param) => next_state,
            violation = self.thresholds.watch(input) => {
                tracing::warn!("threshold violation: {violation}");
                self.on_violation.clone()
            }
            _ = await_telemetry_loss(&input.telemetry_health, &input.time, self.telemetry_loss_threshold) => {
                tracing::warn!("telemetry loss detected");
                self.on_telemetry_loss.clone()
            }
        }
    }
}
