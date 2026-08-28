use std::ops::ControlFlow;

use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;
use sequences::Command;

use crate::io::input::SensorName;
use crate::io::output::ActuatorName;
use crate::state_logic::helpers::ThresholdWatcher;

/// This state runs a sequence until completion, invalid condition, manual transition or reached threshold.
#[derive(Debug)]
pub struct RunSequence {
    /// States where one can manually transition to out of the sequence.
    /// This will interrupt the sequence.
    pub manual_transitions: ManualTransitions,
    /// Thresholds on sensors to watch during the whole of the sequence.
    /// If any of them are violated, this will end the sequence and proceeds to `on_cancelled`.
    pub thresholds: ThresholdWatcher,
    /// The sequence to run.
    pub sequence: Vec<Command<ActuatorName, SensorName, StateName>>,
    /// The state to transition to when the sequence has been completed.
    pub on_completed: StateName,
    /// The state to transition to when a watched threshold has been violated.
    pub on_cancelled: StateName,
}

#[async_trait::async_trait]
impl StateLogic for RunSequence {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::RunSequence::run", skip_all, fields(state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!(?self, "Running a sequence");

        // Run the sequence or receive a manual transition.
        tokio::select! {
            next_state = self.manual_transitions.next(input, param) => next_state,
            violation = self.thresholds.watch(input) => {
                tracing::warn!("Violation: {violation}");
                self.on_cancelled.clone()
            }
            outcome = sequences::run_sequence(&self.sequence, output, input) => match outcome {
                ControlFlow::Continue(_) => self.on_completed.clone(),
                ControlFlow::Break(abort_state) => {
                    tracing::error!("Sequence aborted, transitioning into {abort_state}");
                    abort_state
                }
            },
        }
    }
}
