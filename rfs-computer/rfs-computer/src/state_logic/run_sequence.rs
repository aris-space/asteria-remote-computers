use std::ops::ControlFlow;

use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;
use sequences::Command;

use crate::io::input::SensorName;
use crate::io::output::ActuatorName;

/// This state runs a sequence until completion, invalid condition or manual transition.
#[derive(Debug)]
pub struct RunSequence {
    /// States where one can manually transition to out of the sequence.
    /// This will interrupt the sequence.
    pub manual_transitions: ManualTransitions,
    /// The sequence to run.
    pub sequence: Vec<Command<ActuatorName, SensorName, StateName>>,
    /// The state to transition to when the sequence has been completed.
    pub on_completed: StateName,
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
