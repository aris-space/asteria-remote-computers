use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

use crate::io::types::AbortIgnitionState;

/// The state sends the engine abort signal and then transitions to `on_done`.
#[derive(Debug)]
pub struct AbortIgnition {
    pub(crate) on_done: StateName,
}

#[async_trait::async_trait]
impl StateLogic for AbortIgnition {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::AbortIgnition::run", skip_all, fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        _input: &Self::Input,
        output: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::warn!("Aborting ignition sequence");

        // We use `replace` here to make sure the driver triggers in all cases.
        output.abort_ignition.replace(AbortIgnitionState::Aborted);

        self.on_done.clone()
    }
}
