use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

use crate::io::types::{ECUFiringState, IgnitionState};

/// The state sends the engine start signal and then transitions to `on_done`.
#[derive(Debug)]
pub struct Ignition {
    pub(crate) on_done: StateName,
}

#[async_trait::async_trait]
impl StateLogic for Ignition {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::Ignition::run", skip_all, fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!("Starting ignition sequence");

        // Reset the input to allow another abort. A side effect of event message / state enum incompatiblity.
        input
            .ecu_firing_state
            .replace(ECUFiringState::NotYetInitiated);
        // We use `replace` here to make sure the driver triggers in all cases.
        output.ignite.replace(IgnitionState::Ignited);

        self.on_done.clone()
    }
}
