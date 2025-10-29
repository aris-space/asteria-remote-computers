use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

use crate::io::types::{AbortFuelPressurizationState, AbortOxidizerPressurizationState};

/// The state sends the abort pressurization start signal for the fuel and oxidizer tank and then transitions to `on_done`.
#[derive(Debug)]
pub struct AbortPressurization {
    pub(crate) on_done: StateName,
}

#[async_trait::async_trait]
impl StateLogic for AbortPressurization {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::AbortPressurization::run", skip_all, fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        _input: &Self::Input,
        output: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        output
            .abort_fuel_pressurization
            .replace(AbortFuelPressurizationState::Aborted);
        output
            .abort_oxidizer_pressurization
            .replace(AbortOxidizerPressurizationState::Aborted);
        tracing::info!("Aborting oxidizer and fuel tank pressurization sequences");

        self.on_done.clone()
    }
}
