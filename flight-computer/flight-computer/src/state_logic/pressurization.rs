use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

use crate::io::types::{
    FuelPressurizationCompletedState, FuelPressurizationState,
    OxidizerPressurizationCompletedState, OxidizerPressurizationState, Tank,
};

/// The state sends the pressurization start signal for the fuel or oxidizer tank and then transitions to `on_done`.
#[derive(Debug)]
pub struct Pressurization {
    pub(crate) selected_tank: Tank,
    pub(crate) target_pressure: f32,
    pub(crate) on_done: StateName,
}

#[async_trait::async_trait]
impl StateLogic for Pressurization {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::Pressurization::run", skip_all, fields(
        state = % _p.current_state().0
    ))]
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        match self.selected_tank {
            Tank::Fuel => {
                // This is ugly as we set values in the input tree from the state logic. This is not intended by the architecture design.
                // But in this case it's important to reset those signals as we might run the sequence multiple times and therefore have to detect
                // the completion of the sequence multiple times.
                input
                    .fuel_pressurization_state
                    .replace(FuelPressurizationCompletedState::NotYetCompleted);
                output
                    .pressurize_fuel
                    .replace(FuelPressurizationState::Pressurized(self.target_pressure));
                tracing::info!("Starting fuel tank pressurization sequence");
            }
            Tank::Oxidizer => {
                // The same applies here as in the previous comment.
                input
                    .oxidizer_pressurization_state
                    .replace(OxidizerPressurizationCompletedState::NotYetCompleted);
                output
                    .pressurize_oxidizer
                    .replace(OxidizerPressurizationState::Pressurized(
                        self.target_pressure,
                    ));
                tracing::info!("Starting oxidizer tank pressurization sequence");
            }
        }

        self.on_done.clone()
    }
}
