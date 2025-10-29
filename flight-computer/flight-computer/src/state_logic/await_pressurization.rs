use crate::io::types::{
    FuelPressurizationCompletedState, OxidizerPressurizationCompletedState, Tank,
};
use lib_mission::reactive::{IntoWatched, WatchedExt};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;

/// The state awaits the pressurization completed signal for the `selected_tank` and then transitions to `on_pressurization_completed`.
#[derive(Debug)]
pub struct AwaitPressurization {
    pub manual_transitions: ManualTransitions,
    pub(crate) selected_tank: Tank,
    pub(crate) on_pressurization_completed: StateName,
}

#[async_trait::async_trait]
impl StateLogic for AwaitPressurization {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::AwaitPressurization::run", skip_all, fields(state = %p.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _output: &Self::Output,
        p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        let mut fuel_pressurization_status_watched = input.fuel_pressurization_state.watched();
        let mut oxidizer_pressurization_status_watched =
            input.oxidizer_pressurization_state.watched();

        match self.selected_tank {
            Tank::Fuel => {
                tokio::select! {
                    next_state = self.manual_transitions.next(input, p) => next_state,
                     _ = fuel_pressurization_status_watched.next_satisfying(|s| *s == FuelPressurizationCompletedState::Completed) => self.on_pressurization_completed.clone(),
                }
            }
            Tank::Oxidizer => {
                tokio::select! {
                    next_state = self.manual_transitions.next(input, p) => next_state,
                     _ = oxidizer_pressurization_status_watched.next_satisfying(|s| *s == OxidizerPressurizationCompletedState::Completed) => self.on_pressurization_completed.clone(),
                }
            }
        }
    }
}
