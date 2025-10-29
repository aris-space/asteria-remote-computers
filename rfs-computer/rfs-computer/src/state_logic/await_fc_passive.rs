use lib_mission::reactive::{IntoWatched, WatchedExt};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;

/// This state waits until the FC is passive and then continues to `on_fc_passive`.
/// Also listens for manual transitions during this wait.
#[derive(Debug)]
pub struct AwaitFCPassive {
    pub manual_transitions: ManualTransitions,
    pub on_fc_passive: StateName,
}

#[async_trait::async_trait]
impl StateLogic for AwaitFCPassive {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::AwaitFCPassive::run", skip_all, fields(state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!("Waiting for FC to be passive");

        // Waits for either a manual transition or a change in the fc status that indicates it is now passive.
        let mut fc_status_watched = input.flight_computer_status.watched();
        tokio::select! {
            next_state = self.manual_transitions.next(input, param) => next_state,
            _ = fc_status_watched.next_satisfying(|s| !s.has_pss_control) => self.on_fc_passive.clone(),
        }
    }
}
