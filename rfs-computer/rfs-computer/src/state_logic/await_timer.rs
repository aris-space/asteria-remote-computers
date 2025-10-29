use lib_mission::reactive::{IntoWatched, WatchedExt};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;
use std::time::Duration;

/// This state waits for `duration` before transitioning to `on_timer`.
/// Used during Ignition.
#[derive(Debug)]
pub struct AwaitTimer {
    pub(crate) duration: Duration,
    pub(crate) on_timer: StateName,
    pub(crate) manual_transitions: ManualTransitions,
}

#[async_trait::async_trait]
impl StateLogic for AwaitTimer {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::AwaitTimer::run", skip(input, _p), fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        let start = input.time.get();
        let mut clock = input.time.watched();

        tokio::select! {
            next = self.manual_transitions.next(input, _p) => {
                tracing::info!(cause = "manual", "transitioning");
                return next;
            },
            _ = clock.next_satisfying(|t| *t > start + self.duration) => {
                tracing::info!(cause = "time trigger", "transitioning");
                return self.on_timer.clone();
            },
        }
    }
}
