use std::time::Duration;

use lib_mission::reactive::{IntoWatched, Watched as _, WatchedExt};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;

/// This state waits for manual transitions as long as the RFS is connected via CAN.
/// As soon as `rfs_can_timeout` time passed since the last message from the RFS,
/// transition to `on_timeout`.
#[derive(Debug)]
pub struct WaitWhileConnected {
    pub manual_transitions: ManualTransitions,
    pub rfs_can_timeout: Duration,
    pub on_timeout: StateName,
}

#[async_trait::async_trait]
impl StateLogic for WaitWhileConnected {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::WaitWhileConnected::run", skip_all, fields(state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!("Waiting for transitions while the RFS is connected");

        // Waits for either a manual transition or a the timeout.
        let mut clock = input.time.watched();
        let rfs_status_watched = input.rfs_computer_status.watched();
        tokio::select! {
            next_state = self.manual_transitions.next(input, param) => next_state,
            _ = clock.next_satisfying(
                move |t| *t >= rfs_status_watched.current().1 + self.rfs_can_timeout
            ) => self.on_timeout.clone(),
        }
    }
}
