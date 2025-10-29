use std::time::Duration;

use lib_mission::reactive::{IntoWatched, Watched as _, WatchedExt};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;

use crate::state_logic::await_rfs_mode::RfsPSSMode;

/// This state waits until the RFS has the expected `mode` and then continues to `on_detected`.
/// Also listens for manual transitions during this wait.
///
/// As soon as `rfs_can_timeout` time passed since the last message from the RFS,
/// transition to `on_timeout`.
#[derive(Debug)]
pub struct AwaitRFSModeWhileConnected {
    pub manual_transitions: ManualTransitions,
    pub mode: RfsPSSMode,
    pub on_detected: StateName,
    pub rfs_can_timeout: Duration,
    pub on_timeout: StateName,
}

#[async_trait::async_trait]
impl StateLogic for AwaitRFSModeWhileConnected {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::AwaitRFSModeWhileConnected::run", skip_all, fields(self.mode, state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!("Waiting for FC to be passive");

        // Waits for either a manual transition or a change in the rfs status that indicates it is now passive.
        let mut rfs_status_watched = input.rfs_computer_status.watched();
        let mut clock = input.time.watched();
        let rfs_status_watched_2 = input.rfs_computer_status.watched();
        tokio::select! {
            next_state = self.manual_transitions.next(input, param) => next_state,
            _ = rfs_status_watched.next_satisfying(|(s, t)|
                // Wait for new messages with the expected PSS control
                (input.time.get() < *t + std::time::Duration::from_secs(2)) // Should be sent every second
                && match &self.mode {
                    RfsPSSMode::RFSPassive => !s.has_pss_control,
                    RfsPSSMode::RFSActive => s.has_pss_control,
                }
            ) => self.on_detected.clone(),
            _ = clock.next_satisfying(
                move |t| *t >= rfs_status_watched_2.current().1 + self.rfs_can_timeout
            ) => self.on_timeout.clone(),
        }
    }
}
