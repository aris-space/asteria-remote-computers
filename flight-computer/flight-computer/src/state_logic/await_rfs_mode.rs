use lib_mission::reactive::{IntoWatched, WatchedExt};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum RfsPSSMode {
    /// The default: The RFS is passive and doesn't command PSS valves.
    RFSPassive,
    /// The RFS takes over PSS control.
    RFSActive,
}

/// This state waits until the RFS has the expected `mode` and then continues to `on_detected`.
/// Also listens for manual transitions during this wait.
#[derive(Debug)]
pub struct AwaitRFSMode {
    pub manual_transitions: ManualTransitions,
    pub mode: RfsPSSMode,
    pub on_detected: StateName,
}

#[async_trait::async_trait]
impl StateLogic for AwaitRFSMode {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::AwaitRFSMode::run", skip_all, fields(self.mode, state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!("Waiting for FC to be passive");

        // Waits for either a manual transition or a change in the rfs status that indicates it is now passive.
        let mut rfs_status_watched = input.boards_status.rfs_computer.watched();
        tokio::select! {
            next_state = self.manual_transitions.next(input, param) => next_state,
            _ = rfs_status_watched.next_satisfying(|rep|
                // Wait for new messages with the expected PSS control
                (input.time.get() < rep.timestamp + std::time::Duration::from_secs(2)) // Should be sent every second
                && match &self.mode {
                    RfsPSSMode::RFSPassive => !rep.current_status.has_pss_control,
                    RfsPSSMode::RFSActive => rep.current_status.has_pss_control,
                }
            ) => self.on_detected.clone(),
        }
    }
}
