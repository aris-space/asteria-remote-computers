use datatypes::status::ArmingState;
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

/// This state proceeds to `on_armed` if the RFS' ignition is armed, otherwise proceeds to `on_not_armed`.
/// If no connection exists within 2 sec, it is considered not armed.
#[derive(Debug)]
pub struct ReadRFSArming {
    pub on_armed: StateName,
    pub on_not_armed: StateName,
}

#[async_trait::async_trait]
impl StateLogic for ReadRFSArming {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::ReadRFSArming::run", skip_all, fields(state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        let report = input.boards_status.rfs_computer.get();
        let status = report.current_status;
        let time = report.timestamp;

        if input.time.get() > time + std::time::Duration::from_secs(2) {
            // Should be sent every second
            tracing::warn!("No up to date info on RFS arming, considering not-armed");
            self.on_not_armed.clone()
        } else {
            match status.is_ignition_armed {
                ArmingState::Safe => {
                    tracing::warn!("Detected RFS as not armed");
                    self.on_not_armed.clone()
                }
                ArmingState::Armed => {
                    tracing::warn!("Detected RFS as armed");
                    self.on_armed.clone()
                }
            }
        }
    }
}
