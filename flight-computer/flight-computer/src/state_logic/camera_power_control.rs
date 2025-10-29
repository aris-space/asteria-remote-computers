use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

use crate::io::types::CameraPowerState;

/// This state sets the `camera_powered_on` in the output tree to the `camera_power_state` value.
/// After setting this value, it immediately transitions into the `on_state_set` state.
///
/// This can be used to turn the camera On / Off.
#[derive(Debug)]
pub struct CameraPowerControl {
    pub(crate) camera_power_state: CameraPowerState,
    pub(crate) on_state_set: StateName,
}

#[async_trait::async_trait]
impl StateLogic for CameraPowerControl {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::CameraPowerControl::run", skip_all, fields(self, state = %param.current_state().0))]
    async fn run(
        &self,
        _: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        output.camera_powered_on.replace(self.camera_power_state);
        self.on_state_set.clone()
    }
}
