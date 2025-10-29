use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

use crate::io::types::RecoveryArmingState;

/// This state sets the `recovery_actuators_armed` in the output tree to the `arming_state` value.
/// After setting this value, it immediately transitions into the `on_state_set` state.
///
/// This can be used to Arm / Disarm the recovery board.
#[derive(Debug)]
pub struct ArmRecovery {
    pub(crate) arming_state: RecoveryArmingState,
    pub(crate) on_state_set: StateName,
}

#[async_trait::async_trait]
impl StateLogic for ArmRecovery {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::ArmRecovery::run", skip_all, fields(self, state = %param.current_state().0))]
    async fn run(
        &self,
        _: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        output.recovery_actuators_armed.replace(self.arming_state);
        self.on_state_set.clone()
    }
}
