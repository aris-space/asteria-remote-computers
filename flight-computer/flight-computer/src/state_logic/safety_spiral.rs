use crate::io::types::SteeringMotorRevolutions;
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

/// This state relaxes the motors to `motors_pull_value`
#[derive(Debug)]
pub struct SafetySpiral {
    pub(crate) motors_pull_value: SteeringMotorRevolutions,
    pub(crate) on_motors_pulled: StateName,
}

#[async_trait::async_trait]
impl StateLogic for SafetySpiral {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::SafetySpiral::run", skip_all, fields(self, state = %param.current_state().0))]
    async fn run(
        &self,
        _input: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        output
            .steering_motor_target_deflections
            .replace(Some(self.motors_pull_value.clone()));
        self.on_motors_pulled.clone()
    }
}
