use lib_mission::{
    actuator_control::valves::NormallyClosedValve,
    state_machine::{AdditionalState, StateLogic, StateName},
};
use uremote_electronics::types::SparkPlug;

/// This state sets the software arming state to disarmed and shuts off the igniter valves.
///
/// This is used to disallow the ECU to control to the igniter and stop it should it already has started ignition.
#[derive(Debug)]
pub struct DisarmIgniter {
    pub on_done: StateName,
}

#[async_trait::async_trait]
impl StateLogic for DisarmIgniter {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::DisarmIgniter::run", skip_all, fields(?self, state = %param.current_state().0))]
    async fn run(
        &self,
        _input: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        output.is_software_armed.replace(false);
        output.rfs_actuators.igniter_spark.replace(SparkPlug::Off);
        output
            .rfs_actuators
            .h2_igniter_valve
            .replace(NormallyClosedValve::Closed);
        output
            .rfs_actuators
            .gox_igniter_valve
            .replace(NormallyClosedValve::Closed);

        self.on_done.clone()
    }
}
