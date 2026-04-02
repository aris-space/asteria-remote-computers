use crate::io::types::SteeringMotorRevolutions;
use lib_mission::reactive::{IntoWatched, WatchedExt};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;

/// This state releases the pre-brake hook by pulling the motors to `motors_pull_value` as soon as we enter it.
/// As soon as the motors are pulled beyond `motors_ok_threshhold`, we transition into `on_motors_ok_threshhold_reached`.
#[derive(Debug)]
pub struct HookRelease {
    pub(crate) motors_pull_value: SteeringMotorRevolutions,
    pub(crate) motors_ok_threshold: SteeringMotorRevolutions,
    pub(crate) on_motors_ok_threshhold_reached: StateName,
    pub(crate) manual_transitions: ManualTransitions,
}

#[async_trait::async_trait]
impl StateLogic for HookRelease {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::HookRelease::run", skip_all, fields(self, state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        // First we move the motors to `motors_pull_value`
        output
            .steering_motor_target_deflections
            .replace(Some(self.motors_pull_value.clone()));
        // And then we wait for the motors to exceed the `motors_ok_threshold` or for a manual transition command
        let mut motor_position_watcher = input.motors_position.watched();
        tokio::select! {
            next = self.manual_transitions.next(input, param) => {
                return next;
            },
            _ = motor_position_watcher.next_satisfying(|pos| {
                pos.left_pos >= self.motors_ok_threshold.left_motor_steps
                    && pos.right_pos >= self.motors_ok_threshold.right_motor_steps
            }) => {
                return self.on_motors_ok_threshhold_reached.clone();
            },
        }
    }
}
