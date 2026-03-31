use crate::util::timer::Timer;
use lib_mission::reactive::{IntoWatched, WatchedExt};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;
use std::time::Duration;

/// This state watches the acceleration in z direction. If it reaches `acceleration_limit` in m/s^2
/// and remains over this limit for more than `required_duration` we transition to `on_liftoff_detected`.
/// If the ecu firing state changes to Aborted we transition to `on_ecu_aborted`.
#[derive(Debug)]
pub struct LiftoffDetection {
    pub(crate) acceleration_limit: f32,
    pub(crate) required_duration: Duration,
    pub(crate) on_liftoff_detected: StateName,
    #[allow(dead_code)] // todo
    pub(crate) on_aborted: StateName,
    pub(crate) manual_transitions: ManualTransitions,
}

#[async_trait::async_trait]
impl StateLogic for LiftoffDetection {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::LiftoffDetection::run", skip_all, fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        let mut imu_watcher = input.imu.watched();

        loop {
            tokio::select! {
                next = self.manual_transitions.next(input, _p) => {
                    tracing::info!(cause = "manual", "transitioning");
                    return next;
                },
                // Wait until the condition gets fulfilled. The z axis points towards the tail of the rocket.
                // Because absolute values are used the direction of acceleration on the z axis does not matter
                _ = imu_watcher.next_satisfying(|a| a.acceleration_z.abs() >= self.acceleration_limit) => {
                    // Once inside, we race a `required_duration` timer against the condition breaking.
                    let mut timer = Timer::new(input.time.watched(), self.required_duration);

                    tokio::select! {
                        // If acceleration fell again below the threshold before `required_duration` we loop and watch again.
                        _ = imu_watcher.next_satisfying(|a| a.acceleration_z.abs() < self.acceleration_limit) => {
                            tracing::warn!("acceleration spiked but not sustained over {}s", self.required_duration.as_secs_f32());
                        },
                        // If `required_duration` passed without acceleration dropping below the threshold, we transition to `on_liftoff_detected`
                        _ = timer.wait() => {
                            tracing::info!(cause = "liftoff detected", "transitioning");
                            return self.on_liftoff_detected.clone();
                        },
                        // Also still wait for transitions, which might be delayed otherwise
                        next = self.manual_transitions.next(input, _p) => {
                            tracing::info!(cause = "manual during race", "transitioning");
                            return next;
                        },
                    }
                }
                // TODO: Move to `on_ecu_aborted` if firing aborted
            }
        }
    }
}
