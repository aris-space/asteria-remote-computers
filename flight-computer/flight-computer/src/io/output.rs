//! Contains [`OutputTree`].

use pss_drivers::output::PSSValves;
use watchable::Watchable;

use super::types::{
    CameraPowerState, MainDeploymentState, RecoveryArmingState, SeparationState,
    SteeringMotorRevolutions,
};

/// Holds the things we want to set
#[derive(Debug, Clone, Default)]
pub struct OutputTree {
    /// When turning [`SeparationState::Separated`], the drogue should be separated.
    pub separate_drogue: Watchable<SeparationState>,
    /// When turning [`MainDeploymentState::Deployed`], the chute should be deployed.
    pub deploy_main_chute: Watchable<MainDeploymentState>,

    /// The target revolutions of the steering motors for the parachute.
    /// Whenever this changes, the recovery board will start moving toward this target.
    /// While this is `None`, the motors have not yet moved.
    /// This is to prevent early messages that would start the watchdog.
    pub steering_motor_target_deflections: Watchable<Option<SteeringMotorRevolutions>>,
    /// Whether to give power for the steering motor, deployment and separation actuators.
    pub recovery_actuators_armed: Watchable<RecoveryArmingState>,
    /// Whether to give power to the camera
    pub camera_powered_on: Watchable<CameraPowerState>,

    /// The target states of the tank and engine valves.
    pub valves: PSSValves,
}
