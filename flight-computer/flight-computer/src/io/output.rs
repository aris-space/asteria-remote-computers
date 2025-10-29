//! Contains [`OutputTree`].

use pss_drivers::output::PSSValves;
use watchable::Watchable;

use super::types::{
    AbortFuelPressurizationState, AbortIgnitionState, AbortOxidizerPressurizationState,
    CameraPowerState, FuelPressurizationState, IgnitionState, MainDeploymentState,
    OxidizerPressurizationState, RecoveryArmingState, SeparationState, SteeringMotorRevolutions,
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
    /// When turning [`IgnitionState::Ignited`], the ECU should start the ignition sequence.
    pub ignite: Watchable<IgnitionState>,
    /// When turning [`AbortIgnitionState::Aborted`], the ECU should abort the ignition sequence.
    pub abort_ignition: Watchable<AbortIgnitionState>,

    /// Whether to give power to the camera
    pub camera_powered_on: Watchable<CameraPowerState>,

    /// When turning [`FuelPressurizationState::Pressurized`], the ECU should start the fuel pressurization sequence.
    pub pressurize_fuel: Watchable<FuelPressurizationState>,
    /// When turning [`OxidizerPressurizationState::Pressurized`], the ECU should start the oxidizer pressurization sequence.
    pub pressurize_oxidizer: Watchable<OxidizerPressurizationState>,
    /// When turning [`AbortFuelPressurizationState::Aborted`], the ECU should abort the fuel pressurization sequence.
    pub abort_fuel_pressurization: Watchable<AbortFuelPressurizationState>,
    /// When turning [`AbortOxidizerPressurizationState::Aborted`], the ECU should abort the oxidizer pressurization sequence.
    pub abort_oxidizer_pressurization: Watchable<AbortOxidizerPressurizationState>,
    /// The target states of the tank and engine valves.
    pub valves: PSSValves,
}
