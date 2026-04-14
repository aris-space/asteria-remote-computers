//! Contains [`OutputTree`].

use datatypes::status::ConfigHash;
use dp_flight_computer::FlightComputerStatus;
use io_utils::Broadcast;
use messages::Message;
use messages::datapoints::dp_flight_computer;
use pss_drivers::output::PSSValves;
use watchable::Watchable;

use super::types::{
    CameraPowerState, MainDeploymentState, RecoveryArmingState, SeparationState,
    SteeringMotorRevolutions,
};

/// Holds the things we want to set
#[derive(Debug, Clone, Default, Broadcast)]
#[broadcast(message_type = "messages::Message")]
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

    /// First 5 hex chars of the config file Git object hash. Set once at startup.
    #[datapoint(
        pattern = "Message::FlightComputer(dp_flight_computer::Message::ConfigHash(#value))"
    )]
    #[broadcast(min_freq_hz = 0.2, max_freq_hz = 0.2)]
    pub hash_config: Watchable<ConfigHash>,

    /// Aggregated FC status. Updated by the CAN status driver.
    #[datapoint(pattern = "Message::FlightComputer(dp_flight_computer::Message::Status(#value))")]
    #[broadcast(min_freq_hz = 1.0, max_freq_hz = 1.0)]
    pub fc_status: Watchable<FlightComputerStatus>,
}
