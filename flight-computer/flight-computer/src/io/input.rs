//! Contains [`InputTree`]

use crate::io::types::{
    Acceleration, Location, MainDeploymentState, Orientation, SeparationState,
    SteeringMotorRevolutions, Velocity,
};
use hermes_can::messages::board_status::{ArmingState, RfsComputerStatus};
use hermes_can::messages::sensor_data::{EnvironmentalData, RailStatus24V, RailStatus5V};
use lib_mission::telemetry::TelemetryHealth;
use lib_mission::util::time::MissionTime;
use lib_mission::{actuator_control, transition};
use pss_drivers::input::{PSSSensors, PSSValves};
use watchable::Watchable;

#[derive(Debug, Default, Clone)]
pub struct RawUnprocessedData {
    /// The newest environmental data from the sensors.
    pub environmental_data: Watchable<EnvironmentalData>,
    /// The power status of the 24V rail of the AV stack.
    pub rail_status_24v: Watchable<RailStatus24V>,
    /// The power status of the 5V rail of the AV stack.
    pub rail_status_5v: Watchable<RailStatus5V>,
}

/// Holds all information we gather from the rest of the rocket and environment.
#[derive(Debug, Default, Clone)]
pub struct InputTree {
    /// The current time, as as unix timestamp wrapper.
    pub time: Watchable<MissionTime>,
    /// The newest command from the ground station.
    pub transition_command: Watchable<transition::Command>,
    /// The newest actuator command from the ground station.
    pub actuator_command:
        Watchable<Option<actuator_control::Command<pss_drivers::output::ValveName>>>,
    /// Statistics about the health of the telemetry uplink.
    pub telemetry_health: Watchable<TelemetryHealth>,
    /// The processed health report for the avionics boards.
    pub boards_health: BoardsHealthyAsOf,
    /// The last received status of the RFS Computer and the timestamp when it was received.
    pub rfs_computer_status: Watchable<(RfsComputerStatus, MissionTime)>,
    /// Hardware arming as detected by the recovery board.
    pub recovery_armed: Watchable<ArmingState>,
    /// Main valve hardware arming as detected by the engine control board.
    pub ecu_armed: Watchable<ArmingState>,
    /// The newest raw data from the AV stack sensors.
    pub raw_unprocessed_data: RawUnprocessedData,

    /// The most recently provided positions of the motors.
    pub motors_position: Watchable<SteeringMotorRevolutions>,

    /// The state of whether CATS has triggered separation.
    /// Might also be `Separated` for a short time during start up.
    pub cats_separation: Watchable<SeparationState>,
    /// The state of whether CATS has triggered deployment.
    /// Might also be `Deployed` for a short time during start up.
    pub cats_deployment: Watchable<MainDeploymentState>,

    // GNC fields, see the types for documentation.
    // FIXME: add the angular velocity
    pub location: Watchable<Location>,
    pub velocity: Watchable<Velocity>,
    pub acceleration: Watchable<Acceleration>,
    pub orientation: Watchable<Orientation>,

    /// The valve states of the PSS.
    pub pss_valves: PSSValves,
    /// The sensor readings from the PSS.
    pub pss_sensors: PSSSensors,
}

/// The processed health report for the avionics boards.
///
/// If `Some(t)` the board was last observed to be healthy, which happend at `t`.
/// If `None` the board was last observed unhealthy or not yet observed.
#[derive(Debug, Default, Clone)]
pub struct BoardsHealthyAsOf {
    pub sensor_carrier: Watchable<Option<MissionTime>>,
    pub power_board: Watchable<Option<MissionTime>>,
    pub recovery_board: Watchable<Option<MissionTime>>,
    pub fuel_control_board: Watchable<Option<MissionTime>>,
    pub oxidizer_control_board: Watchable<Option<MissionTime>>,
    pub engine_control_board: Watchable<Option<MissionTime>>,
}

/// Allow common code to read the ground station transition command.
impl transition::GetTransitionCommand for InputTree {
    fn transition_cmd_watcher(
        &self,
    ) -> impl lib_mission::reactive::IntoWatched<transition::Command> + Send {
        &self.transition_command
    }
}
