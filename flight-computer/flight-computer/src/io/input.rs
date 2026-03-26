//! Contains [`InputTree`]

use crate::io::types::{
    MainDeploymentState, SeparationState, SteeringMotorRevolutions,
};
use io_utils::Broadcast;
use datatypes::status::ArmingState;
use datatypes::units::RailStatus;
use dp_sensor_carrier::{EnvironmentalData, ImuData, OrientationData, PositionData, VelocityData};
use dp_rfs_computer::RfsComputerStatus;
use lib_mission::telemetry::TelemetryHealth;
use lib_mission::util::time::MissionTime;
use lib_mission::{actuator_control, transition};
use messages::datapoints::dp_rfs_computer::RfsComputerStatus;
use messages::datapoints::dp_sensor_carrier::EnvironmentalData;
use pss_drivers::input::{PSSSensors, PSSValves};
use watchable::Watchable;

#[derive(Broadcast, Debug, Default, Clone)]
#[broadcast(message_type = "messages::Message", loop_type = "BroadcastLoop")]
pub struct RawUnprocessedData {
    /// The newest environmental data from the sensors.
    #[datapoint(pattern = "messages::Message::SensorCarrier(dp_sensor_carrier::Message::EnvironmentalData(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 1.)]
    pub environmental_data: Watchable<EnvironmentalData>,
    /// The power status of the 24V rail of the AV stack.
    #[datapoint(pattern = "messages::Message::PowerBoard(dp_power_board::Message::RailStatus24V(#value))")]
    #[broadcast(min_freq_hz = 10., max_freq_hz = 10.)]
    pub rail_status_24v: Watchable<RailStatus>,
    /// The power status of the 5V rail of the AV stack.
    #[datapoint(pattern = "messages::Message::PowerBoard(dp_power_board::Message::RailStatus5V(#value))")]
    #[broadcast(min_freq_hz = 10., max_freq_hz = 10.)]
    pub rail_status_5v: Watchable<RailStatus>,
}

/// Holds all information we gather from the rest of the rocket and environment.
#[derive(Broadcast, Debug, Default, Clone)]
#[broadcast(message_type = "messages::Message", loop_type = "BroadcastLoop")]
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
    #[broadcast(flatten)]
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
    #[datapoint(pattern = "messages::Message::SensorCarrier(dp_sensor_carrier::Message::PositionData(#value))")]
    #[broadcast(min_freq_hz = 20., max_freq_hz = 20.)]
    pub location: Watchable<PositionData>,
    
    #[datapoint(pattern = "messages::Message::SensorCarrier(dp_sensor_carrier::Message::VelocityData(#value))")]
    #[broadcast(min_freq_hz = 20., max_freq_hz = 20.)]
    pub velocity: Watchable<VelocityData>,
    
    #[datapoint(pattern = "messages::Message::SensorCarrier(dp_sensor_carrier::Message::ImuData(#value))")]
    #[broadcast(min_freq_hz = 40., max_freq_hz = 40.)]
    pub acceleration: Watchable<ImuData>,
    
    #[datapoint(pattern = "messages::Message::SensorCarrier(dp_sensor_carrier::Message::OrientationData(#value))")]
    #[broadcast(min_freq_hz = 40., max_freq_hz = 40.)]
    pub orientation: Watchable<OrientationData>,

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
