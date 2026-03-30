//! Contains [`InputTree`]

use crate::io::types::{MainDeploymentState, SeparationState};

use datapoints::dp_engine_control_board::Message as ECUMessage;
use datapoints::dp_fuel_control_board::Message as FCBMessage;
use datapoints::dp_oxidizer_control_board::Message as OCBMessage;
use datapoints::dp_power_board::Message as PWBMessage;
use datapoints::dp_recovery_board::Message as RECMessage;
use datapoints::dp_recovery_board::SteeringPositions;
use datapoints::dp_rfs_computer::Message as RFSCMessage;
use datapoints::dp_sensor_carrier::Message as SCMessage;
use datapoints::dp_sensor_carrier::{
    EnvironmentalData, ImuData, OrientationData, PositionData, VelocityData,
};
use datatypes::status::{ArmingState, UplinkTelemetryHealth};
use datatypes::units::RailStatus;
use io_utils::{Broadcast, Collector};
use lib_mission::util::time::MissionTime;
use lib_mission::{actuator_control, transition};
use messages::datapoints;
use messages::Message;
use pss_drivers::input::{PSSSensors, PSSValves};
use watchable::Watchable;

#[derive(Debug, Default, Clone, Broadcast, Collector)]
#[broadcast(message_type = "Message")]
#[collector(
    message_type = "Message",
    update_expr = "let _ = #field.update(#value);"
)]
pub struct RawUnprocessedData {
    /// The newest environmental data from the sensors.
    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::EnvironmentalData(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub environmental_data: Watchable<EnvironmentalData>,
    /// The power status of the 24V rail of the AV stack.
    #[datapoint(pattern = "Message::PowerBoard(PWBMessage::RailStatus24V(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub rail_status_24v: Watchable<RailStatus>,
    /// The power status of the 5V rail of the AV stack.
    #[datapoint(pattern = "Message::PowerBoard(PWBMessage::RailStatus5V(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub rail_status_5v: Watchable<RailStatus>,
}

/// Holds all information we gather from the rest of the rocket and environment.
#[derive(Debug, Default, Clone, Broadcast, Collector)]
#[broadcast(message_type = "messages::Message")]
#[collector(
    message_type = "Message",
    update_expr = "let _ = #field.update(#value);"
)]
pub struct InputTree {
    /// The current time, as as unix timestamp wrapper.
    pub time: Watchable<MissionTime>,
    /// The newest command from the ground station.
    pub transition_command: Watchable<transition::Command>,
    /// The newest actuator command from the ground station.
    pub actuator_command:
        Watchable<Option<actuator_control::Command<pss_drivers::output::ValveName>>>,
    /// Statistics about the health of the telemetry uplink.
    pub telemetry_health: Watchable<UplinkTelemetryHealth>,
    /// The health report for the avionics boards.
    #[collector(flatten)]
    #[broadcast(flatten)]
    pub boards_status: BoardsStatus,
    /// Hardware arming as detected by the recovery board.
    pub recovery_armed: Watchable<ArmingState>,
    /// Main valve hardware arming as detected by the engine control board.
    pub ecu_armed: Watchable<ArmingState>,
    /// The newest raw data from the AV stack sensors.
    #[broadcast(flatten)]
    #[collector(flatten)]
    pub raw_unprocessed_data: RawUnprocessedData,

    /// The most recently provided positions of the motors.
    #[datapoint(pattern = "Message::RecoveryBoard(RECMessage::SteeringActualPositions(#value))")]
    #[broadcast(min_freq_hz = 10., max_freq_hz = 10.)]
    pub motors_position: Watchable<SteeringPositions>,

    /// The state of whether CATS has triggered separation.
    /// Might also be `Separated` for a short time during start up.
    pub cats_separation: Watchable<SeparationState>,
    /// The state of whether CATS has triggered deployment.
    /// Might also be `Deployed` for a short time during start up.
    pub cats_deployment: Watchable<MainDeploymentState>,

    // GNC fields, see the types for documentation.
    // TODO: add the angular velocity
    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::PositionData(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub location: Watchable<PositionData>,
    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::VelocityData(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub velocity: Watchable<VelocityData>,
    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::ImuData(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 10.)]
    pub acceleration: Watchable<ImuData>,
    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::OrientationData(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 10.)]
    pub orientation: Watchable<OrientationData>,

    /// The valve states of the PSS.
    pub pss_valves: PSSValves,
    /// The sensor readings from the PSS.
    pub pss_sensors: PSSSensors,
}

/// A health report which also tracks the previous status, the time,
/// and if it was default constructed.
#[derive(Debug, Default, Clone)]
pub struct StatusReport<T> {
    pub fresh: bool,
    pub timestamp: MissionTime,
    pub current_status: T,
    pub previous_status: T,
}

/// Pushes a new status update to the report.
fn update_status_report<T>(report: &Watchable<StatusReport<T>>, time: MissionTime, new_status: T) {
    use std::ops::DerefMut;

    let mut report = report.write();
    let report = report.deref_mut();
    report.fresh = true;
    report.timestamp = time;
    std::mem::swap(&mut report.previous_status, &mut report.current_status);
    report.current_status = new_status;
}

/// The health report for the avionics boards.
///
/// This keeps track of two reports and the time, to see changes in the error count,
/// as well a stale reports.
#[derive(Debug, Default, Clone, Broadcast, Collector)]
#[broadcast(message_type = "Message", extract_expr = "#value.current_status")]
#[collector(
    message_type = "Message",
    update_expr = "let _ = update_status_report(&#field, self.time.get(), #value);"
)]
pub struct BoardsStatus {
    /// The current time, as as unix timestamp wrapper.
    /// This is a clone from `InputTree` and used for updating the timestamps.
    pub time: Watchable<MissionTime>,

    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::Status(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub rfs_computer: Watchable<StatusReport<datapoints::dp_rfs_computer::RfsComputerStatus>>,

    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::BoardStatus(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub sensor_carrier: Watchable<StatusReport<datapoints::dp_sensor_carrier::SensorCarrierStatus>>,

    #[datapoint(pattern = "Message::PowerBoard(PWBMessage::BoardStatus(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub power_board: Watchable<StatusReport<datatypes::status::StatusCommonMessage>>,

    #[datapoint(pattern = "Message::RecoveryBoard(RECMessage::BoardStatus(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub recovery_board: Watchable<StatusReport<datapoints::dp_recovery_board::RecoveryBoardStatus>>,

    #[datapoint(pattern = "Message::FuelControlBoard(FCBMessage::BoardStatus(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub fuel_control_board:
        Watchable<StatusReport<datapoints::dp_fuel_control_board::FuelControlBoardStatus>>,

    #[datapoint(pattern = "Message::OxidizerControlBoard(OCBMessage::BoardStatus(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub oxidizer_control_board:
        Watchable<StatusReport<datapoints::dp_oxidizer_control_board::OxidizerControlBoardStatus>>,

    #[datapoint(pattern = "Message::EngineControlBoard(ECUMessage::BoardStatus(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub engine_control_board:
        Watchable<StatusReport<datapoints::dp_engine_control_board::EngineControlBoardStatus>>,
}

/// Allow common code to read the ground station transition command.
impl transition::GetTransitionCommand for InputTree {
    fn transition_cmd_watcher(
        &self,
    ) -> impl lib_mission::reactive::IntoWatched<transition::Command> + Send {
        &self.transition_command
    }
}
