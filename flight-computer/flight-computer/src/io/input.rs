//! Contains [`InputTree`]

use crate::io::types::{MainDeploymentState, SeparationState};

use datapoints::dp_backplane::Message as BPMessage;
use datapoints::dp_engine_control_board::Message as ECUMessage;
use datapoints::dp_flight_computer::Message as FCMessage;
use datapoints::dp_fuel_control_board::Message as FCBMessage;
use datapoints::dp_oxidizer_control_board::Message as OCBMessage;
use datapoints::dp_recovery_board::Message as RECMessage;
use datapoints::dp_recovery_board::SteeringPositions;
use datapoints::dp_rfs_computer::Message as RFSCMessage;
use datapoints::dp_sensor_carrier::Message as SCMessage;
use datapoints::dp_sensor_carrier::{
    EnvironmentalData, ImuData, MagnetometerData, OrientationData, PositionData, VelocityData,
};
use datatypes::status::{
    ArmingState, BuildInformationCommon, DownlinkTelemetryStats, UplinkTelemetryHealth,
};
use datatypes::units::HPa;
use datatypes::units::RailStatus;
use datatypes::units::{BarG, Celsius};
use io_utils::{Broadcast, Collector};
use lib_mission::state_machine::StateName;
use lib_mission::util::time::MissionTime;
use lib_mission::{actuator_control, transition};
use messages::datapoints;
use messages::datapoints::dp_flight_computer::__message_meta_types::ManualStateTransition;
use messages::Message;
use pss_drivers::input::{PSSSensors, PSSValves};
use sequences::{SensorNameExt, SensorStore};
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
    #[datapoint(pattern = "Message::Backplane(BPMessage::RailStatus24V(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub rail_status_24v: Watchable<RailStatus>,
    /// The power status of the 5V rail of the AV stack.
    #[datapoint(pattern = "Message::Backplane(BPMessage::RailStatus5V(#value))")]
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
    /// The current state machine state.
    /// TODO: this is maybe part of a larger "Self-State", not Input, but works for now.
    #[broadcast(flatten)]
    pub state: CurrentStateWrapper,

    /// The current time, as as unix timestamp wrapper.
    pub time: Watchable<MissionTime>,
    /// The newest command from the ground station.
    pub transition_command: Watchable<ManualStateTransition>,
    /// The newest actuator command from the ground station.
    pub actuator_command:
        Watchable<Option<actuator_control::Command<crate::io::output::ActuatorName>>>,

    /// Statistics about the health of the telemetry uplink.
    #[datapoint(pattern = "Message::FlightComputer(FCMessage::UplinkTelemetryHealth(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 1.)]
    pub telemetry_health: Watchable<UplinkTelemetryHealth>,
    /// Statistics about the telemetry downlink (bytes and messages sent).
    #[datapoint(pattern = "Message::FlightComputer(FCMessage::DownlinkTelemetryStats(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 1.)]
    pub downlink_telemetry_stats: Watchable<DownlinkTelemetryStats>,
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
    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::PositionData(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub location: Watchable<PositionData>,
    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::VelocityData(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub velocity: Watchable<VelocityData>,
    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::ImuData(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 10.)]
    pub imu: Watchable<ImuData>,
    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::OrientationData(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 10.)]
    pub orientation: Watchable<OrientationData>,
    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::MagnetometerData(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 4.)]
    pub magnetometer: Watchable<MagnetometerData>,
    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::PressureData(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 4.)]
    pub pressure: Watchable<HPa>,

    /// Build information reported by each board on the CAN bus.
    #[broadcast(flatten)]
    #[collector(flatten)]
    pub boards_build_info: BoardsBuildInfo,

    /// The valve states of the PSS.
    #[broadcast(flatten)]
    #[collector(flatten)]
    pub pss_valves: PSSValves,

    /// The sensor readings from the PSS.
    #[broadcast(flatten)]
    #[collector(flatten)]
    pub pss_sensors: PSSSensors,

    /// The states of the RFS actuators the FC can command over CAN.
    #[broadcast(flatten)]
    #[collector(flatten)]
    pub rfs_actuators: rfs_drivers::input::Actuators,

    /// The sensor readings the RFS exposes to the FC over CAN.
    #[broadcast(flatten)]
    #[collector(flatten)]
    pub rfs_sensors: rfs_drivers::input::Sensors,
}

/// The current state machine wrapped in a struct such that we can use `#[broadcast(extract_expr)]`,
/// and it is not collected.
#[derive(Debug, Clone, Broadcast)]
#[broadcast(
    message_type = "Message",
    extract_expr = "lib_mission::telemetry::encode_current_state(#value)"
)]
pub struct CurrentStateWrapper {
    /// *Do not write to this!* This is exposed only for `Broadcast`
    #[datapoint(pattern = "Message::FlightComputer(FCMessage::StateMachineState(#value))")]
    #[broadcast(min_freq_hz = 3., max_freq_hz = 10.)]
    pub current_state: watchable::Watchable<(u32, StateName)>,
}

/// A temporary default constructor, will be overwritten in `main.rs`
impl Default for CurrentStateWrapper {
    fn default() -> Self {
        Self {
            current_state: Watchable::new((
                0,
                StateName::new_unchecked("TEMPORARY").expect("short enough"),
            )),
        }
    }
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
    let mut report = report.write();
    let report = &mut *report; // deref such that we can split the borrow for `swap`.
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

    #[datapoint(pattern = "Message::Backplane(BPMessage::BoardStatus(#value))")]
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

/// Build information for every board on the CAN bus.
#[derive(Debug, Default, Clone, Broadcast, Collector)]
#[broadcast(message_type = "Message")]
#[collector(
    message_type = "Message",
    update_expr = "let _ = #field.update(#value);"
)]
pub struct BoardsBuildInfo {
    #[datapoint(pattern = "Message::FlightComputer(FCMessage::BuildInfo(#value))")]
    #[broadcast(min_freq_hz = 0.05, max_freq_hz = 0.2)]
    pub flight_computer: Watchable<BuildInformationCommon>,

    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::BuildInfo(#value))")]
    #[broadcast(min_freq_hz = 0.05, max_freq_hz = 0.2)]
    pub rfs_computer: Watchable<BuildInformationCommon>,

    #[datapoint(pattern = "Message::SensorCarrier(SCMessage::BuildInfo(#value))")]
    #[broadcast(min_freq_hz = 0.05, max_freq_hz = 0.2)]
    pub sensor_carrier: Watchable<BuildInformationCommon>,

    // 
    #[datapoint(pattern = "Message::RecoveryBoard(RECMessage::BuildInfo(#value))")]
    #[broadcast(min_freq_hz = 0.05, max_freq_hz = 0.2)]
    pub recovery_board: Watchable<BuildInformationCommon>,

    #[datapoint(pattern = "Message::FuelControlBoard(FCBMessage::BuildInfo(#value))")]
    #[broadcast(min_freq_hz = 0.05, max_freq_hz = 0.2)]
    pub fuel_control_board: Watchable<BuildInformationCommon>,

    #[datapoint(pattern = "Message::OxidizerControlBoard(OCBMessage::BuildInfo(#value))")]
    #[broadcast(min_freq_hz = 0.05, max_freq_hz = 0.2)]
    pub oxidizer_control_board: Watchable<BuildInformationCommon>,

    #[datapoint(pattern = "Message::EngineControlBoard(ECUMessage::BuildInfo(#value))")]
    #[broadcast(min_freq_hz = 0.05, max_freq_hz = 0.2)]
    pub engine_control_board: Watchable<BuildInformationCommon>,
}

/// Allow common code to read the ground station transition command.
impl transition::GetTransitionCommand for InputTree {
    fn transition_cmd_watcher(
        &self,
    ) -> impl lib_mission::reactive::IntoWatched<datatypes::status::ManualStateTransition> + Send
    {
        &self.transition_command
    }
}

/// A sensor readable by the state machine, be it on the PSS in the rocket or
/// exposed by the RFS to the flight computer over CAN.
///
/// See the matching types in [`pss_drivers::input::SensorName`] and
/// [`rfs_drivers::input::SensorName`].
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(untagged)]
pub enum SensorName {
    PSSSensor(pss_drivers::input::SensorName),
    RfsSensor(rfs_drivers::input::SensorName),
}

impl SensorNameExt for SensorName {
    fn is_pressure_sensor(&self) -> bool {
        match self {
            SensorName::PSSSensor(name) => name.is_pressure_sensor(),
            SensorName::RfsSensor(name) => name.is_pressure_sensor(),
        }
    }

    fn is_temperature_sensor(&self) -> bool {
        match self {
            SensorName::PSSSensor(name) => name.is_temperature_sensor(),
            SensorName::RfsSensor(name) => name.is_temperature_sensor(),
        }
    }
}

/// Implement parsing based on serde.
impl std::str::FromStr for SensorName {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        use serde::Deserialize as _;
        Self::deserialize(<&str as serde::de::IntoDeserializer>::into_deserializer(s))
            .map_err(|e| e.to_string())
    }
}

impl SensorStore<SensorName> for InputTree {
    fn get_pressure_datapoint<'a>(&'a self, name: &SensorName) -> Option<&'a Watchable<BarG>> {
        match name {
            SensorName::PSSSensor(name) => self.pss_sensors.get_pressure_datapoint(name),
            SensorName::RfsSensor(name) => self.rfs_sensors.get_pressure_datapoint(name),
        }
    }

    fn get_temperature_datapoint<'a>(
        &'a self,
        name: &SensorName,
    ) -> Option<&'a Watchable<Celsius>> {
        match name {
            SensorName::PSSSensor(name) => self.pss_sensors.get_temperature_datapoint(name),
            SensorName::RfsSensor(name) => self.rfs_sensors.get_temperature_datapoint(name),
        }
    }
}

#[test]
fn test_sensor_name_from_str() {
    assert_eq!(
        "eng_cc_p".parse::<SensorName>().expect("must parse"),
        SensorName::PSSSensor(pss_drivers::input::SensorName::eng_cc_p)
    );
    assert_eq!(
        "ign_cc_p".parse::<SensorName>().expect("must parse"),
        SensorName::RfsSensor(rfs_drivers::input::SensorName::ign_cc_p)
    );
    assert!("not_a_sensor".parse::<SensorName>().is_err());
}
