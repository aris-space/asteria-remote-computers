//! Contains [`InputTree`]

use datatypes::actuator::ThrottleableValve;
use datatypes::status::{DownlinkTelemetryStats, ManualStateTransition, UplinkTelemetryHealth};
use datatypes::units::BarG;
use io_utils::{Broadcast, Collector};
use lib_mission::state_machine::StateName;
use lib_mission::util::time::MissionTime;
use lib_mission::{actuator_control, transition};
use messages::Message;
use messages::datapoints::dp_flight_computer::FlightComputerStatus;
use messages::datapoints::dp_flight_computer::Message as FCMessage;
use messages::datapoints::dp_rfs_computer::Message as RFSCMessage;
use pss_drivers::input::{PSSSensors, PSSValves};
use uremote_electronics::input as uremote;
use watchable::Watchable;

use super::output::ActuatorName;

/// Holds all information we gather from the rest of the system and environment.
#[derive(Debug, Default, Clone, Broadcast, Collector)]
#[broadcast(message_type = "Message")]
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
    /// The newest transition command from the ground station.
    pub transition_command: Watchable<ManualStateTransition>,
    /// The newest actuator command from the ground station.
    pub actuator_command: Watchable<Option<actuator_control::Command<ActuatorName>>>,
    /// Statistics about the health of the telemetry uplink.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::UplinkTelemetryHealth(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 1.)]
    pub uplink_telemetry_health: Watchable<UplinkTelemetryHealth>,
    /// Statistics about the telemetry downlink (bytes and messages sent).
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::DownlinkTelemetryStats(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 1.)]
    pub downlink_telemetry_stats: Watchable<DownlinkTelemetryStats>,
    // FIXME: unused now
    /// `true` if can is connected. `false` by default and is set if messages are received.
    pub is_can_connected: Watchable<bool>,
    // FIXME: this is not wired up (see FC input probably)
    /// The processed health report for the rocket boards.
    pub rocket_health: BoardsHealthyAsOf,
    /// The last received status of the flight computer.
    #[datapoint(pattern = "Message::FlightComputer(FCMessage::Status(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub flight_computer_status: Watchable<FlightComputerStatus>,

    /// The sensors connected to the uremote.
    #[broadcast(flatten)]
    pub uremote_sensors: uremote::Sensors,
    /// The state of the actuators connected to the uremote.
    #[broadcast(flatten)]
    pub uremote_actuators: uremote::Actuators,

    // The state of the pressure sensors that communicate on the serial bus:
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::N2PreBoosterPressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub n2_pre_booster_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::H2PurgePressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub h2_purge_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::N2PostBoosterPressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub n2_post_booster_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::LoxPressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub lox_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::PurgePressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub purge_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::PneumaticPressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub pneumatic_pressure: Watchable<BarG>,
    // Igniter system:
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::GoxPressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub gox_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::H2Pressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub h2_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::IgniterPressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub igniter_pressure: Watchable<BarG>,

    /// The current state for the lox fill valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::LoxFillValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub lox_fill_valve: Watchable<ThrottleableValve>,

    /// The valve states of the PSS in the rocket.
    #[broadcast(flatten)]
    #[collector(flatten)]
    pub pss_valves: PSSValves,
    /// The sensor readings from the PSS in the rocket.
    #[broadcast(flatten)]
    #[collector(flatten)]
    pub pss_sensors: PSSSensors,
}

/// The processed health report for the rocket boards.
///
/// If `Some(t)` the board was last observed to be healthy, which happend at `t`.
/// If `None` the board was last observed unhealthy or not yet observed.
#[derive(Debug, Default, Clone)]
pub struct BoardsHealthyAsOf {
    pub flight_computer: Watchable<Option<MissionTime>>,
    pub fuel_control_board: Watchable<Option<MissionTime>>,
    pub oxidizer_control_board: Watchable<Option<MissionTime>>,
    pub engine_control_board: Watchable<Option<MissionTime>>,
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
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::StateMachineState(#value))")]
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

/// Allow common code to read the ground station transition command.
impl transition::GetTransitionCommand for InputTree {
    fn transition_cmd_watcher(
        &self,
    ) -> impl lib_mission::reactive::IntoWatched<datatypes::status::ManualStateTransition> + Send
    {
        &self.transition_command
    }
}
