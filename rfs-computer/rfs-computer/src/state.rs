use datatypes::status::{ConfigHash, DownlinkTelemetryStats, UplinkTelemetryHealth};
use io_utils::Broadcast;
use lib_mission::state_machine::StateName;
use messages::Message;
use messages::datapoints::dp_rfs_computer::Message as RFSMessage;
use messages::datapoints::dp_rfs_computer::RfsComputerStatus;
use watchable::Watchable;

#[derive(Debug, Clone, Broadcast)]
#[broadcast(message_type = "Message")]
pub struct State {
    /// The current state machine state.
    #[broadcast(flatten)]
    pub state_machine_state: CurrentStateWrapper,

    /// First 5 hex chars of the config file Git object hash. Set once at startup.
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::ConfigHash(#value))")]
    #[broadcast(min_freq_hz = 0.2, max_freq_hz = 0.2)]
    pub hash_config: Watchable<ConfigHash>,

    /// If the system is in a software armed state.
    /// This will allow the ECU to control the igniter actuators.
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::Status(#value))")]
    #[broadcast(min_freq_hz = 1.0, max_freq_hz = 1.0)]
    pub rfs_computer_status: Watchable<RfsComputerStatus>, // more appropriate I think

    /// Statistics about the health of the telemetry uplink.
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::UplinkTelemetryHealth(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 1.)]
    pub uplink_telemetry_health: Watchable<UplinkTelemetryHealth>,

    /// Statistics about the telemetry downlink (bytes and messages sent).
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::DownlinkTelemetryStats(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 1.)]
    pub downlink_telemetry_stats: Watchable<DownlinkTelemetryStats>,

    // FIXME: unused now
    /// `true` if can is connected. `false` by default and is set if messages are received.
    pub is_can_connected: Watchable<bool>,
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
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::StateMachineState(#value))")]
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
