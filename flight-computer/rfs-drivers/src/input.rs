use datatypes::actuator::{DigitalOut, NormallyClosedValve};
use datatypes::units::{BarG, Celsius};
use io_utils::Broadcast;
use io_utils::Collector;
use messages::Message;
use messages::datapoints::dp_rfs_computer::Message as RFSCMessage;
use watchable::Watchable;

/// The states of the actuators that are exposed.
#[derive(Debug, Default, Clone, Collector, Broadcast)]
#[broadcast(message_type = "Message")]
#[collector(
    message_type = "Message",
    update_expr = "let _ = #field.update(#value);"
)]
pub struct Actuators {
    /// The state of the GOX purge valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::GoxPurgeValveState(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 15.)]
    pub gox_purge_valve: Watchable<NormallyClosedValve>,
    /// The state of the H2 purge valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::H2PurgeValveState(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 15.)]
    pub h2_purge_valve: Watchable<NormallyClosedValve>,
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::H2IgniterValveState(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 15.)]
    pub h2_igniter_valve: Watchable<NormallyClosedValve>,
    /// The state of the GOX igniter valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::GoxIgniterValveState(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 15.)]
    pub gox_igniter_valve: Watchable<NormallyClosedValve>,
    /// The state of the igniter spark.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::IgniterSparkPlugState(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 15.)]
    pub igniter_spark: Watchable<DigitalOut>,
}

/// Exposed sensors on the RFS.
#[derive(Debug, Default, Clone, Collector, Broadcast)]
#[broadcast(message_type = "Message")]
#[collector(
    message_type = "Message",
    update_expr = "let _ = #field.update(#value);"
)]
pub struct Sensors {
    /// The last received pressure of the igniter box.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::IgniterPressure(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 10.)]
    pub igniter_pressure: Watchable<BarG>,
    /// The last received temperature of the igniter box.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::IgniterTemperature(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 3.)]
    pub igniter_temperature: Watchable<Celsius>,
}
