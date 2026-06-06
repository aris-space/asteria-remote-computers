use datatypes::actuator::{DigitalOut, InvertedDigitalOut, NormallyClosedValve, NormallyOpenValve};
use datatypes::sensor::DigitalIn;
use datatypes::units::Celsius;
use io_utils::Broadcast;
use messages::Message;
use messages::datapoints::dp_rfs_computer::Message as RFSCMessage;
use watchable::Watchable;

#[derive(Debug, Default, Clone, Broadcast)]
#[broadcast(message_type = "Message")]
pub struct Sensors {
    /// The last received temperature of the Lox line.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::LoxTemperature(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 3.)]
    pub lox_temperature: Watchable<Celsius>,
    /// The last received temperature of the igniter.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::IgniterTemperature(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 3.)]
    pub igniter_temperature: Watchable<Celsius>,
    /// The last received temperature of the cabinet thermocouple.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::CabinetTemperature(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub cabinet_temperature: Watchable<Celsius>,
    /// The last received temperature of the ambient thermocouple.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::AmbientTemperature(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub ambient_temperature: Watchable<Celsius>,
    /// `On` if proximity of the lox decoupler to the retracted state is detected.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::LoxDecouplerProximity(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 5.)]
    pub lox_decoupler_proximity: Watchable<DigitalIn>,
    /// `On` if proximity of the n2 decoupler to the retracted state is detected.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::N2DecouplerProximity(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 5.)]
    pub n2_decoupler_proximity: Watchable<DigitalIn>,
    /// `On` if the valves receive power.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::ValvesArmed(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 5.)]
    pub valves_armed: Watchable<DigitalIn>,
    /// `On` if the spark plug receives power.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::IgniterArmed(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 5.)]
    pub igniter_armed: Watchable<DigitalIn>,
}

/// The states of the actuators connected to the u-remote.
///
/// For most valves this is based on whether they are powered or not.
/// H2 Bleed and Gox Bleed have additional feedback.
#[derive(Debug, Default, Clone, Broadcast)]
#[broadcast(message_type = "Message")]
#[allow(clippy::struct_field_names)]
pub struct Actuators {
    /// The state of the MAXON_BOARD_POWER.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::MaxonBoardPowerState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub maxon_board_power: Watchable<InvertedDigitalOut>,
    /// The state of the CUSTOM1.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::Custom1State(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub custom1: Watchable<DigitalOut>,
    /// The state of the CUSTOM2.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::Custom2State(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub custom2: Watchable<DigitalOut>,
    /// The state of the CUSTOM3.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::Custom3State(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub custom3: Watchable<DigitalOut>,

    /// The state of the GOX purge valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::GoxPurgeValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub gox_purge_valve: Watchable<NormallyClosedValve>,
    /// The state of the H2 purge valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::H2PurgeValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub h2_purge_valve: Watchable<NormallyClosedValve>,
    /// The state of the N2 bleed valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::N2BleedValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub n2_bleed_valve: Watchable<NormallyOpenValve>,
    /// The state of the LOX purge valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::LoxPurgeValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub lox_purge_valve: Watchable<NormallyClosedValve>,
    /// The state of the GOX bleed valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::GoxBleedValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub gox_bleed_valve: Watchable<NormallyOpenValve>,
    /// The state of the N2 purge valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::N2PurgeValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub n2_purge_valve: Watchable<NormallyClosedValve>,
    /// The state of the N2 fill valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::N2FillValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub n2_fill_valve: Watchable<NormallyClosedValve>,
    /// The state of the H2 bleed valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::H2BleedValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub h2_bleed_valve: Watchable<NormallyOpenValve>,
    /// The state of the LOX bleed valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::LoxBleedValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub lox_bleed_valve: Watchable<NormallyClosedValve>,
    /// The state of the Booster valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::BoosterValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub booster_valve: Watchable<NormallyClosedValve>,
    /// The state of the H2 igniter valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::H2IgniterValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub h2_igniter_valve: Watchable<NormallyClosedValve>,
    /// The state of the GOX igniter valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::GoxIgniterValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub gox_igniter_valve: Watchable<NormallyClosedValve>,
    /// The state of the igniter spark.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::IgniterSparkPlugState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub igniter_spark: Watchable<DigitalOut>,
}
