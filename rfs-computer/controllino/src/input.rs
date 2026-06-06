use datatypes::actuator::{DigitalOut, InvertedDigitalOut, NormallyClosedValve};
use datatypes::sensor::DigitalIn;
use datatypes::status::ArmingState;
use datatypes::units::{BarG, Celsius};
use io_utils::Broadcast;
use messages::Message;
use messages::datapoints::dp_rfs_computer::Message as RFSCMessage;
use watchable::Watchable;

#[derive(Debug, Default, Clone, Broadcast)]
#[broadcast(message_type = "Message")]
pub struct Sensors {
    /// `On` if the valves of the SMART box are getting power.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::ValvesArmedSmartBox(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 5.)]
    pub valve_power_interlock: Watchable<ArmingState>,
    /// `On` if the rocket is receiving power and connection is detected.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::RocketConnected(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 5.)]
    pub rocket_connection_detect: Watchable<DigitalIn>,
    /// `On` if proximity of the lox decoupler to the retracted state is detected.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::LoxDecouplerProximity(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 5.)]
    pub lox_decoupler_proximity: Watchable<DigitalIn>,
    /// `On` if proximity of the n2 decoupler to the retracted state is detected.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::N2DecouplerProximity(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 5.)]
    pub n2_decoupler_proximity: Watchable<DigitalIn>,

    /// The last received pressure of the igniter box.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::IgniterPressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub igniter_pressure: Watchable<BarG>,
    /// The last received temperature of the igniter box.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::IgniterTemperature(#value))")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 3.)]
    pub igniter_temperature: Watchable<Celsius>,
    /// The last received ambient temperature inside the SMART box.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::SmartBoxTemperature(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 2.)]
    pub smart_box_temperature: Watchable<Celsius>,
}

/// The states of the actuators connected to the SMART box Controllino.
#[derive(Debug, Default, Clone, Broadcast)]
#[broadcast(message_type = "Message")]
#[allow(clippy::struct_field_names)]
pub struct Actuators {
    /// The state of the ROCKET_POWER.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::RocketPowerState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub rocket_power: Watchable<InvertedDigitalOut>,

    /// The state of the LOX decoupler extend valve.
    #[datapoint(
        pattern = "Message::RfsComputer(RFSCMessage::LoxDecouplerExtendValveState(#value))"
    )]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub lox_decoupler_extend_valve: Watchable<NormallyClosedValve>,
    /// The state of the LOX decoupler retract valve.
    #[datapoint(
        pattern = "Message::RfsComputer(RFSCMessage::LoxDecouplerRetractValveState(#value))"
    )]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub lox_decoupler_retract_valve: Watchable<NormallyClosedValve>,
    /// The state of the N2 decoupler extend valve.
    #[datapoint(
        pattern = "Message::RfsComputer(RFSCMessage::N2DecouplerExtendValveState(#value))"
    )]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub n2_decoupler_extend_valve: Watchable<NormallyClosedValve>,
    /// The state of the N2 decoupler retract valve.
    #[datapoint(
        pattern = "Message::RfsComputer(RFSCMessage::N2DecouplerRetractValveState(#value))"
    )]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub n2_decoupler_retract_valve: Watchable<NormallyClosedValve>,
    /// The state of the N2 shower valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::N2ShowerValvesState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub n2_shower_valve: Watchable<NormallyClosedValve>,
    /// The state of the N2 engine purge valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::N2RocketPurgeValveState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub n2_engine_purge_valve: Watchable<NormallyClosedValve>,

    /// The state of the rocket heating.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::RocketHeatingState(#value))")]
    #[broadcast(min_freq_hz = 1.5, max_freq_hz = 20.)]
    pub rocket_heating: Watchable<DigitalOut>,
}
