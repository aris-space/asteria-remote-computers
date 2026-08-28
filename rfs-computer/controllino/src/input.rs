use datatypes::actuator::{DigitalOut, InvertedDigitalOut, NormallyClosedValve};
use datatypes::sensor::DigitalIn;
use datatypes::status::ArmingState;
use datatypes::units::{BarG, Celsius};
use io_utils::{Broadcast, Collector};
use messages::Message;
use messages::datapoints::dp_rfs_computer::Message as RFSCMessage;
use sequences::{SensorNameExt, SensorStore};
use watchable::Watchable;

#[derive(Debug, Default, Clone, Broadcast, Collector)]
#[broadcast(message_type = "Message")]
#[collector(
    message_type = "Message",
    update_expr = "let _ = #field.update(#value);"
)]
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

/// See the matching fields in [`Sensors`].
///
/// Names follow the `#[pss(name = "...")]` instrumentation names in
/// [`messages::datapoints::dp_rfs_computer::Message`].
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Deserialize, serde::Serialize, Hash,
)]
#[allow(non_camel_case_types)]
pub enum SensorName {
    /// Name of the igniter pressure sensor.
    ign_cc_p,
    /// Name of the igniter temperature sensor.
    ign_cc_t,
    /// Name of the SMART box temperature sensor.
    smart_cabinent_t,
}

impl SensorNameExt for SensorName {
    fn is_pressure_sensor(&self) -> bool {
        matches!(self, SensorName::ign_cc_p)
    }

    fn is_temperature_sensor(&self) -> bool {
        matches!(self, SensorName::ign_cc_t | SensorName::smart_cabinent_t)
    }
}

impl std::str::FromStr for SensorName {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        use serde::Deserialize as _;
        Self::deserialize(<&str as serde::de::IntoDeserializer>::into_deserializer(s))
            .map_err(|e| e.to_string())
    }
}

impl SensorStore<SensorName> for Sensors {
    fn get_pressure_datapoint<'a>(&'a self, name: &SensorName) -> Option<&'a Watchable<BarG>> {
        match name {
            SensorName::ign_cc_p => Some(&self.igniter_pressure),
            SensorName::ign_cc_t | SensorName::smart_cabinent_t => None,
        }
    }

    fn get_temperature_datapoint<'a>(
        &'a self,
        name: &SensorName,
    ) -> Option<&'a Watchable<Celsius>> {
        match name {
            SensorName::ign_cc_t => Some(&self.igniter_temperature),
            SensorName::smart_cabinent_t => Some(&self.smart_box_temperature),
            SensorName::ign_cc_p => None,
        }
    }
}

/// The states of the actuators connected to the SMART box Controllino.
#[derive(Debug, Default, Clone, Broadcast, Collector)]
#[broadcast(message_type = "Message")]
#[collector(
    message_type = "Message",
    update_expr = "let _ = #field.update(#value);"
)]
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
