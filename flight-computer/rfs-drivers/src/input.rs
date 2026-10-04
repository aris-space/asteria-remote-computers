use datatypes::actuator::{DigitalOut, NormallyClosedValve};
use datatypes::units::{BarG, Celsius};
use io_utils::Broadcast;
use io_utils::Collector;
use messages::Message;
use messages::datapoints::dp_rfs_computer::Message as RFSCMessage;
use sequences::{SensorNameExt, SensorStore};
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
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 10.)]
    pub gox_purge_valve: Watchable<NormallyClosedValve>,
    /// The state of the H2 purge valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::H2PurgeValveState(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 10.)]
    pub h2_purge_valve: Watchable<NormallyClosedValve>,
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::H2IgniterValveState(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 10.)]
    pub h2_igniter_valve: Watchable<NormallyClosedValve>,
    /// The state of the GOX igniter valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::GoxIgniterValveState(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 10.)]
    pub gox_igniter_valve: Watchable<NormallyClosedValve>,
    /// The state of the igniter spark.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::IgniterSparkPlugState(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 10.)]
    pub igniter_spark: Watchable<DigitalOut>,
    /// The state of the N2 engine purge valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::N2RocketPurgeValveState(#value))")]
    #[broadcast(min_freq_hz = 0.5, max_freq_hz = 10.)]
    pub n2_engine_purge_valve: Watchable<NormallyClosedValve>,
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
    #[broadcast(min_freq_hz = 0.2, max_freq_hz = 3.)]
    pub igniter_pressure: Watchable<BarG>,
    /// The last received temperature of the igniter box.
    #[datapoint(pattern = "Message::RfsComputer(RFSCMessage::IgniterTemperature(#value))")]
    #[broadcast(min_freq_hz = 0.2, max_freq_hz = 1.5)]
    pub igniter_temperature: Watchable<Celsius>,
}

/// See the matching types in [`Sensors`].
///
/// Names follow the `#[pss(name = "...")]` instrumentation names in
/// [`messages::datapoints::dp_rfs_computer::Message`].
///
/// ```
/// use rfs_drivers::input::SensorName;
/// assert_eq!("ign_cc_p".parse::<SensorName>().unwrap(), SensorName::ign_cc_p)
/// ```
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Deserialize, serde::Serialize, Hash,
)]
#[allow(non_camel_case_types)]
pub enum SensorName {
    /// Name of the igniter combustion chamber pressure sensor.
    ign_cc_p,
    /// Name of the igniter combustion chamber temperature sensor.
    ///
    /// Note that the matching datapoint has no CAN id, so on the flight computer
    /// this never receives a value. See [`Sensors::igniter_temperature`].
    ign_cc_t,
}

impl SensorNameExt for SensorName {
    fn is_pressure_sensor(&self) -> bool {
        matches!(self, SensorName::ign_cc_p)
    }

    fn is_temperature_sensor(&self) -> bool {
        matches!(self, SensorName::ign_cc_t)
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
        let watchable = match name {
            SensorName::ign_cc_p => &self.igniter_pressure,
            // `name` is not the name of a pressure sensor.
            SensorName::ign_cc_t => return None,
        };
        Some(watchable)
    }

    fn get_temperature_datapoint<'a>(
        &'a self,
        name: &SensorName,
    ) -> Option<&'a Watchable<Celsius>> {
        let watchable = match name {
            SensorName::ign_cc_t => &self.igniter_temperature,
            // `name` is not the name of a temperature sensor.
            SensorName::ign_cc_p => return None,
        };
        Some(watchable)
    }
}
