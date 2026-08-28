//! Contains [`InputTree`]

use datatypes::actuator::ThrottleableValve;
use datatypes::status::ManualStateTransition;
use datatypes::units::{BarG, Celsius};
use io_utils::{Broadcast, Collector};
use lib_mission::util::time::MissionTime;
use lib_mission::{actuator_control, transition};
use messages::Message;
use messages::datapoints::dp_flight_computer::FlightComputerStatus;
use messages::datapoints::dp_flight_computer::Message as FCMessage;
use messages::datapoints::dp_rfs_computer::Message as RFSMessage;
use pss_drivers::input::{PSSSensors, PSSValves};
use sequences::{SensorNameExt, SensorStore};
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
    /// The current time, as as unix timestamp wrapper.
    pub time: Watchable<MissionTime>,
    /// The newest transition command from the ground station.
    pub transition_command: Watchable<ManualStateTransition>,
    /// The newest actuator command from the ground station.
    pub actuator_command: Watchable<Option<actuator_control::Command<ActuatorName>>>,

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

    /// The valve states of controllio attached eletronics.
    #[broadcast(flatten)]
    pub controllino_valves: controllino::input::Actuators,
    /// The sensor readings from controllio attached eletronics.
    #[broadcast(flatten)]
    pub controllino_sensors: controllino::input::Sensors,

    // The state of the pressure sensors that communicate on the serial bus:
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::N2PreBoosterPressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub n2_pre_booster_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::H2PurgePressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub h2_purge_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::N2PostBoosterPressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub n2_post_booster_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::LoxPressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub lox_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::PurgePressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub purge_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::PneumaticPressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub pneumatic_pressure: Watchable<BarG>,
    // Igniter system:
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::GoxPressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub gox_pressure: Watchable<BarG>,
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::H2Pressure(#value))")]
    #[broadcast(min_freq_hz = 6., max_freq_hz = 14.)]
    pub h2_pressure: Watchable<BarG>,

    /// The current state for the lox fill valve.
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::LoxFillValveState(#value))")]
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

/// A sensor readable by the state machine, be it read directly on the RFS or
/// by one of the attached electronics boards.
///
/// See the matching types in [`uremote::SensorName`], [`controllino::input::SensorName`],
/// [`pss_drivers::input::SensorName`] and [`RfsSensorName`].
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(untagged)]
pub enum SensorName {
    URSensor(uremote::SensorName),
    ControllinoSensor(controllino::input::SensorName),
    PSSSensor(pss_drivers::input::SensorName),
    RfsSensor(RfsSensorName),
}

/// Names of the pressure sensors that communicate directly with the RFS over the serial bus,
/// i.e. that are neither behind the u-remote nor the Controllino.
///
/// See the matching fields in [`InputTree`].
///
/// Names follow the `#[pss(name = "...")]` instrumentation names in
/// [`messages::datapoints::dp_rfs_computer::Message`].
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Deserialize, serde::Serialize, Hash,
)]
#[allow(non_camel_case_types)]
pub enum RfsSensorName {
    /// Name of the GOX igniter line pressure sensor.
    gox_line_p,
    /// Name of the H2 igniter line pressure sensor.
    h2_line_p,
    /// Name of the N2 pre-booster line pressure sensor.
    n2_pre_bst_p,
    /// Name of the N2 post-booster line pressure sensor.
    n2_post_bst_p,
    /// Name of the LOX line pressure sensor.
    lox_line_p,
    /// Name of the purge line pressure sensor.
    n2_prg_p,
    /// Name of the pneumatic line pressure sensor.
    n2_pnm_p,
    /// Name of the H2 purge line pressure sensor.
    h2_prg_p,
}

impl SensorNameExt for SensorName {
    fn is_pressure_sensor(&self) -> bool {
        match self {
            SensorName::URSensor(name) => name.is_pressure_sensor(),
            SensorName::ControllinoSensor(name) => name.is_pressure_sensor(),
            SensorName::PSSSensor(name) => name.is_pressure_sensor(),
            SensorName::RfsSensor(_) => true,
        }
    }

    fn is_temperature_sensor(&self) -> bool {
        match self {
            SensorName::URSensor(name) => name.is_temperature_sensor(),
            SensorName::ControllinoSensor(name) => name.is_temperature_sensor(),
            SensorName::PSSSensor(name) => name.is_temperature_sensor(),
            SensorName::RfsSensor(_) => false,
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

impl std::str::FromStr for RfsSensorName {
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
            SensorName::URSensor(name) => self.uremote_sensors.get_pressure_datapoint(name),
            SensorName::ControllinoSensor(name) => {
                self.controllino_sensors.get_pressure_datapoint(name)
            }
            SensorName::PSSSensor(name) => self.pss_sensors.get_pressure_datapoint(name),
            SensorName::RfsSensor(name) => Some(match name {
                RfsSensorName::gox_line_p => &self.gox_pressure,
                RfsSensorName::h2_line_p => &self.h2_pressure,
                RfsSensorName::n2_pre_bst_p => &self.n2_pre_booster_pressure,
                RfsSensorName::n2_post_bst_p => &self.n2_post_booster_pressure,
                RfsSensorName::lox_line_p => &self.lox_pressure,
                RfsSensorName::n2_prg_p => &self.purge_pressure,
                RfsSensorName::n2_pnm_p => &self.pneumatic_pressure,
                RfsSensorName::h2_prg_p => &self.h2_purge_pressure,
            }),
        }
    }

    fn get_temperature_datapoint<'a>(
        &'a self,
        name: &SensorName,
    ) -> Option<&'a Watchable<Celsius>> {
        match name {
            SensorName::URSensor(name) => self.uremote_sensors.get_temperature_datapoint(name),
            SensorName::ControllinoSensor(name) => {
                self.controllino_sensors.get_temperature_datapoint(name)
            }
            SensorName::PSSSensor(name) => self.pss_sensors.get_temperature_datapoint(name),
            // None of the RFS-local sensors are temperature sensors.
            SensorName::RfsSensor(_) => None,
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

#[test]
fn test_sensor_name_from_str() {
    assert_eq!(
        "lox_line_t".parse::<SensorName>().expect("must parse"),
        SensorName::URSensor(uremote::SensorName::lox_line_t)
    );
    assert_eq!(
        "ign_cc_p".parse::<SensorName>().expect("must parse"),
        SensorName::ControllinoSensor(controllino::input::SensorName::ign_cc_p)
    );
    assert_eq!(
        "eng_cc_p".parse::<SensorName>().expect("must parse"),
        SensorName::PSSSensor(pss_drivers::input::SensorName::eng_cc_p)
    );
    assert_eq!(
        "gox_line_p".parse::<SensorName>().expect("must parse"),
        SensorName::RfsSensor(RfsSensorName::gox_line_p)
    );
}
