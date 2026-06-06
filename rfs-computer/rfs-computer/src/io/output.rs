//! Contains [`OutputTree`].

use std::collections::BTreeMap;

use datatypes::actuator::Settable;
use datatypes::actuator::ThrottleableValve;
use datatypes::status::ConfigHash;
use io_utils::Broadcast;
use messages::Message;
use messages::datapoints::dp_rfs_computer::{Message as RFSMessage, RfsComputerStatus};
use pss_drivers::output::PSSValves;
use uremote_electronics::output as uremote;
use watchable::Watchable;

/// A manually controllable actuator in the system
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
#[serde(untagged)]
pub enum ActuatorName {
    URValve(uremote::ActuatorName),
    ControllinoValve(controllino::output::ActuatorName),
    PSSValve(pss_drivers::output::ValveName),
    LoxFillValve(LoxFillValveName),
}

/// Holds the things we want to set
#[derive(Debug, Clone, Broadcast)]
#[broadcast(message_type = "messages::Message")]
pub struct OutputTree {
    /// A grouping of all the valves and actuators attached to the weidmüller u-remote coupler.
    pub rfs_actuators: uremote::Actuators,
    /// A grouping of all the valves and actuators attached to the SMART box Controllino.
    pub controllino_actuators: controllino::output::Actuators,
    /// The target states of the tank and engine valves in the rocket.
    pub pss_valves: PSSValves,

    /// The target state for the lox fill valve (`lox_fll_vlv`).
    pub lox_fill_valve: Watchable<ThrottleableValve>,

    /// First 5 hex chars of the config file Git object hash. Set once at startup.
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::ConfigHash(#value))")]
    #[broadcast(min_freq_hz = 0.2, max_freq_hz = 0.2)]
    pub hash_config: Watchable<ConfigHash>,

    /// If the system is in a software armed state.
    /// This will allow the ECU to control the igniter actuators.
    #[datapoint(pattern = "Message::RfsComputer(RFSMessage::Status(#value))")]
    #[broadcast(min_freq_hz = 1.0, max_freq_hz = 1.0)]
    pub rfs_computer_status: Watchable<RfsComputerStatus>, // more appropriate I think
}

impl OutputTree {
    /// Combines all the settables
    pub fn as_settables(&self) -> BTreeMap<ActuatorName, Box<dyn Settable + Send>> {
        let mut settables = BTreeMap::new();
        settables.extend(
            self.rfs_actuators
                .as_settables()
                .into_iter()
                .map(|(k, v)| (ActuatorName::URValve(k), v)),
        );
        settables.extend(
            self.pss_valves
                .as_settables()
                .into_iter()
                .map(|(k, v)| (ActuatorName::PSSValve(k), v)),
        );
        settables.insert(
            ActuatorName::LoxFillValve(LoxFillValveName::lox_fll_vlv),
            Box::new(self.lox_fill_valve.clone()) as Box<dyn Settable + Send>,
        );
        settables
    }
}

/// Implement parsing based on serde.
impl std::str::FromStr for ActuatorName {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        use serde::Deserialize as _;
        Self::deserialize(<&str as serde::de::IntoDeserializer>::into_deserializer(s))
            .map_err(|e| e.to_string())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
#[allow(non_camel_case_types)] // This is just a hack to make serde do what I want
pub enum LoxFillValveName {
    lox_fll_vlv,
}

impl OutputTree {
    /// The constructor for OutputTree. It must be provided
    /// given that Default cannot be derived for every field.
    pub fn new(
        can: std::sync::mpsc::Sender<Message>,
        controllino: controllino::driver::Client,
    ) -> Self {
        Self {
            rfs_actuators: uremote::Actuators::default(),
            controllino_actuators: controllino.into_output_tree(),
            pss_valves: PSSValves::new(can, pss_drivers::output::CommandSender::RFS),
            lox_fill_valve: Watchable::default(),
            hash_config: Watchable::default(),
            rfs_computer_status: Watchable::default(),
        }
    }
}

#[test]
fn test_name_from_str() {
    assert_eq!(
        "gox_prg_vlv".parse::<ActuatorName>().expect("must parse"),
        ActuatorName::URValve(uremote::ActuatorName::gox_prg_vlv)
    );
    assert_eq!(
        "n2_dec_ret".parse::<ActuatorName>().expect("must parse"),
        ActuatorName::URValve(uremote::ActuatorName::n2_dec_ret)
    );
    assert_eq!(
        "oss_ml_vlv".parse::<ActuatorName>().expect("must parse"),
        ActuatorName::PSSValve(pss_drivers::output::ValveName::oss_ml_vlv)
    );
    assert_eq!(
        "prz_copv_vnt".parse::<ActuatorName>().expect("must parse"),
        ActuatorName::PSSValve(pss_drivers::output::ValveName::prz_copv_vnt)
    );
    assert_eq!(
        "lox_fll_vlv".parse::<ActuatorName>().expect("must parse"),
        ActuatorName::LoxFillValve(LoxFillValveName::lox_fll_vlv)
    );
}
