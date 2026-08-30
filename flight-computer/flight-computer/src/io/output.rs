//! Contains [`OutputTree`].

use std::collections::BTreeMap;

use datatypes::actuator::Settable;
use datatypes::status::ConfigHash;
use dp_flight_computer::FlightComputerStatus;
use io_utils::Broadcast;
use lib_mission::actuator_control::ActuatorStore;
use messages::datapoints::dp_flight_computer;
use messages::Message;
use pss_drivers::output::PSSValves;
use rfs_drivers::output::ActuatorChannel;
use watchable::Watchable;

use super::types::{
    CameraPowerState, MainDeploymentState, RecoveryArmingState, SeparationState,
    SteeringMotorRevolutions,
};

/// Holds the things we want to set
#[derive(Debug, Clone, Broadcast)]
#[broadcast(message_type = "messages::Message")]
pub struct OutputTree {
    /// When turning [`SeparationState::Separated`], the drogue should be separated.
    pub separate_drogue: Watchable<SeparationState>,
    /// When turning [`MainDeploymentState::Deployed`], the chute should be deployed.
    pub deploy_main_chute: Watchable<MainDeploymentState>,

    /// The target revolutions of the steering motors for the parachute.
    /// Whenever this changes, the recovery board will start moving toward this target.
    /// While this is `None`, the motors have not yet moved.
    /// This is to prevent early messages that would start the watchdog.
    pub steering_motor_target_deflections: Watchable<Option<SteeringMotorRevolutions>>,
    /// Whether to give power for the steering motor, deployment and separation actuators.
    pub recovery_actuators_armed: Watchable<RecoveryArmingState>,
    /// Whether to give power to the camera
    pub camera_powered_on: Watchable<CameraPowerState>,

    /// The target states of the tank and engine valves.
    pub valves: PSSValves,

    /// The target states of the RFS actuators
    pub rfs_actuators: ActuatorChannel,

    /// First 5 hex chars of the config file Git object hash. Set once at startup.
    #[datapoint(
        pattern = "Message::FlightComputer(dp_flight_computer::Message::ConfigHash(#value))"
    )]
    #[broadcast(min_freq_hz = 0.2, max_freq_hz = 0.2)]
    pub hash_config: Watchable<ConfigHash>,

    /// Aggregated FC status. Updated by the CAN status driver.
    #[datapoint(pattern = "Message::FlightComputer(dp_flight_computer::Message::Status(#value))")]
    #[broadcast(min_freq_hz = 1.0, max_freq_hz = 1.0)]
    pub fc_status: Watchable<FlightComputerStatus>,
}

/// The constructor for OutputTree. It must be provided
/// given that Default cannot be derived for every field.
impl OutputTree {
    pub fn new(tx: &std::sync::mpsc::Sender<Message>) -> Self {
        Self {
            separate_drogue: Watchable::default(),
            deploy_main_chute: Watchable::default(),
            steering_motor_target_deflections: Watchable::default(),
            recovery_actuators_armed: Watchable::default(),
            camera_powered_on: Watchable::default(),
            valves: PSSValves::new(tx.clone(), pss_drivers::output::CommandSender::FC),
            rfs_actuators: ActuatorChannel::new(tx.clone()),
            hash_config: Watchable::default(),
            fc_status: Watchable::default(),
        }
    }
}

/// A manually controllable actuator reachable from the flight computer.
///
/// This spans both the valves the FC drives on the PSS and the actuators it
/// commands on the RFS over CAN, so that a single name resolves to the right
/// one wherever it is written in a config or a sequence.
///
/// See the matching types in [`pss_drivers::output::ValveName`] and
/// [`rfs_drivers::output::ActuatorName`].
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
#[serde(untagged)]
pub enum ActuatorName {
    PSSValve(pss_drivers::output::ValveName),
    RfsActuator(rfs_drivers::output::ActuatorName),
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

impl ActuatorStore<ActuatorName> for OutputTree {
    /// Combines the PSS valves and the RFS actuators into one map.
    fn as_settables(&self) -> BTreeMap<ActuatorName, Box<dyn Settable + Send>> {
        let mut settables = BTreeMap::new();
        settables.extend(
            self.valves
                .as_settables()
                .into_iter()
                .map(|(k, v)| (ActuatorName::PSSValve(k), v)),
        );
        settables.extend(
            self.rfs_actuators
                .as_settables()
                .into_iter()
                .map(|(k, v)| (ActuatorName::RfsActuator(k), v)),
        );
        settables
    }
}

#[test]
fn test_actuator_name_from_str() {
    assert_eq!(
        "oss_ml_vlv".parse::<ActuatorName>().expect("must parse"),
        ActuatorName::PSSValve(pss_drivers::output::ValveName::oss_ml_vlv)
    );
    assert_eq!(
        "gox_ign_vlv".parse::<ActuatorName>().expect("must parse"),
        ActuatorName::RfsActuator(rfs_drivers::output::ActuatorName::gox_ign_vlv)
    );
    assert!("not_a_valve".parse::<ActuatorName>().is_err());
}
