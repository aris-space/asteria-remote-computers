use std::collections::HashSet;
use std::net::SocketAddr;
use std::time::Duration;

use lib_mission::actuator_control::command::ActuatorControl;
use lib_mission::common_states;
use lib_mission::config::{BoxableState, MakeState};
use lib_mission::state_machine::{BoxedState, StateName};
use lib_mission::transition::ManualTransitions;
use sequences::{Command, deserialize_sequence, validate_sequence};
use serde::{Deserialize, Serialize};

use crate::io::input::SensorName;
use crate::io::output::ActuatorName;
use crate::state_logic;
use crate::state_logic::helpers::ThresholdWatcher;
use keller23sx::BaudRate;

/// The full configuration file for the system.
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// The sub config defining the state machine.
    pub state_machine: MissionProfile,
    /// The sub config for telemetry.
    pub telemetry: TelemetryConfig,
    /// The sub config for the Weidmüller u-remote coupler driver.
    pub uremote: URemoteConfig,
    /// The sub config for the controllino driver.
    pub controllino: ControllinoConfig,
    /// The sub config for serial communication over Modbus RTU.
    pub serial: SerialConfig,
    /// The sub config for CAN.
    pub can: CANConfig,
    /// The rate in Hz at which the time should be updated with the system time.
    pub system_time_rate_hz: f32,
}

/// The sub config for telemetry.
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TelemetryConfig {
    /// The address used in [`mavlink_core::connect`].
    /// Typically "udpout:127.0.0.1:{PORT}".
    pub mavlink_address: String,
    /// The [names][crate::io::output::ActuatorName] of the actuators that are always allowed to be activated in this state.
    #[serde(default)]
    pub always_allowed_actuators: Vec<String>,
}

/// The sub config for serial communication over Modbus RTU.
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SerialConfig {
    /// The address used in [`tokio_serial::new`].
    /// Typically "/dev/ttyS0".
    pub serial_port: String,
    /// The baud rate used in [`tokio_serial::new`].
    pub baud_rate: BaudRate,
    /// The timeout for [`tokio_serial::new`].
    pub timeout_ms: u64,
    /// The rate in Hz at which the pressure sensors should poll the data.
    pub pressure_polling_hz: f32,
    /// The rate in Hz at which the current state of the valve should be polled.
    pub valve_state_polling_hz: f32,
}

/// The sub config for CAN.
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CANConfig {
    /// The address used in `socketcan::Socket::open`.
    pub socket_address: String,
}

/// The sub config for the Weidmüller u-remote coupler driver.
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct URemoteConfig {
    /// The address to the TCP modbus coupler.
    /// Typically "192.168.0.222:502".
    pub address: String,
    /// The minimal frequency of reads. Reads can happen fast than this.
    /// This needs to be above 1Hz such that the uremote watchdog doesn't time out.
    pub min_read_frequency_hz: f32,
}

/// The sub config for the Controllino driver.
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllinoConfig {
    /// The TCP modbus server address, usually `192.168.0.227:502`.
    pub address: SocketAddr,
    /// The timeout for a request before it should be cancelled.
    #[serde(deserialize_with = "deserialize_duration_secs")]
    #[serde(rename = "request_timeout_secs")]
    pub request_timeout: Duration,
    /// The rate in Hz at which the pressure sensors should be polled.
    pub pressure_polling_hz: f32,
    /// The rate in Hz at which the temperature sensors should be polled.
    pub temperature_polling_hz: f32,
    /// The rate in Hz at which the digital sensors should be polled.
    pub digital_polling_hz: f32,
    /// The rate in Hz at which the actuator state should be polled.
    pub actuator_polling_hz: f32,
}

fn deserialize_duration_secs<'de, D>(deserializer: D) -> Result<Duration, D::Error>
where
    D: serde::de::Deserializer<'de>,
{
    let secs = f32::deserialize(deserializer)?;
    Ok(Duration::from_secs_f32(secs))
}

/// The main config file specification.
pub type MissionProfile = lib_mission::config::MissionProfile<StateLogicConfig>;

/// The logic of a state.
#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum StateLogicConfig {
    /// Describes a [`common_states::Waiting`].
    Waiting,
    /// Describes a [`common_states::Shutdown`].
    Shutdown,
    /// Describes a [`state_logic::ManualControl`].
    ManualControl {
        /// The [names][crate::io::output::ActuatorName] of the actuators that are allowed to be activated in this state.
        allowed_actuators: Vec<String>,
    },
    /// Describes a [`state_logic::MonitoredManualControl`].
    MonitoredManualControl {
        /// The [names][crate::io::output::ActuatorName] of the actuators that are allowed to be activated in this state.
        allowed_actuators: Vec<String>,
        /// Thresholds on sensors to watch during the whole of the sequence.
        /// If any of them are violated, this will end the sequence and proceeds to `on_cancelled`.
        #[serde(default)]
        thresholds: ThresholdWatcher,
        /// If the last telemetry message was received more than `telemetry_loss_threshold_sec` seconds ago,
        /// exit this state.
        telemetry_loss_threshold_sec: f32,
        /// The state to transition to when a sensor or telemetry violation occured.
        on_violation: String,
    },
    /// Describes a [`state_logic::RunSeqWithControl`].
    RunSeqWithControl {
        /// The [names][crate::io::output::ActuatorName] of the actuators that are allowed to be activated in this state.
        allowed_actuators: Vec<String>,
        /// Thresholds on sensors to watch during the whole of the sequence.
        /// If any of them are violated, this will end the sequence and proceeds to `on_cancelled`.
        #[serde(default)]
        thresholds: ThresholdWatcher,
        /// If the last telemetry message was received more than `telemetry_loss_threshold_sec` seconds ago,
        /// exit this state.
        telemetry_loss_threshold_sec: f32,
        /// The state to transition to when a sensor or telemetry violation occured.
        on_violation: String,
        /// The sequence to run, as a DSL string (see the `sequence-parser` crate).
        #[serde(deserialize_with = "deserialize_sequence", skip_serializing)]
        sequence: Vec<Command<ActuatorName, SensorName, String>>,
        /// The state to transition to when the sequence has been completed.
        on_completed: String,
    },
    /// Describes a [`state_logic::AwaitTimer`].
    AwaitTimer {
        /// The duration of the timer in seconds.
        duration_sec: f32,
        /// The name of the state to which should be transitioned after the timer expired.
        on_timer: String,
    },
    /// Describes a [`state_logic::DiagnosticsGate`].
    DiagnosticsGate {
        /// The name of the state which to proceed to if all sensors are healthy.
        on_tests_pass: String,
        /// The name of the state which to proceed to if a sensor is unhealthy.
        on_tests_fail: String,
    },
    /// Describes a [`state_logic::PSSDiagnosticsGate`].
    PSSDiagnosticsGate {
        /// Maximum allowed duration in seconds since last message to consider a board responsive.
        outdated_threshold_sec: f64,
        /// If the diagnostics succeed, we will transition to the state with the name of `on_tests_pass`.
        on_tests_pass: String,
        /// If the diagnostics fail, we will transition to the state with the name of `on_tests_fail`.
        on_tests_fail: String,
    },
    /// Describes a [`state_logic::DepressurizeRFS`].
    DepressurizeRFS {
        /// The name of the state which to proceed to after completing setting all valves but lox bleed normal.
        on_done: String,
    },
    /// Describes a [`state_logic::SetNormalState`].
    SetNormalState {
        /// The name of the state which to proceed to after completing setting all valves.
        on_done: String,
    },
    /// Describes a [`state_logic::RunSequence`].
    RunSequence {
        /// The sequence to run, as a DSL string (see the `sequence-parser` crate).
        #[serde(deserialize_with = "deserialize_sequence", skip_serializing)]
        sequence: Vec<Command<ActuatorName, SensorName, String>>,
        /// Thresholds on sensors to watch during the whole of the sequence.
        /// If any of them are violated, this will end the sequence and proceeds to `on_cancelled`.
        #[serde(default)]
        thresholds: ThresholdWatcher,
        /// The state to transition to when the sequence has been completed.
        on_completed: String,
        /// The state to transition to when a watched threshold has been violated.
        on_cancelled: String,
    },
    /// Describes a [`state_logic::DisarmIgniter`].
    DisarmIgniter {
        /// The name of the state which to proceed to after software disarming the igniter.
        on_done: String,
    },
    /// Describes a [`state_logic::WaitForIgnition`].
    WaitForIgnition {
        /// The name of the state which to proceed to after the igniter turned on.
        on_ignition: String,
        /// The name of the state which to proceed to if the RFS is not ready for ignition.
        on_not_ready: String,
        /// The [names][crate::io::output::ActuatorName] of the actuators that are allowed to be activated in this state.
        allowed_actuators: Vec<String>,
    },
}

#[allow(clippy::too_many_lines)]
impl MakeState for StateLogicConfig {
    type State = BoxedState<
        crate::state_logic::Input,
        crate::state_logic::Output,
        crate::state_logic::Parameter,
    >;

    fn make(
        self,
        names: &HashSet<StateName>,
        transitions: ManualTransitions,
    ) -> lib_mission::Result<Self::State> {
        Ok(match self {
            StateLogicConfig::Waiting => common_states::Waiting::new(transitions).into_box(),
            StateLogicConfig::Shutdown => common_states::Shutdown::default().into_box(),
            StateLogicConfig::ManualControl { allowed_actuators } => state_logic::ManualControl {
                manual_transitions: transitions,
                actuator_control: ActuatorControl::from_strings(
                    allowed_actuators.iter().map(String::as_str),
                )
                .map_err(lib_mission::Error::malformed_config)?,
            }
            .into_box(),
            StateLogicConfig::MonitoredManualControl {
                allowed_actuators,
                thresholds,
                telemetry_loss_threshold_sec,
                on_violation,
            } => state_logic::MonitoredManualControl {
                manual_transitions: transitions,
                actuator_control: ActuatorControl::from_strings(
                    allowed_actuators.iter().map(String::as_str),
                )
                .map_err(lib_mission::Error::malformed_config)?,
                telemetry_loss_threshold: std::time::Duration::from_secs_f32(
                    telemetry_loss_threshold_sec,
                ),
                thresholds,
                on_violation: StateName::try_new(&on_violation, names)?,
            }
            .into_box(),
            StateLogicConfig::RunSeqWithControl {
                allowed_actuators,
                thresholds,
                telemetry_loss_threshold_sec,
                on_violation,
                sequence,
                on_completed,
            } => state_logic::RunSeqWithControl {
                manual_transitions: transitions,
                actuator_control: ActuatorControl::from_strings(
                    allowed_actuators.iter().map(String::as_str),
                )
                .map_err(lib_mission::Error::malformed_config)?,
                telemetry_loss_threshold: std::time::Duration::from_secs_f32(
                    telemetry_loss_threshold_sec,
                ),
                thresholds,
                on_violation: StateName::try_new(&on_violation, names)?,
                sequence: validate_sequence(sequence, names)?,
                on_completed: StateName::try_new(&on_completed, names)?,
            }
            .into_box(),
            StateLogicConfig::AwaitTimer {
                duration_sec,
                on_timer,
            } => state_logic::AwaitTimer {
                duration: std::time::Duration::from_secs_f32(duration_sec),
                on_timer: StateName::try_new(&on_timer, names)?,
                manual_transitions: transitions,
            }
            .into_box(),
            StateLogicConfig::DiagnosticsGate {
                on_tests_pass,
                on_tests_fail,
            } => state_logic::DiagnosticsGate {
                on_tests_pass: StateName::try_new(&on_tests_pass, names)?,
                on_tests_fail: StateName::try_new(&on_tests_fail, names)?,
            }
            .into_box(),
            StateLogicConfig::PSSDiagnosticsGate {
                outdated_threshold_sec,
                on_tests_pass,
                on_tests_fail,
            } => state_logic::PSSDiagnosticsGate {
                outdated_threshold: std::time::Duration::from_secs_f64(outdated_threshold_sec),
                on_tests_pass: StateName::try_new(&on_tests_pass, names)?,
                on_tests_fail: StateName::try_new(&on_tests_fail, names)?,
            }
            .into_box(),
            StateLogicConfig::DepressurizeRFS { on_done } => state_logic::DepressurizeRFS {
                on_done: StateName::try_new(&on_done, names)?,
            }
            .into_box(),
            StateLogicConfig::SetNormalState { on_done } => state_logic::SetNormalState {
                on_done: StateName::try_new(&on_done, names)?,
            }
            .into_box(),
            StateLogicConfig::RunSequence {
                sequence,
                thresholds,
                on_completed,
                on_cancelled,
            } => state_logic::RunSequence {
                manual_transitions: transitions,
                sequence: validate_sequence(sequence, names)?,
                thresholds,
                on_completed: StateName::try_new(&on_completed, names)?,
                on_cancelled: StateName::try_new(&on_cancelled, names)?,
            }
            .into_box(),
            StateLogicConfig::DisarmIgniter { on_done } => state_logic::DisarmIgniter {
                on_done: StateName::try_new(&on_done, names)?,
            }
            .into_box(),
            StateLogicConfig::WaitForIgnition {
                on_ignition,
                on_not_ready,
                allowed_actuators,
            } => state_logic::WaitForIgnition {
                manual_transitions: transitions,
                on_ignition: StateName::try_new(&on_ignition, names)?,
                on_not_ready: StateName::try_new(&on_not_ready, names)?,
                actuator_control: ActuatorControl::from_strings(
                    allowed_actuators.iter().map(String::as_str),
                )
                .map_err(lib_mission::Error::malformed_config)?,
            }
            .into_box(),
        })
    }
}
