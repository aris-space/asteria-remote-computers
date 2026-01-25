use std::collections::HashSet;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use gnc_interface::GuidanceController;
use lib_mission::actuator_control::command::ActuatorControl;
use lib_mission::common_states;
use lib_mission::config::{BoxableState, MakeState};
use lib_mission::state_machine::{BoxedState, StateName};
use lib_mission::transition::ManualTransitions;
use serde::{Deserialize, Serialize};

use crate::io::input::InputTree;
use crate::io::output::OutputTree;
use crate::io::types::{CameraPowerState, RecoveryArmingState, SteeringMotorRevolutions};
use crate::state_logic;
use crate::state_logic::helpers::ThresholdWatcher;

/// The full configuration file for the system.
// Note this is generic over the State machine config (`MissionProfile<GNC>`), not GNC
// to make serde not emit too strict trait bounds
#[derive(Debug, Serialize, Deserialize)]
pub struct Config<SM> {
    /// The sub config defining the state machine.
    pub state_machine: SM,
    /// The sub config for telemetry.
    pub telemetry: TelemetryConfig,
    /// The sub config for CAN.
    pub can: CANConfig,
    /// The rate in Hz at which the time should be updated with the system time.
    pub system_time_rate_hz: f32,
}

/// The sub config for telemetry.
#[derive(Debug, Serialize, Deserialize)]
pub struct TelemetryConfig {
    /// The address used in [`mavlink_core::connect`].
    /// Typically "udpout:127.0.0.1:{PORT}".
    pub mavlink_address: String,
    /// Defines the rates at which certain measurements are sent down in Hz.
    pub downlink_rates: DownLinkRateConfig,
}

/// Defines the rates at which certain measurements are sent down.
#[derive(Debug, Serialize, Deserialize)]
pub struct DownLinkRateConfig {
    /// The rate in Hz at which the current state should be sent down.
    pub state_hz: f32,
    /// The rate in Hz at which the kinematics (position, velocity, acceleration, orientation) should be sent down.
    pub kinematics_hz: f32,
    /// The rate in Hz at which the raw unprocessed data (environmental data, rail status 24V, rail status 5V) should be sent down.
    pub av_stack_data_hz: f32,
    /// The rate in Hz at which the steering motor positions should be sent down.
    pub steering_data_hz: f32,
    /// The rate in Hz at which the state of the actuator is sent down.
    pub actuator_feedback_hz: f32,
    /// The rate in Hz at which the temperature readings are sent down.
    pub temperature_sensors_hz: f32,
    /// The rate in Hz at which the pressure readings are sent down.
    pub pressure_sensors_hz: f32,
    /// The rate in Hz at which the tank fill levels are sent down.
    pub fill_levels_hz: f32,
}

/// The sub config for CAN.
#[derive(Debug, Serialize, Deserialize)]
pub struct CANConfig {
    /// The address used in `socketcan::Socket::open`.
    pub socket_address: String,
}

/// The main config file specification.
pub type MissionProfile<Gnc> = lib_mission::config::MissionProfile<StateLogicConfig<Gnc>>;

/// The logic of a state.
#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum StateLogicConfig<Gnc: GuidanceController> {
    /// Describes a [`common_states::Waiting`].
    Waiting,
    /// Describes a [`state_logic::AwaitTimer`].
    AwaitTimer {
        /// The duration of the timer in seconds.
        duration_sec: f32,
        /// The name of the state to which should be transitioned after the timer expired.
        on_timer: String,
    },
    /// Describes a [`state_logic::MonitoredWaiting`].
    MonitoredWaiting {
        /// Thresholds on sensors to watch during waiting.
        /// If any of them are violated, this will proceed to `on_violation`.
        #[serde(default)]
        thresholds: ThresholdWatcher,
        /// If the last telemetry message was received more than `telemetry_loss_threshold_sec` seconds ago,
        /// exit this state.
        telemetry_loss_threshold_sec: f32,
        /// The state to transition to when a sensor violation occured.
        on_violation: String,
    },
    /// Describes a [`state_logic::Diagnostics`].
    Diagnostics {
        /// If the `time_limit_sec` is exceeded, we will transition to the state with the name of `on_tests_fail`.
        outdated_threshold_sec: f64,
        /// If `true`, also checks the PSS boards, as is usual during a flight.
        /// Else, only the AV stack boards and recovery board are checked.
        check_pss_boards: bool,
        /// If the diagnostics succeed, we will transition to the state with the name of `on_tests_pass`.
        on_tests_pass: String,
        /// If the diagnostics fail, we will transition to the state with the name of `on_tests_fail`.
        on_tests_fail: String,
    },
    /// Describes a [`state_logic::HeightOrTimeTrigger`].
    HeightOrTimeTrigger {
        /// If the time limit is reached, we proceed to the `on_time_limit_reached` state
        time_limit_sec: f64,
        on_time_limit_reached: String,
        /// If the height limit is reached, we proceed to the `on_height_limit_reached` state
        height_limit_m: f32,
        on_height_limit_reached: String,
    },
    /// Describes a [`state_logic::HeightTimeOrCATSDeplTrigger`].
    HeightTimeOrCATSDeplTrigger {
        /// If the time limit is reached, we proceed to the `on_time_limit_reached` state
        time_limit_sec: f64,
        on_time_limit_reached: String,
        /// If the height limit is reached, we proceed to the `on_height_limit_reached` state
        height_limit_m: f32,
        on_height_limit_reached: String,
        /// If CATS triggers deployment, we transition to `on_cats_deployment`
        on_cats_deployment: String,
    },
    /// Describes a [`state_logic::Separate`].
    Separation {
        /// If the `deploy_time_sec` time passes since transitioning into the state,
        /// we proceed to the `on_after_separation` state  
        deploy_time_sec: f64,
        on_after_separation: String,
    },
    /// Describes a [`state_logic::MainChuteDeployment`].
    MainChuteDeployment {
        /// If the `deploy_time_sec` time passes since transitioning into the state,
        /// we proceed to the `on_after_deployment` state  
        deploy_time_sec: f64,
        on_after_deployment: String,
    },
    /// Describes a [`state_logic::HookRelease`].
    HookRelease {
        /// The revolution count from the intial state (positive being pulling) we have to pull for the hook to be released.
        /// `motors_pull_value_rev.0` is the value of the left motor
        /// `motors_pull_value_rev.1` is the value of the right motor
        motors_pull_value_rev: (f64, f64),
        /// When both motors reach the respective pull revolutions of `motors_ok_threshold_rev`, we transition to the `on_motors_ok_threshhold_reached` state.
        /// `motor_ok_threshold_rev.0` is the value of the left motor
        /// `motor_ok_threshold_rev.1` is the value of the right motor
        motors_ok_threshold_rev: (f64, f64),
        on_motors_ok_threshhold_reached: String,
    },
    ControlLoop {
        /// GNC config values.
        gnc_config: Gnc::Config,
        /// How often the control loop is called in Hz.
        loop_frequency_hz: f32,
        /// The number of full revolutions pulling the left line for the 0 and 1 GNC output in this order.
        /// Can be negative to signify relaxation beyond the intial state.
        revolution_interval_mapping_left: (f64, f64),
        /// The number of full revolutions pulling the right line for the 0 and 1 GNC output in this order.
        /// Can be negative to signify relaxation beyond the intial state.
        revolution_interval_mapping_right: (f64, f64),
        /// The time limit for "touchdown detection".
        time_limit_sec: u64,
        /// The name of the state to advance to once we "detected touchdown" (timer runs out).
        on_touchdown: String,
        /// The name of the state to advance to when GNC return an error.
        on_failure: String,
    },
    /// Describes a [`state_logic::SafetySpiral`].
    SafetySpiral {
        /// The value to which we will move the motors in revolutions.
        /// `motors_pull_value_rev.0` is the value of the left motor
        /// `motors_pull_value_rev.1` is the value of the right motor
        motors_pull_value_rev: (f64, f64),
        on_motors_pulled: String,
    },
    /// Describes a [`state_logic::ArmRecovery`].
    ArmRecovery {
        /// Target software arming state of the recovery actuators (Steering, Separation and Deployment).
        /// `Armed` means they are provided power.
        arming_state: RecoveryArmingState,
        /// The state to which we transition after giving or cutting power.
        on_state_set: String,
    },
    /// Describes a [`state_logic::CameraPowerControl`]
    CameraPowerControl {
        /// Target power state of the cameras (recording and livestream).
        camera_power_state: CameraPowerState,
        /// The state to which we transition after giving or cutting power.
        on_state_set: String,
    },
    /// Describes a [`state_logic::ManualControl`].
    ManualControl {
        /// The [names][pss_drivers::output::ValveName] of the actuators that are allowed to be activated in this state.
        allowed_actuators: Vec<String>,
    },
    /// Describes a [`state_logic::SetPSSMode`].
    SetPSSMode {
        /// How the mode should be set
        mode: state_logic::set_pss_mode::PSSMode,
        /// The name of the state which to proceed to after setting the mode.
        on_done: String,
    },
    /// Describes a [`state_logic::SetNormalState`].
    SetNormalState {
        /// The name of the state which to proceed to after completing setting all valves.
        on_done: String,
    },
    /// Describes a [`state_logic::SetActuators`].
    SetActuators {
        /// Which valves should be set to which position
        targets: std::collections::BTreeMap<
            pss_drivers::output::ValveName,
            state_logic::set_actuators::Value,
        >,
        /// The name of the state which to proceed to after completing setting the valves.
        on_done: String,
    },
    /// Describes a [`state_logic::ReadRFSArming`].
    ReadRFSArming {
        /// The name of the state which to proceed to if the rfs is armed for ignition.
        on_armed: String,
        /// The name of the state which to proceed to if the rfs is not armed for ignition.
        on_not_armed: String,
    },
    /// Describes a [`state_logic::AwaitRFSMode`].
    AwaitRFSMode {
        /// What mode should be awaited
        mode: state_logic::await_rfs_mode::RfsPSSMode,
        /// The name of the state which to proceed to after setting the mode.
        on_detected: String,
    },
    /// Describes a [`state_logic::AwaitRFSModeWhileConnected`].
    AwaitRFSModeWhileConnected {
        /// What mode should be awaited
        mode: state_logic::await_rfs_mode::RfsPSSMode,
        /// The name of the state which to proceed to after setting the mode.
        on_detected: String,
        /// The max time after a message from the RFS before a transition is triggered.
        rfs_can_timeout_sec: f32,
        /// The name of the state which to proceed to after a timeout.
        on_timeout: String,
    },
    /// Describes a [`state_logic::WaitWhileConnected`].
    WaitWhileConnected {
        /// The max time after a message from the RFS before a transition is triggered.
        rfs_can_timeout_sec: f32,
        /// The name of the state which to proceed to after a timeout.
        on_timeout: String,
    },
    /// Describes a [`state_logic::Ignition`].
    Ignition {
        /// The name of the state which to proceed to after sending the ignition signal.
        on_done: String,
    },
    /// Describes a [`state_logic::AbortIgnition`].
    AbortIgnition {
        /// The name of the state which to proceed to after sending the abort signal.
        on_done: String,
    },
    LiftoffDetection {
        /// If the acceleration limit in -z direction is reached, we detect liftoff and proceed to the `on_liftoff_detected` state
        acceleration_limit_ms2: f32,
        /// The acceleration has to be over the `acceleration_limit_ms2` for at least the `required_duration_s`.
        required_duration_s: f32,
        /// The name of the state which to proceed to after detecting liftoff.
        on_liftoff_detected: String,
        /// The name of the state which to proceed to after ecu aborted.
        on_ecu_aborted: String,
    },
    /// Describes a [`state_logic::ApogeeTrigger`].
    ApogeeTrigger {
        /// If the time to reach apogee is shorter than `trigger_before_apogee_time_s` apogee gets detected.
        trigger_before_apogee_time_s: f32,
        /// The name of the state which to proceed to after apogee detection.
        on_apogee_reached: String,
    },
    /// Describes a [`state_logic::Thrust`].
    Thrust {
        /// When the ECU signals an completed or aborted firing, we transition to this state.
        on_firing_finished: String,
        /// If the downward velocity is larger than `abort_downward_velocity_m_s` apogee gets detected.
        abort_downward_velocity_m_s: f32,
        /// The name of the state which to proceed to after apogee detection.
        on_apogee_reached: String,
    },
    /// Describes a [`state_logic::MonitoredManualControl`].
    MonitoredManualControl {
        /// The [names][pss_drivers::output::ValveName] of the actuators that are allowed to be activated in this state.
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
    /// Describes a [`common_states::Shutdown`].
    Shutdown,
}

#[allow(clippy::too_many_lines)]
impl<Gnc: GuidanceController> MakeState for StateLogicConfig<Gnc> {
    type State = BoxedState<InputTree, OutputTree, ()>;

    fn make(
        self,
        names: &HashSet<StateName>,
        manual_transitions: ManualTransitions,
    ) -> lib_mission::Result<Self::State> {
        Ok(match self {
            StateLogicConfig::Waiting => common_states::Waiting::new(manual_transitions).into_box(),
            StateLogicConfig::AwaitTimer {
                duration_sec,
                on_timer,
            } => state_logic::AwaitTimer {
                duration: Duration::from_secs_f32(duration_sec),
                on_timer: StateName::try_new(&on_timer, names)?,
                manual_transitions,
            }
            .into_box(),
            StateLogicConfig::MonitoredWaiting {
                thresholds,
                telemetry_loss_threshold_sec,
                on_violation,
            } => state_logic::MonitoredWaiting {
                manual_transitions,
                thresholds,
                telemetry_loss_threshold: std::time::Duration::from_secs_f32(
                    telemetry_loss_threshold_sec,
                ),
                on_violation: StateName::try_new(&on_violation, names)?,
            }
            .into_box(),
            StateLogicConfig::Diagnostics {
                outdated_threshold_sec,
                check_pss_boards,
                on_tests_pass,
                on_tests_fail,
            } => state_logic::Diagnostics {
                outdated_threshold: Duration::from_secs_f64(outdated_threshold_sec),
                check_pss_boards,
                on_tests_pass: StateName::try_new(&on_tests_pass, names)?,
                on_tests_fail: StateName::try_new(&on_tests_fail, names)?,
            }
            .into_box(),
            StateLogicConfig::HeightOrTimeTrigger {
                time_limit_sec,
                on_time_limit_reached,
                height_limit_m,
                on_height_limit_reached,
            } => state_logic::HeightOrTimeTrigger {
                time_limit: Duration::from_secs_f64(time_limit_sec),
                on_time_limit_reached: StateName::try_new(&on_time_limit_reached, names)?,
                height_limit: height_limit_m,
                on_height_limit_reached: StateName::try_new(&on_height_limit_reached, names)?,
                manual_transitions,
            }
            .into_box(),
            StateLogicConfig::HeightTimeOrCATSDeplTrigger {
                time_limit_sec,
                on_time_limit_reached,
                height_limit_m,
                on_height_limit_reached,
                on_cats_deployment,
            } => state_logic::HeightTimeOrCATSDeplTrigger {
                time_limit: Duration::from_secs_f64(time_limit_sec),
                on_time_limit_reached: StateName::try_new(&on_time_limit_reached, names)?,
                height_limit: height_limit_m,
                on_height_limit_reached: StateName::try_new(&on_height_limit_reached, names)?,
                manual_transitions,
                on_cats_deployment: StateName::try_new(&on_cats_deployment, names)?,
            }
            .into_box(),
            StateLogicConfig::Separation {
                deploy_time_sec,
                on_after_separation,
            } => state_logic::Separate {
                deploy_time: Duration::from_secs_f64(deploy_time_sec),
                on_after_separation: StateName::try_new(&on_after_separation, names)?,
            }
            .into_box(),
            StateLogicConfig::MainChuteDeployment {
                deploy_time_sec,
                on_after_deployment,
            } => state_logic::MainChuteDeployment {
                deploy_time: Duration::from_secs_f64(deploy_time_sec),
                on_after_deployment: StateName::try_new(&on_after_deployment, names)?,
            }
            .into_box(),
            StateLogicConfig::HookRelease {
                motors_pull_value_rev,
                motors_ok_threshold_rev,
                on_motors_ok_threshhold_reached,
            } => state_logic::HookRelease {
                motors_pull_value: SteeringMotorRevolutions::try_from_revolutions(
                    motors_pull_value_rev.0,
                    motors_pull_value_rev.1,
                )
                .ok_or_else(|| {
                    lib_mission::Error::malformed_config(
                        "Hook release revolutions are not well-defined",
                    )
                })?,
                motors_ok_threshold: SteeringMotorRevolutions::try_from_revolutions(
                    motors_ok_threshold_rev.0,
                    motors_ok_threshold_rev.1,
                )
                .ok_or_else(|| {
                    lib_mission::Error::malformed_config(
                        "Hook release revolutions are not well-defined",
                    )
                })?,
                on_motors_ok_threshhold_reached: StateName::try_new(
                    &on_motors_ok_threshhold_reached,
                    names,
                )?,
                manual_transitions,
            }
            .into_box(),
            StateLogicConfig::ControlLoop {
                gnc_config,
                loop_frequency_hz,
                revolution_interval_mapping_left,
                revolution_interval_mapping_right,
                on_failure,
                time_limit_sec,
                on_touchdown,
            } => {
                for v in [
                    revolution_interval_mapping_left.0,
                    revolution_interval_mapping_left.1,
                    revolution_interval_mapping_right.0,
                    revolution_interval_mapping_right.1,
                ] {
                    if !(-7.0..=7.0).contains(&v) {
                        return Err(lib_mission::Error::malformed_config(
                            "GNC revolutions outside allowed +- 7 interval",
                        ));
                    }
                }

                state_logic::ControlLoop::<Gnc> {
                    gnc_controller: Mutex::new(
                        Gnc::new(gnc_config).map_err(lib_mission::Error::malformed_config)?,
                    ),
                    loop_period: Duration::from_secs_f32(1. / loop_frequency_hz),
                    manual_transitions,
                    time_limit: Duration::from_secs(time_limit_sec),
                    on_touchdown: StateName::try_new(&on_touchdown, names)?,
                    on_failure: StateName::try_new(&on_failure, names)?,
                    revolution_interval_mapping_left,
                    revolution_interval_mapping_right,
                }
                .into_box()
            }
            StateLogicConfig::SafetySpiral {
                motors_pull_value_rev,
                on_motors_pulled,
            } => state_logic::SafetySpiral {
                motors_pull_value: SteeringMotorRevolutions::try_from_revolutions(
                    motors_pull_value_rev.0,
                    motors_pull_value_rev.1,
                )
                .ok_or_else(|| {
                    lib_mission::Error::malformed_config(
                        "Hook release revolutions are not well-defined",
                    )
                })?,
                on_motors_pulled: StateName::try_new(&on_motors_pulled, names)?,
            }
            .into_box(),
            StateLogicConfig::ArmRecovery {
                arming_state,
                on_state_set,
            } => state_logic::ArmRecovery {
                arming_state,
                on_state_set: StateName::try_new(&on_state_set, names)?,
            }
            .into_box(),
            StateLogicConfig::CameraPowerControl {
                camera_power_state,
                on_state_set,
            } => state_logic::CameraPowerControl {
                camera_power_state,
                on_state_set: StateName::try_new(&on_state_set, names)?,
            }
            .into_box(),
            StateLogicConfig::ManualControl { allowed_actuators } => state_logic::ManualControl {
                manual_transitions,
                actuator_control: Arc::new(
                    ActuatorControl::from_strings(allowed_actuators.iter().map(String::as_str))
                        .map_err(lib_mission::Error::malformed_config)?,
                ),
            }
            .into_box(),
            StateLogicConfig::SetPSSMode { mode, on_done } => state_logic::SetPSSMode {
                mode,
                on_done: StateName::try_new(&on_done, names)?,
            }
            .into_box(),
            StateLogicConfig::SetNormalState { on_done } => state_logic::SetNormalState {
                on_done: StateName::try_new(&on_done, names)?,
            }
            .into_box(),
            StateLogicConfig::SetActuators { targets, on_done } => state_logic::SetActuators {
                targets,
                on_done: StateName::try_new(&on_done, names)?,
            }
            .into_box(),
            StateLogicConfig::ReadRFSArming {
                on_armed,
                on_not_armed,
            } => state_logic::ReadRFSArming {
                on_armed: StateName::try_new(&on_armed, names)?,
                on_not_armed: StateName::try_new(&on_not_armed, names)?,
            }
            .into_box(),
            StateLogicConfig::AwaitRFSMode { mode, on_detected } => state_logic::AwaitRFSMode {
                manual_transitions,
                mode,
                on_detected: StateName::try_new(&on_detected, names)?,
            }
            .into_box(),
            StateLogicConfig::AwaitRFSModeWhileConnected {
                mode,
                on_detected,
                rfs_can_timeout_sec,
                on_timeout,
            } => state_logic::AwaitRFSModeWhileConnected {
                manual_transitions,
                mode,
                on_detected: StateName::try_new(&on_detected, names)?,
                rfs_can_timeout: Duration::from_secs_f32(rfs_can_timeout_sec),
                on_timeout: StateName::try_new(&on_timeout, names)?,
            }
            .into_box(),
            StateLogicConfig::WaitWhileConnected {
                rfs_can_timeout_sec,
                on_timeout,
            } => state_logic::WaitWhileConnected {
                manual_transitions,
                rfs_can_timeout: Duration::from_secs_f32(rfs_can_timeout_sec),
                on_timeout: StateName::try_new(&on_timeout, names)?,
            }
            .into_box(),
            StateLogicConfig::Ignition { on_done } => state_logic::Ignition {
                on_done: StateName::try_new(&on_done, names)?,
            }
            .into_box(),
            StateLogicConfig::AbortIgnition { on_done } => state_logic::AbortIgnition {
                on_done: StateName::try_new(&on_done, names)?,
            }
            .into_box(),
            StateLogicConfig::LiftoffDetection {
                acceleration_limit_ms2,
                required_duration_s,
                on_liftoff_detected,
                on_ecu_aborted,
            } => state_logic::LiftoffDetection {
                acceleration_limit: acceleration_limit_ms2,
                required_duration: Duration::from_secs_f32(required_duration_s),
                on_liftoff_detected: StateName::try_new(&on_liftoff_detected, names)?,
                on_ecu_aborted: StateName::try_new(&on_ecu_aborted, names)?,
                manual_transitions,
            }
            .into_box(),
            StateLogicConfig::ApogeeTrigger {
                trigger_before_apogee_time_s,
                on_apogee_reached,
            } => state_logic::ApogeeTrigger {
                trigger_before_apogee_time: Duration::from_secs_f32(trigger_before_apogee_time_s),
                on_apogee_reached: StateName::try_new(&on_apogee_reached, names)?,
                manual_transitions,
            }
            .into_box(),
            StateLogicConfig::Thrust {
                abort_downward_velocity_m_s,
                on_apogee_reached,
                on_firing_finished,
            } => state_logic::Thrust {
                abort_downward_velocity_m_s,
                on_apogee_reached: StateName::try_new(&on_apogee_reached, names)?,
                on_firing_finished: StateName::try_new(&on_firing_finished, names)?,
                manual_transitions,
            }
            .into_box(),
            StateLogicConfig::MonitoredManualControl {
                allowed_actuators,
                thresholds,
                telemetry_loss_threshold_sec,
                on_violation,
            } => state_logic::MonitoredManualControl {
                manual_transitions,
                allowed_actuators: Arc::new(
                    ActuatorControl::from_strings(allowed_actuators.iter().map(String::as_str))
                        .map_err(lib_mission::Error::malformed_config)?,
                ),
                telemetry_loss_threshold: std::time::Duration::from_secs_f32(
                    telemetry_loss_threshold_sec,
                ),
                thresholds,
                on_violation: StateName::try_new(&on_violation, names)?,
            }
            .into_box(),
            StateLogicConfig::Shutdown => common_states::Shutdown::default().into_box(),
        })
    }
}
