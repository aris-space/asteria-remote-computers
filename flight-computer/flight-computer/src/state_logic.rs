pub type Input = crate::io::input::InputTree;
pub type Output = crate::io::output::OutputTree;
pub type Parameter = ();

pub(crate) mod helpers;

pub mod await_timer;
pub use await_timer::AwaitTimer;

pub mod monitored_await;
pub use monitored_await::MonitoredWaiting;

pub mod diagnostics;
pub use diagnostics::Diagnostics;

pub mod height_or_time_trigger;
pub use height_or_time_trigger::HeightOrTimeTrigger;

pub mod height_time_or_cats_deployment_trigger;
pub use height_time_or_cats_deployment_trigger::HeightTimeOrCATSDeplTrigger;

pub mod separation;
pub use separation::Separate;

pub mod main_chute_deployment;
pub use main_chute_deployment::MainChuteDeployment;

pub mod hook_release;
pub use hook_release::HookRelease;

pub mod control_loop;
pub use control_loop::ControlLoop;

pub mod safety_spiral;
pub use safety_spiral::SafetySpiral;

pub mod recovery_arming;
pub use recovery_arming::ArmRecovery;

pub mod camera_power_control;
pub use camera_power_control::CameraPowerControl;

mod manual_control;
pub use manual_control::ManualControl;

pub mod set_normal_state;
pub use set_normal_state::SetNormalState;

pub mod set_actuators;
pub use set_actuators::SetActuators;

pub mod set_pss_mode;
pub use set_pss_mode::SetPSSMode;

pub mod read_rfs_arming;
pub use read_rfs_arming::ReadRFSArming;

pub mod await_rfs_mode;
pub use await_rfs_mode::AwaitRFSMode;

pub mod await_rfs_mode_while_connected;
pub use await_rfs_mode_while_connected::AwaitRFSModeWhileConnected;

pub mod wait_while_connected;
pub use wait_while_connected::WaitWhileConnected;

pub mod ignition;
pub use ignition::Ignition;

pub mod abort_ignition;
pub use abort_ignition::AbortIgnition;

pub mod liftoff_detection;
pub use liftoff_detection::LiftoffDetection;

pub mod apogee_trigger;
pub use apogee_trigger::ApogeeTrigger;

pub mod pressurization;
pub use pressurization::Pressurization;

pub mod await_pressurization;
pub use await_pressurization::AwaitPressurization;

pub mod abort_pressurization;
pub use abort_pressurization::AbortPressurization;

pub mod thrust;
pub use thrust::Thrust;

pub mod monitored_manual_control;
pub use monitored_manual_control::MonitoredManualControl;
