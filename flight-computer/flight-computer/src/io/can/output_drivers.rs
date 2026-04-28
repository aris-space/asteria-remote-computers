use std::time::Duration;

use datatypes::actuator::{CameraPowerStatus};
use datatypes::status::{ArmingState, BuildInformationCommon, StatusCommonMessage};
use dp_flight_computer::FlightComputerStatus;
use dp_recovery_board::{RecoveryPowerConfig, SteeringPositions};
use lib_mission::reactive::{Repeatedly, Watched, WatchedExt};
use lib_mission::util::init::BuildInfo;
use lib_mission::util::time::MissionTime;
use messages::datapoints::{dp_flight_computer, dp_power_board, dp_recovery_board};
use tokio::task::AbortHandle;
use tracing::Instrument;
use watchable::Watchable;

use crate::io::types::{
    CameraPowerState, MainDeploymentState, RecoveryArmingState, SeparationState,
    SteeringMotorRevolutions,
};

/// Sends [`dp_recovery_board::Message::SteeringTargetPositions`].
pub fn spawn_steering_motor_driver(
    mut watched_deflections: impl Watched<Option<SteeringMotorRevolutions>> + Send + 'static,
    clock: impl Watched<MissionTime> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            let mut repeatedly = Repeatedly::new(clock, Duration::from_secs_f32(1. / 5.)); // 5 Hz
            loop {
                let defl = tokio::select! {
                    defl = watched_deflections.next() => defl,
                    _ = repeatedly.next() => watched_deflections.current(),
                };
                let Some(defl) = defl else {
                    continue;
                };

                let msg = messages::Message::RecoveryBoard(
                    dp_recovery_board::Message::SteeringTargetPositions(SteeringPositions {
                        left_pos: defl.left_motor_steps,
                        right_pos: defl.right_motor_steps,
                    }),
                );

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::SteeringMotor")),
    )
    .abort_handle()
}

/// Sends [`dp_recovery_board::Message::RecoveryPowerConfig`].
pub fn spawn_recovery_arming_driver(
    mut watched_arming: impl Watched<RecoveryArmingState> + Send + 'static,
    mut clock: impl Watched<MissionTime> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            loop {
                let state = watched_arming.next().await;

                let steering_power = matches!(
                    state,
                    RecoveryArmingState::SteeringMotorsLocked | RecoveryArmingState::Armed
                );
                if steering_power {
                    tracing::info!("powering steering motores");
                } else {
                    tracing::info!("unpowering steering motores");
                }

                let chutes_power = matches!(state, RecoveryArmingState::Armed);
                if chutes_power {
                    tracing::info!("powering separation/deployment motors");
                } else {
                    tracing::info!("unpowering separation/deployment motors");
                }

                // Send it 2 times 100ms apart. Shouldn't wait much longer as this can be called often.
                for _ in 0..2 {
                    let msg = messages::Message::RecoveryBoard(
                        dp_recovery_board::Message::RecoveryPowerConfig(RecoveryPowerConfig {
                            separation_enabled: chutes_power,
                            deployment_enabled: chutes_power,
                            steering_enabled: steering_power,
                        }),
                    );

                    if tx.send(msg).is_err() {
                        tracing::warn!("CAN channel was closed. Shutting down!");
                        break;
                    }

                    let wait_end = clock.current() + Duration::from_millis(200);
                    clock.next_satisfying(|t| *t >= wait_end).await;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::RecoveryArming")),
    )
    .abort_handle()
}

/// Sends [`dp_power_board::Message::CameraPowerControl`]
pub fn spawn_camera_driver(
    mut watched_camera_power: impl Watched<CameraPowerState> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(async move {
        loop {
            let camera_on = watched_camera_power.next().await.is_on();
            if camera_on {
                tracing::info!("turning camera on");
            } else {
                tracing::info!("turning camera off");
            }

            // Send the message 1 time, we suppose that CAN is reliable.
            let msg = messages::Message::PowerBoard(dp_power_board::Message::CameraPowerControl(
                CameraPowerStatus {
                    live_pwr_enabled: camera_on,
                    recording_pwr_enabled: camera_on,
                },
            ));

            if tx.send(msg).is_err() {
                tracing::warn!("CAN channel was closed. Shutting down!");
                break;
            }
        }
    })
    .abort_handle()
}

/// Sends [`dp_recovery_board::Message::SeparationTrigger`].
pub fn spawn_separation_driver(
    mut watched_sep: impl Watched<SeparationState> + Send + 'static,
    mut clock: impl Watched<MissionTime> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            loop {
                let SeparationState::Separated = watched_sep.next().await else {
                    continue;
                };
                tracing::info!("separating");

                // Send it 10 times over 2 sec, just to be sure if CAN has weird problems.
                for _ in 0..10 {
                    let msg = messages::Message::RecoveryBoard(
                        dp_recovery_board::Message::SeparationTrigger,
                    );

                    if tx.send(msg).is_err() {
                        tracing::warn!("CAN channel was closed. Shutting down!");
                        break;
                    }

                    let wait_end = clock.current() + Duration::from_millis(200);
                    clock.next_satisfying(|t| *t >= wait_end).await;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::Separation")),
    )
    .abort_handle()
}

/// Sends [`dp_recovery_board::Message::DeploymentTrigger`].
pub fn spawn_deployment_driver(
    mut watched_sep: impl Watched<MainDeploymentState> + Send + 'static,
    mut clock: impl Watched<MissionTime> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            loop {
                let MainDeploymentState::Deployed = watched_sep.next().await else {
                    continue;
                };
                tracing::info!("deploying main chute");

                // Send it 10 times over 2 sec, just to be sure if CAN has weird problems.
                for _ in 0..10 {
                    let msg = messages::Message::RecoveryBoard(
                        dp_recovery_board::Message::DeploymentTrigger,
                    );

                    if tx.send(msg).is_err() {
                        tracing::warn!("CAN channel was closed. Shutting down!");
                        break;
                    }

                    let wait_end = clock.current() + Duration::from_millis(200);
                    clock.next_satisfying(|t| *t >= wait_end).await;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::Deployment")),
    )
    .abort_handle()
}

/// Sends [`dp_flight_computer::Message::Status`] at 1Hz.
#[allow(clippy::cast_possible_truncation)] // u64 micros since start
pub fn spawn_flight_computer_status_driver(
    has_pss_control: impl Watched<bool> + Send + 'static,
    is_software_armed: impl Watched<RecoveryArmingState> + Send + 'static,
    fc_status: Watchable<FlightComputerStatus>,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs(1));
            let start_time = std::time::SystemTime::now();
            loop {
                interval.tick().await;
                let status = FlightComputerStatus {
                    common: StatusCommonMessage {
                        errors: 0,
                        micros_since_restart: std::time::SystemTime::now()
                            .duration_since(start_time)
                            .expect("Time moved backwards")
                            .as_micros() as u64,
                    },
                    has_pss_control: has_pss_control.current(),
                    is_software_armed: is_software_armed.current().into(),
                };
                let _ = fc_status.update(status.clone());
                let msg =
                    messages::Message::FlightComputer(dp_flight_computer::Message::Status(status));

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::FlightComputerStatus")),
    )
    .abort_handle()
}

/// Sends [`dp_flight_computer::Message::BuildInfo`] at 0.2Hz.
#[allow(clippy::needless_pass_by_value)] // needed to move `build_info` to future. Is a const anyway.
pub fn spawn_flight_computer_debug_driver(
    build_info: BuildInfo,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    let commit_hash: [u8; 7] = build_info.git_commit.as_bytes()[..7]
        .try_into()
        .expect("git commit is long enough and then the size will match");
    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs(5));
            loop {
                interval.tick().await;
                let msg = messages::Message::FlightComputer(
                    dp_flight_computer::Message::BuildInfo(BuildInformationCommon {
                        unix_timestamp: build_info.unix_timestamp,
                        author_initials: build_info.author_initials,
                        is_release: build_info.is_release,
                        debug_defmt_rtt: false, // Not applicable to FC
                        commit_hash,
                        is_git_dirty: build_info.git_dirty,
                        can_semver: [
                            dp_flight_computer::VERSION_MAJOR,
                            dp_flight_computer::VERSION_MINOR,
                            dp_flight_computer::VERSION_PATCH,
                        ],
                    }),
                );

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::FlightComputerBuild")),
    )
    .abort_handle()
}

/// Converts the more fined grained arming state into the software arming state necessary for the state machine.
impl From<RecoveryArmingState> for ArmingState {
    fn from(value: RecoveryArmingState) -> Self {
        match value {
            RecoveryArmingState::SteeringMotorsLocked | RecoveryArmingState::Unarmed => {
                ArmingState::Safe
            }
            RecoveryArmingState::Armed => ArmingState::Armed,
        }
    }
}
