use std::time::Duration;

use hermes_can::messages;
use lib_mission::reactive::{Repeatedly, Watched, WatchedExt};
use lib_mission::util::init::BuildInfo;
use lib_mission::util::time::MissionTime;
use tokio::task::AbortHandle;
use tracing::Instrument;

use crate::io::types::{
    AbortFuelPressurizationState, AbortIgnitionState, AbortOxidizerPressurizationState,
    CameraPowerState, FuelPressurizationState, IgnitionState, MainDeploymentState,
    OxidizerPressurizationState, RecoveryArmingState, SeparationState, SteeringMotorRevolutions,
};

/// Sends [`messages::event_messages::SteeringTargetPositions`].
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

                let msg = messages::Message::SteeringTargetPositions(
                    messages::event_messages::SteeringTargetPositions {
                        left_pos: defl.left_motor_steps,
                        right_pos: defl.right_motor_steps,
                    },
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

/// Sends [`messages::event_messages::RecoveryPowerConfig`].
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
                    let msg = messages::Message::RecoveryPowerConfig(
                        messages::event_messages::RecoveryPowerConfig {
                            separation_enabled: chutes_power,
                            deployment_enabled: chutes_power,
                            steering_enabled: steering_power,
                        },
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

/// Sends [`messages::event_messages::CameraPowerControl`]
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
            let msg = messages::Message::CameraPowerControl(
                messages::event_messages::CameraPowerControl {
                    live_pwr_enabled: camera_on,
                    recording_pwr_enabled: camera_on,
                },
            );

            if tx.send(msg).is_err() {
                tracing::warn!("CAN channel was closed. Shutting down!");
                break;
            }
        }
    })
    .abort_handle()
}

/// Sends [`messages::event_messages::SeparationTrigger`].
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
                    let msg = messages::Message::SeparationTrigger(
                        messages::event_messages::SeparationTrigger,
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

/// Sends [`messages::event_messages::DeploymentTrigger`].
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
                    let msg = messages::Message::DeploymentTrigger(
                        messages::event_messages::DeploymentTrigger,
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

/// Sends [`messages::board_status::FlightComputerStatus`] at 1Hz.
#[allow(clippy::cast_possible_truncation)] // u64 micros since start
pub fn spawn_flight_computer_status_driver(
    has_pss_control: impl Watched<bool> + Send + 'static,
    is_software_armed: impl Watched<RecoveryArmingState> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs(1));
            let start_time = std::time::SystemTime::now();
            loop {
                interval.tick().await;
                let msg = messages::Message::FlightComputerStatus(
                    messages::board_status::FlightComputerStatus {
                        common: messages::board_status::StatusCommonMessage {
                            errors: 0,
                            micros_since_restart: std::time::SystemTime::now()
                                .duration_since(start_time)
                                .expect("Time moved backwards")
                                .as_micros()
                                as u64,
                        },
                        has_pss_control: has_pss_control.current(),
                        is_software_armed: is_software_armed.current().into(),
                    },
                );

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

/// Sends [`messages::debug_info::FlightComputerBuildInfo`] at 0.2Hz.
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
                let msg = messages::Message::FlightComputerBuildInfo(
                    messages::debug_info::FlightComputerBuildInfo {
                        data: messages::debug_info::BuildInformationCommon {
                            unix_timestamp: build_info.unix_timestamp,
                            author_initials: build_info.author_initials,
                            is_release: build_info.is_release,
                            debug_defmt_rtt: false, // Not applicable to FC
                            commit_hash,
                            is_git_dirty: build_info.git_dirty,
                            can_semver: [
                                hermes_can::VERSION_MAJOR,
                                hermes_can::VERSION_MINOR,
                                hermes_can::VERSION_PATCH,
                            ],
                        },
                    },
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
impl From<RecoveryArmingState> for messages::board_status::ArmingState {
    fn from(value: RecoveryArmingState) -> Self {
        match value {
            RecoveryArmingState::SteeringMotorsLocked | RecoveryArmingState::Unarmed => {
                messages::board_status::ArmingState::Safe
            }
            RecoveryArmingState::Armed => messages::board_status::ArmingState::Armed,
        }
    }
}

/// Sends [`messages::event_messages::FiringInitiation`].
pub fn spawn_ignition_driver(
    mut watched_sep: impl Watched<IgnitionState> + Send + 'static,
    mut clock: impl Watched<MissionTime> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            loop {
                let IgnitionState::Ignited = watched_sep.next().await else {
                    continue;
                };
                tracing::info!("ECU starts ignition sequence");

                // Send it 2 times with 50ms in between.
                for _ in 0..2 {
                    let msg = messages::Message::FiringInitiation(
                        messages::event_messages::FiringInitiation,
                    );

                    if tx.send(msg).is_err() {
                        tracing::warn!("CAN channel was closed. Shutting down!");
                        break;
                    }

                    let wait_end = clock.current() + Duration::from_millis(50);
                    clock.next_satisfying(|t| *t >= wait_end).await;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::Ignition")),
    )
    .abort_handle()
}

/// Sends [`messages::event_messages::FiringAbortInitiation`].
pub fn spawn_abort_ignition_driver(
    mut watched_sep: impl Watched<AbortIgnitionState> + Send + 'static,
    mut clock: impl Watched<MissionTime> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            loop {
                let AbortIgnitionState::Aborted = watched_sep.next().await else {
                    continue;
                };
                tracing::info!("ECU aborts ignition sequence");

                // Send it 5 times over 1 sec, just to be sure if CAN has weird problems.
                for _ in 0..5 {
                    let msg = messages::Message::FiringAbortInitiation(
                        messages::event_messages::FiringAbortInitiation,
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
        .instrument(tracing::info_span!("drivers::AbortIgnition")),
    )
    .abort_handle()
}

/// Sends [`messages::event_messages::ThrustCurveConfig`] at 0.1Hz.
pub fn spawn_thrust_curve_driver(
    thrust_curve: hermes_can::messages::event_messages::ThrustCurveConfig,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    // Send the ECU config
    tokio::spawn(
        async move {
            tokio::time::sleep(std::time::Duration::from_secs(2)).await;
            loop {
                tokio::time::sleep(std::time::Duration::from_secs(10)).await;
                if tx
                    .send(hermes_can::messages::Message::ThrustCurveConfig(
                        thrust_curve.clone(),
                    ))
                    .is_err()
                {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::ThrustCurve")),
    )
    .abort_handle()
}

/// Sends [`messages::event_messages::FuelPressurization`].
pub fn spawn_fuel_pressurization_driver(
    mut watched_sep: impl Watched<FuelPressurizationState> + Send + 'static,
    mut clock: impl Watched<MissionTime> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            loop {
                let FuelPressurizationState::Pressurized(target_pressure) =
                    watched_sep.next().await
                else {
                    continue;
                };
                tracing::info!("ECU starts fuel pressurization sequence");

                // Send it 2 times with 50ms in between.
                for _ in 0..2 {
                    let msg = messages::Message::FuelPressurization(
                        messages::event_messages::FuelPressurization { target_pressure },
                    );

                    if tx.send(msg).is_err() {
                        tracing::warn!("CAN channel was closed. Shutting down!");
                        break;
                    }

                    let wait_end = clock.current() + Duration::from_millis(50);
                    clock.next_satisfying(|t| *t >= wait_end).await;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::FuelPressurization")),
    )
    .abort_handle()
}

/// Sends [`messages::event_messages::OxidizerPressurization`].
pub fn spawn_oxidizer_pressurization_driver(
    mut watched_sep: impl Watched<OxidizerPressurizationState> + Send + 'static,
    mut clock: impl Watched<MissionTime> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            loop {
                let OxidizerPressurizationState::Pressurized(target_pressure) =
                    watched_sep.next().await
                else {
                    continue;
                };
                tracing::info!("ECU starts oxidizer pressurization sequence");

                // Send it 2 times with 50ms in between.
                for _ in 0..2 {
                    let msg = messages::Message::OxidizerPressurization(
                        messages::event_messages::OxidizerPressurization { target_pressure },
                    );

                    if tx.send(msg).is_err() {
                        tracing::warn!("CAN channel was closed. Shutting down!");
                        break;
                    }

                    let wait_end = clock.current() + Duration::from_millis(50);
                    clock.next_satisfying(|t| *t >= wait_end).await;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::OxidizerPressurization")),
    )
    .abort_handle()
}

/// Sends [`messages::event_messages::FuelPressurizationAbort`].
pub fn spawn_abort_fuel_pressurization_driver(
    mut watched_sep: impl Watched<AbortFuelPressurizationState> + Send + 'static,
    mut clock: impl Watched<MissionTime> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            loop {
                let AbortFuelPressurizationState::Aborted = watched_sep.next().await else {
                    continue;
                };
                tracing::info!("ECU aborts fuel pressurization sequence");

                // Send it 5 times over 1/2 sec, just to be sure if CAN has weird problems.
                for _ in 0..5 {
                    let msg = messages::Message::FuelPressurizationAbort(
                        messages::event_messages::FuelPressurizationAbort,
                    );

                    if tx.send(msg).is_err() {
                        tracing::warn!("CAN channel was closed. Shutting down!");
                        break;
                    }

                    let wait_end = clock.current() + Duration::from_millis(100);
                    clock.next_satisfying(|t| *t >= wait_end).await;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::AbortFuelPressurization")),
    )
    .abort_handle()
}

/// Sends [`messages::event_messages::OxidizerPressurizationAbort`].
pub fn spawn_abort_oxidizer_pressurization_driver(
    mut watched_sep: impl Watched<AbortOxidizerPressurizationState> + Send + 'static,
    mut clock: impl Watched<MissionTime> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            loop {
                let AbortOxidizerPressurizationState::Aborted = watched_sep.next().await else {
                    continue;
                };
                tracing::info!("ECU aborts oxidizer pressurization sequence");

                // Send it 5 times over 1/2 sec, just to be sure if CAN has weird problems.
                for _ in 0..5 {
                    let msg = messages::Message::OxidizerPressurizationAbort(
                        messages::event_messages::OxidizerPressurizationAbort,
                    );

                    if tx.send(msg).is_err() {
                        tracing::warn!("CAN channel was closed. Shutting down!");
                        break;
                    }

                    let wait_end = clock.current() + Duration::from_millis(100);
                    clock.next_satisfying(|t| *t >= wait_end).await;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::AbortOxidizerPressurization")),
    )
    .abort_handle()
}
