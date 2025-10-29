use std::time::Duration;

use hermes_can::messages;
use lib_mission::actuator_control::Energizable;
use lib_mission::actuator_control::valves::NormallyClosedValve;
use lib_mission::reactive::Watched;
use lib_mission::util::init::BuildInfo;
use lib_mission::util::types::{BarG, Celsius};
use tokio::task::AbortHandle;
use tracing::Instrument;
use uremote_electronics::types::SparkPlug;

/// Sends [`messages::board_status::RfsComputerStatus`] at 1Hz.
#[allow(clippy::cast_possible_truncation)] // u64 micros since start
pub fn spawn_rfsc_status_driver(
    has_pss_control: impl Watched<bool> + Send + 'static,
    is_ignition_armed: impl Watched<bool> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs(1));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            let start_time = std::time::SystemTime::now();
            loop {
                interval.tick().await;
                let msg = messages::Message::RfsComputerStatus(
                    messages::board_status::RfsComputerStatus {
                        common: messages::board_status::StatusCommonMessage {
                            errors: 0,
                            micros_since_restart: std::time::SystemTime::now()
                                .duration_since(start_time)
                                .expect("Time moved backwards")
                                .as_micros()
                                as u64,
                        },
                        has_pss_control: has_pss_control.current(),
                        is_ignition_armed: arming_state_from_bool(is_ignition_armed.current()),
                    },
                );

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::RfsComputerStatus")),
    )
    .abort_handle()
}

/// Sends [`messages::debug_info::FlightComputerBuildInfo`] at 0.2Hz.
#[allow(clippy::needless_pass_by_value)] // needed to move `build_info` to future. Is a const anyway.
pub fn spawn_rfsc_debug_driver(
    build_info: BuildInfo,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    let commit_hash: [u8; 7] = build_info.git_commit.as_bytes()[..7]
        .try_into()
        .expect("git commit is long enough and then the size will match");
    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs(5));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                interval.tick().await;
                let msg = messages::Message::RfsComputerBuildInfo(
                    messages::debug_info::RfsComputerBuildInfo {
                        data: messages::debug_info::BuildInformationCommon {
                            unix_timestamp: build_info.unix_timestamp,
                            author_initials: build_info.author_initials,
                            is_release: build_info.is_release,
                            debug_defmt_rtt: false, // Not applicable to RFSC
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
        .instrument(tracing::info_span!("drivers::RfsComputerBuild")),
    )
    .abort_handle()
}

/// Sends [`messages::sensor_data::IgniterPressure`] at 30Hz (not 40Hz as not that much data is available) to the ECU.
#[allow(clippy::cast_possible_truncation)] // u64 micros since start
pub fn spawn_ecu_igniter_pressure_driver(
    igniter_pressure: impl Watched<BarG> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    const PRESSURE_RATE: f32 = 30.;

    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs_f32(1. / PRESSURE_RATE));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                interval.tick().await;
                let msg =
                    messages::Message::IgniterPressure(messages::sensor_data::IgniterPressure {
                        igniter_pressure: igniter_pressure.current().as_f32(),
                    });

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::ECUIgniterPressure")),
    )
    .abort_handle()
}

/// Sends [`messages::sensor_data::IgniterTemperature`] at 10Hz to the ECU.
#[allow(clippy::cast_possible_truncation)] // u64 micros since start
pub fn spawn_ecu_igniter_temperature_driver(
    igniter_temperature: impl Watched<Celsius> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    const TEMP_RATE: f32 = 10.;

    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs_f32(1. / TEMP_RATE));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                interval.tick().await;
                let msg = messages::Message::IgniterTemperature(
                    messages::sensor_data::IgniterTemperature {
                        igniter_temperature: igniter_temperature.current().as_f32(),
                    },
                );

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::ECUIgniterTemperature")),
    )
    .abort_handle()
}

/// Sends [`messages::board_status::RfsComputerValveStates`] at 5Hz to the ECU.
#[allow(clippy::cast_possible_truncation)] // u64 micros since start
pub fn spawn_ecu_valve_state_driver(
    h2_igniter_valve: impl Watched<NormallyClosedValve> + Send + 'static,
    gox_igniter_valve: impl Watched<NormallyClosedValve> + Send + 'static,
    h2_purge_valve: impl Watched<NormallyClosedValve> + Send + 'static,
    gox_purge_valve: impl Watched<NormallyClosedValve> + Send + 'static,
    igniter_spark: impl Watched<SparkPlug> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    const FEEDBACK_RATE: f32 = 5.;

    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs_f32(1. / FEEDBACK_RATE));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                interval.tick().await;
                let msg = messages::Message::RfsComputerValveStates(
                    messages::board_status::RfsComputerValveStates {
                        gox_ign_vlv: valve_state_from_energizable(&gox_igniter_valve.current()),
                        h2_ign_vlv: valve_state_from_energizable(&h2_igniter_valve.current()),
                        ign_spk_plg: valve_state_from_energizable(&igniter_spark.current()),
                        gox_prg_vlv: valve_state_from_energizable(&gox_purge_valve.current()),
                        h2_prg_vlv: valve_state_from_energizable(&h2_purge_valve.current()),
                    },
                );

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::ECURfsValveState")),
    )
    .abort_handle()
}

/// Gets the CAN active/inactive state based on an `Energizable`
fn valve_state_from_energizable(valve: &impl Energizable) -> messages::board_status::ValveState {
    if valve.is_active() {
        messages::board_status::ValveState::Active
    } else {
        messages::board_status::ValveState::Inactive
    }
}

fn arming_state_from_bool(is_armed: bool) -> messages::board_status::ArmingState {
    if is_armed {
        messages::board_status::ArmingState::Armed
    } else {
        messages::board_status::ArmingState::Safe
    }
}
