use std::time::Duration;

use datatypes::actuator::{DigitalOut, NormallyClosedValve};
use datatypes::sensor::DigitalIn;
use datatypes::status::BuildInformationCommon;
use datatypes::units::BarG;
use lib_mission::reactive::Watched;
use lib_mission::util::init::BuildInfo;
use messages::datapoints::dp_rfs_computer;
use tokio::task::AbortHandle;
use tracing::Instrument;
use watchable::Watchable;

/// Sends `RfsComputerStatus` at 1Hz.
#[allow(clippy::cast_possible_truncation)] // u64 micros since start
pub fn spawn_rfsc_status_driver(
    rocket_connection_detect: Watchable<DigitalIn>,
    rfsc_status: Watchable<dp_rfs_computer::RfsComputerStatus>,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs(1));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                interval.tick().await;
                let status = rfsc_status.get();
                let msg = messages::Message::RfsComputer(dp_rfs_computer::Message::Status(status));

                if matches!(rocket_connection_detect.get(), DigitalIn::Off) {
                    continue;
                }

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

/// Sends `FlightComputerBuildInfo` at 0.2Hz.
#[allow(clippy::needless_pass_by_value)] // needed to move `build_info` to future. Is a const anyway.
pub fn spawn_rfsc_debug_driver(
    rocket_connection_detect: Watchable<DigitalIn>,
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
                let msg = messages::Message::RfsComputer(dp_rfs_computer::Message::BuildInfo(
                    BuildInformationCommon {
                        unix_timestamp: build_info.unix_timestamp,
                        author_initials: build_info.author_initials,
                        is_release: build_info.is_release,
                        debug_defmt_rtt: false, // Not applicable to RFSC
                        commit_hash,
                        is_git_dirty: build_info.git_dirty,
                        can_semver: [
                            dp_rfs_computer::VERSION_MAJOR,
                            dp_rfs_computer::VERSION_MINOR,
                            dp_rfs_computer::VERSION_PATCH,
                        ],
                    },
                ));

                if matches!(rocket_connection_detect.get(), DigitalIn::Off) {
                    continue;
                }

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

/// Sends `IgniterPressure` at 30Hz (not 40Hz as not that much data is available) to the FC.
#[allow(clippy::cast_possible_truncation)] // u64 micros since start
pub fn spawn_fc_igniter_pressure_driver(
    rocket_connection_detect: Watchable<DigitalIn>,
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
                let msg = messages::Message::RfsComputer(
                    dp_rfs_computer::Message::IgniterPressure(BarG(igniter_pressure.current().0)),
                );

                if matches!(rocket_connection_detect.get(), DigitalIn::Off) {
                    continue;
                }

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::FCIgniterPressure")),
    )
    .abort_handle()
}

/// Sends `GoxIgniterValveState` at 5Hz.
pub fn spawn_gox_igniter_valve_state_driver(
    rocket_connection_detect: Watchable<DigitalIn>,
    state: impl Watched<NormallyClosedValve> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    const STATE_RATE: f32 = 5.;

    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs_f32(1. / STATE_RATE));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                interval.tick().await;
                let msg = messages::Message::RfsComputer(
                    dp_rfs_computer::Message::GoxIgniterValveState(state.current()),
                );

                if matches!(rocket_connection_detect.get(), DigitalIn::Off) {
                    continue;
                }

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::GoxIgniterValveState")),
    )
    .abort_handle()
}

/// Sends `H2IgniterValveState` at 5Hz.
pub fn spawn_h2_igniter_valve_state_driver(
    rocket_connection_detect: Watchable<DigitalIn>,
    state: impl Watched<NormallyClosedValve> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    const STATE_RATE: f32 = 5.;

    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs_f32(1. / STATE_RATE));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                interval.tick().await;
                let msg = messages::Message::RfsComputer(
                    dp_rfs_computer::Message::H2IgniterValveState(state.current()),
                );

                if matches!(rocket_connection_detect.get(), DigitalIn::Off) {
                    continue;
                }

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::H2IgniterValveState")),
    )
    .abort_handle()
}

/// Sends `GoxPurgeValveState` at 5Hz.
pub fn spawn_gox_purge_valve_state_driver(
    rocket_connection_detect: Watchable<DigitalIn>,
    state: impl Watched<NormallyClosedValve> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    const STATE_RATE: f32 = 5.;

    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs_f32(1. / STATE_RATE));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                interval.tick().await;
                let msg = messages::Message::RfsComputer(
                    dp_rfs_computer::Message::GoxPurgeValveState(state.current()),
                );

                if matches!(rocket_connection_detect.get(), DigitalIn::Off) {
                    continue;
                }

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::GoxPurgeValveState")),
    )
    .abort_handle()
}

/// Sends `H2PurgeValveState` at 5Hz.
pub fn spawn_h2_purge_valve_state_driver(
    rocket_connection_detect: Watchable<DigitalIn>,
    state: impl Watched<NormallyClosedValve> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    const STATE_RATE: f32 = 5.;

    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs_f32(1. / STATE_RATE));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                interval.tick().await;
                let msg = messages::Message::RfsComputer(
                    dp_rfs_computer::Message::H2PurgeValveState(state.current()),
                );

                if matches!(rocket_connection_detect.get(), DigitalIn::Off) {
                    continue;
                }

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::H2PurgeValveState")),
    )
    .abort_handle()
}

/// Sends `IgniterSparkPlugState` at 5Hz.
pub fn spawn_igniter_spark_plug_state_driver(
    rocket_connection_detect: Watchable<DigitalIn>,
    state: impl Watched<DigitalOut> + Send + 'static,
    tx: std::sync::mpsc::Sender<messages::Message>,
) -> AbortHandle {
    const STATE_RATE: f32 = 5.;

    tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(Duration::from_secs_f32(1. / STATE_RATE));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                interval.tick().await;
                let msg = messages::Message::RfsComputer(
                    dp_rfs_computer::Message::IgniterSparkPlugState(state.current()),
                );

                if matches!(rocket_connection_detect.get(), DigitalIn::Off) {
                    continue;
                }

                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel was closed. Shutting down!");
                    break;
                }
            }
        }
        .instrument(tracing::info_span!("drivers::IgniterSparkPlugState")),
    )
    .abort_handle()
}
