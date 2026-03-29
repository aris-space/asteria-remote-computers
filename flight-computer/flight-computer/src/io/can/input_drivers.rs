use datatypes::status::ArmingState;
use datatypes::units::RailStatus;
use lib_mission::can::ConsumerRegistry;
use lib_mission::reactive::Watched;
use lib_mission::util::time::MissionTime;
use messages::datapoints::dp_engine_control_board;
use messages::datapoints::dp_fuel_control_board;
use messages::datapoints::dp_oxidizer_control_board;
use messages::datapoints::dp_power_board;
use messages::datapoints::dp_recovery_board;
use messages::datapoints::dp_rfs_computer;
use messages::datapoints::dp_rfs_computer::RfsComputerStatus;
use messages::datapoints::dp_sensor_carrier;
use messages::datapoints::dp_sensor_carrier::EnvironmentalData;
use tokio::task::AbortHandle;
use tracing::Instrument as _;
use watchable::Watchable;

use crate::io::input::BoardsHealthyAsOf;
use crate::io::types::SteeringMotorRevolutions;

/// Listens for board_status messages and updates [`BoardsHealthyAsOf`].
///
/// Additionally stores the full RFS computer status and arming states.
#[allow(clippy::too_many_lines)] // fine, probably will be refactored soon
pub fn spawn_and_register_board_health_driver(
    registry: &mut ConsumerRegistry,
    health: BoardsHealthyAsOf,
    rfs_status: Watchable<(RfsComputerStatus, MissionTime)>,
    recovery_armed: Watchable<ArmingState>,
    ecu_armed: Watchable<ArmingState>,
    clock: impl Watched<MissionTime> + Send + Sync + 'static,
) -> AbortHandle {
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
    for id in [
        // FIXME: extracting the id like this is ugly, but the whole thing will be refactored probably
        messages::datapoints::dp_recovery_board::__message_meta_can_ids::BoardStatus,
        messages::datapoints::dp_sensor_carrier::__message_meta_can_ids::BoardStatus,
        messages::datapoints::dp_engine_control_board::__message_meta_can_ids::BoardStatus,
        messages::datapoints::dp_power_board::__message_meta_can_ids::BoardStatus,
        messages::datapoints::dp_fuel_control_board::__message_meta_can_ids::BoardStatus,
        messages::datapoints::dp_oxidizer_control_board::__message_meta_can_ids::BoardStatus,
        messages::datapoints::dp_rfs_computer::__message_meta_can_ids::Status,
    ] {
        if let Err(()) = registry.register(id, tx.clone()) {
            tracing::error!("kicking out other consumer");
        }
    }

    tokio::spawn(
        async move {
            let mut last_error_count_power_board = 0;
            let mut last_error_count_sensor_carrier = 0;
            while let Some(msg) = rx.recv().await {
                use datatypes::status::SensorStatus;

                // Depending on the board, there are different messages and conditions. Unify them here.
                let (healthy, output, name) = match &msg {
                    messages::Message::RecoveryBoard(dp_recovery_board::Message::BoardStatus(
                        board_status,
                    )) => {
                        let _ = recovery_armed.update(board_status.arming_state);
                        (
                            is_recovery_board_healthy(board_status),
                            &health.recovery_board,
                            "Recovery Board",
                        )
                    }
                    messages::Message::SensorCarrier(dp_sensor_carrier::Message::BoardStatus(
                        board_status,
                    )) => {
                        let sensors = &board_status.sensor_health;
                        // Only 1 imu and 1 gps are mission critical.
                        let healthy = (sensors.gps_1 == SensorStatus::Online
                            || sensors.gps_2 == SensorStatus::Online)
                            && (sensors.imu_1 == SensorStatus::Online
                                || sensors.imu_2 == SensorStatus::Online)
                            && board_status.common.errors == last_error_count_sensor_carrier;
                        last_error_count_sensor_carrier = board_status.common.errors;

                        (healthy, &health.sensor_carrier, "Sensor Carrier")
                    }
                    messages::Message::EngineControlBoard(
                        dp_engine_control_board::Message::BoardStatus(board_status),
                    ) => {
                        let _ = ecu_armed.update(board_status.armed);
                        let healthy = board_status.common.errors == 0;
                        (
                            healthy,
                            &health.engine_control_board,
                            "Engine Control Board",
                        )
                    }
                    messages::Message::PowerBoard(dp_power_board::Message::BoardStatus(
                        board_status,
                    )) => {
                        let healthy = board_status.errors == last_error_count_power_board;
                        last_error_count_power_board = board_status.errors;
                        (healthy, &health.power_board, "Power Board")
                    }
                    messages::Message::FuelControlBoard(
                        dp_fuel_control_board::Message::BoardStatus(board_status),
                    ) => {
                        let healthy = board_status.common.errors == 0
                            && board_status.thermocouple_status == SensorStatus::Online
                            && board_status.pressure_bus == SensorStatus::Online;
                        (healthy, &health.fuel_control_board, "Fuel Control Board")
                    }
                    messages::Message::OxidizerControlBoard(
                        dp_oxidizer_control_board::Message::BoardStatus(board_status),
                    ) => {
                        let healthy = board_status.common.errors == 0
                            && board_status.thermocouple_status == SensorStatus::Online
                            && board_status.pressure_bus == SensorStatus::Online;
                        (
                            healthy,
                            &health.oxidizer_control_board,
                            "Oxidizer Control Board",
                        )
                    }
                    messages::Message::RfsComputer(dp_rfs_computer::Message::Status(
                        board_status,
                    )) => {
                        let _ = rfs_status.update((board_status.clone(), clock.current()));
                        continue; // Don't store it in the board healths
                    }
                    _ => {
                        tracing::warn!("Received message it hasn't subscribed to");
                        continue;
                    }
                };
                if healthy {
                    // If healthy, report the current time to the input tree
                    let _ = output.update(Some(clock.current()));
                } else {
                    // Else, report the unhealthyness and log a warning
                    tracing::warn!(?msg, "{name} is unhealthy");
                    let _ = output.update(None);
                }
            }
            tracing::warn!("Shutting down");
        }
        .instrument(tracing::info_span!("drivers::BoardHealth")),
    )
    .abort_handle()
}

/// Checks the recovery board status for a simple healthy / unhealthy state.
/// Does not do any error reporting, will only return `false` if *something* is unhealthy.
fn is_recovery_board_healthy(
    board_status: &messages::datapoints::dp_recovery_board::RecoveryBoardStatus,
) -> bool {
    use messages::datapoints::dp_recovery_board::{ActuatorStatus, WatchdogState};

    board_status.common.errors == 0
        && matches!(
            board_status.steering_watchdog_status,
            WatchdogState::Disabled | WatchdogState::Active
        )
        && matches!(
            board_status.steering_general,
            ActuatorStatus::Connected | ActuatorStatus::PowerOn
        )
        // FIXME: for REC board V2 this should be enabled again, but only if steering has power.
        // && board_status.steering_left_connected
        // && board_status.steering_right_connected
        && (
            // Only one of the separation servos need to be connected, the other is at the CATS.
            matches!(
                board_status.sep1_status,
                ActuatorStatus::Connected | ActuatorStatus::PowerOn
            ) || matches!(
                board_status.sep2_status,
                ActuatorStatus::Connected | ActuatorStatus::PowerOn
            )
        )
        && (
            // Only one of the deployment servos need to be connected, the other is at the CATS.
            matches!(
                board_status.depl1_status,
                ActuatorStatus::Connected | ActuatorStatus::PowerOn
            ) || matches!(
                board_status.depl2_status,
                ActuatorStatus::Connected | ActuatorStatus::PowerOn
            )
        )
}

/// Listens for `SteeringActualPositions` messages.
pub fn spawn_and_register_actual_steering_driver(
    registry: &mut ConsumerRegistry,
    actual_pos: Watchable<SteeringMotorRevolutions>,
) -> AbortHandle {
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
    if let Err(()) = registry.register(
        messages::datapoints::dp_recovery_board::__message_meta_can_ids::SteeringActualPositions,
        tx,
    ) {
        tracing::error!("kicking out other consumer");
    }

    tokio::spawn(
        async move {
            while let Some(msg) = rx.recv().await {
                let messages::Message::RecoveryBoard(
                    dp_recovery_board::Message::SteeringActualPositions(pos),
                ) = msg
                else {
                    tracing::warn!("Received message it hasn't subscribed to");
                    continue;
                };

                let revs = SteeringMotorRevolutions {
                    left_motor_steps: pos.left_pos,
                    right_motor_steps: pos.right_pos,
                };
                let _old = actual_pos.update(revs);
            }
            tracing::warn!("Shutting down");
        }
        .instrument(tracing::info_span!("drivers::ActualSteeringPos")),
    )
    .abort_handle()
}
