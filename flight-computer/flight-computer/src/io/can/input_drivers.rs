use hermes_can::messages;
use hermes_can::messages::board_status::{ArmingState, RfsComputerStatus};
use hermes_can::messages::sensor_data::{EnvironmentalData, RailStatus24V, RailStatus5V};
use hermes_can::CanMessage as _;
use lib_mission::can::ConsumerRegistry;
use lib_mission::reactive::Watched;
use lib_mission::util::time::MissionTime;
use tokio::task::AbortHandle;
use tracing::Instrument as _;
use watchable::Watchable;

use crate::io::input::BoardsHealthyAsOf;
use crate::io::types::{Acceleration, Location, Orientation, SteeringMotorRevolutions, Velocity};

/// Listens for [`messages::board_status`] messages and updates [`BoardsHealthyAsOf`].
///
/// Additionally stores the full RFS computer status and arming states.
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
        messages::board_status::RecoveryBoardStatus::ID,
        messages::board_status::SensorCarrierStatus::ID,
        messages::board_status::EngineControlBoardStatus::ID,
        messages::board_status::PowerBoardStatus::ID,
        messages::board_status::FuelControlBoardStatus::ID,
        messages::board_status::OxidizerControlBoardStatus::ID,
        messages::board_status::RfsComputerStatus::ID,
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
                use messages::board_status::SensorStatus;

                // Depending on the board, there are different messages and conditions. Unify them here.
                let (healthy, output, name) = match &msg {
                    messages::Message::RecoveryBoardStatus(board_status) => {
                        let _ = recovery_armed.update(board_status.arming_state.clone());
                        (
                            is_recovery_board_healthy(board_status),
                            &health.recovery_board,
                            "Recovery Board",
                        )
                    }
                    messages::Message::SensorCarrierStatus(board_status) => {
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
                    messages::Message::EngineControlBoardStatus(board_status) => {
                        let _ = ecu_armed.update(board_status.armed.clone());
                        let healthy = board_status.common.errors == 0;
                        (
                            healthy,
                            &health.engine_control_board,
                            "Engine Control Board",
                        )
                    }
                    messages::Message::PowerBoardStatus(board_status) => {
                        let healthy = board_status.common.errors == last_error_count_power_board;
                        last_error_count_power_board = board_status.common.errors;
                        (healthy, &health.power_board, "Power Board")
                    }
                    messages::Message::FuelControlBoardStatus(board_status) => {
                        let healthy = board_status.common.errors == 0
                            && board_status.thermocouple_status == SensorStatus::Online
                            && board_status.pressure_bus == SensorStatus::Online;
                        (healthy, &health.fuel_control_board, "Fuel Control Board")
                    }
                    messages::Message::OxidizerControlBoardStatus(board_status) => {
                        let healthy = board_status.common.errors == 0
                            && board_status.thermocouple_status == SensorStatus::Online
                            && board_status.pressure_bus == SensorStatus::Online;
                        (
                            healthy,
                            &health.oxidizer_control_board,
                            "Oxidizer Control Board",
                        )
                    }
                    messages::Message::RfsComputerStatus(board_status) => {
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
fn is_recovery_board_healthy(board_status: &messages::board_status::RecoveryBoardStatus) -> bool {
    use messages::board_status::{ActuatorStatus, WatchdogState};

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

/// Listens for sensor carrier sensor data.
pub fn spawn_and_register_kinematics_driver(
    registry: &mut ConsumerRegistry,
    location: Watchable<Location>,
    velocity: Watchable<Velocity>,
    acceleration: Watchable<Acceleration>,
    orientation: Watchable<Orientation>,
) -> AbortHandle {
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
    if let Err(()) = registry.register(messages::sensor_data::PositionData::ID, tx.clone()) {
        tracing::error!("kicking out other consumer");
    }
    if let Err(()) = registry.register(messages::sensor_data::VelocityData::ID, tx.clone()) {
        tracing::error!("kicking out other consumer");
    }
    if let Err(()) = registry.register(messages::sensor_data::ImuData::ID, tx.clone()) {
        tracing::error!("kicking out other consumer");
    }
    if let Err(()) = registry.register(messages::sensor_data::OrientationData::ID, tx) {
        tracing::error!("kicking out other consumer");
    }

    let task = async move {
        while let Some(msg) = rx.recv().await {
            match msg {
                messages::Message::PositionData(pos) => {
                    // NED is native for position
                    let _ = location.update(Location {
                        latitude: pos.location_latitude,
                        longitude: pos.location_longitude,
                        altitude: pos.location_hamsl,
                        horizontal_accuracy: pos.horizontal_accuracy,
                        vertical_accuracy: pos.vertical_accuracy,
                    });
                }
                messages::Message::VelocityData(vel) => {
                    // NED is native for velocity, but store both
                    let _ = velocity.update(Velocity {
                        north: vel.velocity_north,
                        east: vel.velocity_east,
                        down: vel.velocity_down,
                        x: vel.velocity_x,
                        y: vel.velocity_y,
                        z: vel.velocity_z,
                    });
                }
                messages::Message::ImuData(imu) => {
                    // This is using potentially off-yaw data, but also the native ones
                    let _ = acceleration.update(Acceleration {
                        north: imu.acceleration_north,
                        east: imu.acceleration_east,
                        down: imu.acceleration_down,
                        x: imu.acceleration_x,
                        y: imu.acceleration_y,
                        z: imu.acceleration_z,
                    });
                }
                messages::Message::OrientationData(orient) => {
                    // This is using potentially off-yaw data
                    let _ = orientation.update(orient);
                }
                // TODO: Angular velocity
                _ => tracing::warn!("Received message it hasn't subscribed to"),
            }
        }
        tracing::warn!("Shutting down");
    }
    .instrument(tracing::info_span!("drivers::SensorData"));
    tokio::spawn(task).abort_handle()
}

/// Listens for sensor carrier sensor raw data that we won't process.
pub fn spawn_and_register_raw_unprocessed_data_driver(
    registry: &mut ConsumerRegistry,
    environmental_data: Watchable<EnvironmentalData>,
    rail_status_24v: Watchable<RailStatus24V>,
    rail_status_5v: Watchable<RailStatus5V>,
) -> AbortHandle {
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
    if let Err(()) = registry.register(messages::sensor_data::EnvironmentalData::ID, tx.clone()) {
        tracing::error!("kicking out other consumer");
    }
    if let Err(()) = registry.register(messages::sensor_data::RailStatus24V::ID, tx.clone()) {
        tracing::error!("kicking out other consumer");
    }
    if let Err(()) = registry.register(messages::sensor_data::RailStatus5V::ID, tx.clone()) {
        tracing::error!("kicking out other consumer");
    }

    let task = async move {
        while let Some(msg) = rx.recv().await {
            match msg {
                messages::Message::EnvironmentalData(environment) => {
                    let _ = environmental_data.update(environment);
                }
                messages::Message::RailStatus24V(rail_s_24v) => {
                    let _ = rail_status_24v.update(rail_s_24v);
                }
                messages::Message::RailStatus5V(rail_s_5v) => {
                    let _ = rail_status_5v.update(rail_s_5v);
                }
                _ => tracing::warn!("Received message it hasn't subscribed to"),
            }
        }
        tracing::warn!("Shutting down");
    }
    .instrument(tracing::info_span!("drivers::SensorRawUnprocessedData"));
    tokio::spawn(task).abort_handle()
}

/// Listens for [`messages::event_messages::SteeringActualPositions`].
pub fn spawn_and_register_actual_steering_driver(
    registry: &mut ConsumerRegistry,
    actual_pos: Watchable<SteeringMotorRevolutions>,
) -> AbortHandle {
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
    if let Err(()) = registry.register(messages::event_messages::SteeringActualPositions::ID, tx) {
        tracing::error!("kicking out other consumer");
    }

    tokio::spawn(
        async move {
            while let Some(msg) = rx.recv().await {
                let messages::Message::SteeringActualPositions(pos) = msg else {
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
