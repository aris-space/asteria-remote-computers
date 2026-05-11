use crate::io::input::StatusReport;
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::util::time::MissionTime;
use std::time::Duration;
use watchable::Watchable;

/// This state checks the health of the boards and distinguishes between three possibilities:
/// 1. One of the boards unhealthy => Log the problem and go to the `on_tests_fail` state.
/// 2. If the last report from the board is more than `outdated_threshold` ago => Log the problem and go to the
///    `on_tests_fail` state.
/// 3. Boards healthy => Go to the `on_tests_pass` state.
#[derive(Debug)]
pub struct Diagnostics {
    pub(crate) outdated_threshold: Duration,
    /// If `true`, also checks the PSS boards, as is usual during a flight.
    /// Else, only the AV stack boards and recovery board are checked.
    pub check_pss_boards: bool,
    /// If `true`, include checks for the separation, as is usual during a flight.
    /// Else, only the only the deployment and steering are checked in the recovery board
    pub check_separation: bool,
    pub(crate) on_tests_pass: StateName,
    pub(crate) on_tests_fail: StateName,
}

#[async_trait::async_trait]
impl StateLogic for Diagnostics {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::Diagnostics::run", skip_all, fields(self.check_pss_boards, state = %_p.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!("Entered the Diagnostics state...");

        let now = input.time.get();

        let healths = [
            check_board(
                &input.boards_status.sensor_carrier,
                "sensor carrier",
                now,
                self.outdated_threshold,
                is_sensor_carrier_healthy,
            ),
            check_board(
                &input.boards_status.power_board,
                "power board",
                now,
                self.outdated_threshold,
                is_power_board_healthy,
            ),
            check_board(
                &input.boards_status.recovery_board,
                "recovery board",
                now,
                self.outdated_threshold,
                if self.check_separation {
                    is_recovery_board_healthy
                } else {
                    is_recovery_board_healthy_no_sep
                },
            ),
            self.check_pss_boards
                && check_board(
                    &input.boards_status.fuel_control_board,
                    "fuel control board",
                    now,
                    self.outdated_threshold,
                    is_fuel_control_board_healthy,
                ),
            self.check_pss_boards
                && check_board(
                    &input.boards_status.oxidizer_control_board,
                    "oxidizer control board",
                    now,
                    self.outdated_threshold,
                    is_oxidizer_control_board_healthy,
                ),
            self.check_pss_boards
                && check_board(
                    &input.boards_status.engine_control_board,
                    "engine control board",
                    now,
                    self.outdated_threshold,
                    is_engine_control_board_healthy,
                ),
        ];

        if healths.iter().all(|&x| x) {
            self.on_tests_pass.clone()
        } else {
            self.on_tests_fail.clone()
        }
    }
}

fn check_board<T: Clone + Default>(
    report: &Watchable<StatusReport<T>>,
    name: &str,
    now: MissionTime,
    outdated_threshold: Duration,
    is_healthy: impl Fn(&T, &T) -> bool,
) -> bool {
    let report = report.read();
    if !report.fresh {
        tracing::warn!("Failing Diagnostics: {name} hasn't sent a report yet");
        return false;
    }
    if now > report.timestamp + outdated_threshold {
        tracing::warn!("Failing Diagnostics: {name} hasn't responded lately");
        return false;
    }
    if !is_healthy(&report.current_status, &report.previous_status) {
        tracing::warn!("Failing Diagnostics: Last response from {name} was unhealthy");
        return false;
    }
    true
}

fn is_sensor_carrier_healthy(
    current: &messages::datapoints::dp_sensor_carrier::SensorCarrierStatus,
    prev: &messages::datapoints::dp_sensor_carrier::SensorCarrierStatus,
) -> bool {
    use datatypes::status::SensorStatus;
    let sensors = &current.sensor_health;
    (sensors.gps_1 == SensorStatus::Online || sensors.gps_2 == SensorStatus::Online)
        && (sensors.imu_1 == SensorStatus::Online || sensors.imu_2 == SensorStatus::Online)
        && current.common.errors == prev.common.errors
}

fn is_engine_control_board_healthy(
    current: &messages::datapoints::dp_engine_control_board::EngineControlBoardStatus,
    _prev: &messages::datapoints::dp_engine_control_board::EngineControlBoardStatus,
) -> bool {
    current.common.errors == 0
}

fn is_power_board_healthy(
    current: &datatypes::status::StatusCommonMessage,
    prev: &datatypes::status::StatusCommonMessage,
) -> bool {
    current.errors == prev.errors
}

fn is_fuel_control_board_healthy(
    current: &messages::datapoints::dp_fuel_control_board::FuelControlBoardStatus,
    _prev: &messages::datapoints::dp_fuel_control_board::FuelControlBoardStatus,
) -> bool {
    use datatypes::status::SensorStatus;
    current.common.errors == 0
        && current.thermocouple_status == SensorStatus::Online
        && current.pressure_bus == SensorStatus::Online
}

fn is_oxidizer_control_board_healthy(
    current: &messages::datapoints::dp_oxidizer_control_board::OxidizerControlBoardStatus,
    _prev: &messages::datapoints::dp_oxidizer_control_board::OxidizerControlBoardStatus,
) -> bool {
    use datatypes::status::SensorStatus;
    current.common.errors == 0
        && current.thermocouple_status == SensorStatus::Online
        && current.pressure_bus == SensorStatus::Online
}

/// Checks the recovery board status for a simple healthy / unhealthy state.
/// Does not do any error reporting, will only return `false` if *something* is unhealthy.
fn is_recovery_board_healthy(
    current: &messages::datapoints::dp_recovery_board::RecoveryBoardStatus,
    _prev: &messages::datapoints::dp_recovery_board::RecoveryBoardStatus,
) -> bool {
    use messages::datapoints::dp_recovery_board::{ActuatorStatus, WatchdogState};

    current.common.errors == 0
        && matches!(
            current.steering_watchdog_status,
            WatchdogState::Disabled | WatchdogState::Active
        )
        && matches!(
            current.steering_general,
            ActuatorStatus::Connected | ActuatorStatus::PowerOn
        )
        // FIXME: for REC board V2 this should be enabled again, but only if steering has power.
        // && current.steering_left_connected
        // && current.steering_right_connected
        && (
            // Only one of the separation servos need to be connected, the other is at the CATS.
            matches!(
                current.sep1_status,
                ActuatorStatus::Connected | ActuatorStatus::PowerOn
            ) || matches!(
                current.sep2_status,
                ActuatorStatus::Connected | ActuatorStatus::PowerOn
            )
        )
        && (
            // Only one of the deployment servos need to be connected, the other is at the CATS.
            matches!(
                current.depl1_status,
                ActuatorStatus::Connected | ActuatorStatus::PowerOn
            ) || matches!(
                current.depl2_status,
                ActuatorStatus::Connected | ActuatorStatus::PowerOn
            )
        )
}

/// Checks the recovery board status for a simple healthy / unhealthy state.
/// Skips checks for separation
fn is_recovery_board_healthy_no_sep(
    current: &messages::datapoints::dp_recovery_board::RecoveryBoardStatus,
    _prev: &messages::datapoints::dp_recovery_board::RecoveryBoardStatus,
) -> bool {
    use messages::datapoints::dp_recovery_board::{ActuatorStatus, WatchdogState};

    current.common.errors == 0
        && matches!(
            current.steering_watchdog_status,
            WatchdogState::Disabled | WatchdogState::Active
        )
        && matches!(
            current.steering_general,
            ActuatorStatus::Connected | ActuatorStatus::PowerOn
        )
        && (
            // Only one of the deployment servos need to be connected, the other is at the CATS.
            matches!(
                current.depl1_status,
                ActuatorStatus::Connected | ActuatorStatus::PowerOn
            ) || matches!(
                current.depl2_status,
                ActuatorStatus::Connected | ActuatorStatus::PowerOn
            )
        )
}
