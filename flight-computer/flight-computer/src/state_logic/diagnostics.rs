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
    /// If neither this nor `check_main_chute` is set, the recovery board is not checked at all.
    pub check_separation: bool,
    /// If `true`, include checks for recovery deployment and steering.
    /// If neither this nor `check_separation` is set, the recovery board is not checked at all.
    pub check_main_chute: bool,
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
            !(self.check_separation || self.check_main_chute)
                || check_board(
                    &input.boards_status.recovery_board,
                    "recovery board",
                    now,
                    self.outdated_threshold,
                    |current, prev| {
                        is_recovery_board_healthy(
                            current,
                            prev,
                            self.check_separation,
                            self.check_main_chute,
                        )
                    },
                ),
            !(self.check_pss_boards)
                || check_board(
                    &input.boards_status.fuel_control_board,
                    "fuel control board",
                    now,
                    self.outdated_threshold,
                    is_fuel_control_board_healthy,
                ),
            !(self.check_pss_boards)
                || check_board(
                    &input.boards_status.oxidizer_control_board,
                    "oxidizer control board",
                    now,
                    self.outdated_threshold,
                    is_oxidizer_control_board_healthy,
                ),
            !(self.check_pss_boards)
                || check_board(
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
/// `check_separation` and `check_main_chute` control whether the separation / deployment
/// servos are included in the check.
fn is_recovery_board_healthy(
    current: &messages::datapoints::dp_recovery_board::RecoveryBoardStatus,
    _prev: &messages::datapoints::dp_recovery_board::RecoveryBoardStatus,
    check_separation: bool,
    check_main_chute: bool,
) -> bool {
    use messages::datapoints::dp_recovery_board::{ActuatorStatus, WatchdogState};

    current.common.errors == 0
        && (
            !check_main_chute
                || (matches!(
                    current.steering_watchdog_status,
                    WatchdogState::Disabled | WatchdogState::Active
                ) && matches!(
                    current.steering_general,
                    ActuatorStatus::Connected | ActuatorStatus::PowerOn
                )
                // FIXME: for REC board V2 this should be enabled again, but only if steering has power.
                // && current.steering_left_connected
                // && current.steering_right_connected
                )
        )
        && (
            !check_separation
                || (
                    // Only one of the separation servos need to be connected, the other is at the CATS.
                    matches!(
                        current.sep1_status,
                        ActuatorStatus::Connected | ActuatorStatus::PowerOn
                    ) || matches!(
                        current.sep2_status,
                        ActuatorStatus::Connected | ActuatorStatus::PowerOn
                    )
                )
        )
        && (
            !check_main_chute
                || (
                    // Only one of the deployment servos need to be connected, the other is at the CATS.
                    matches!(
                        current.depl1_status,
                        ActuatorStatus::Connected | ActuatorStatus::PowerOn
                    ) || matches!(
                        current.depl2_status,
                        ActuatorStatus::Connected | ActuatorStatus::PowerOn
                    )
                )
        )
}
