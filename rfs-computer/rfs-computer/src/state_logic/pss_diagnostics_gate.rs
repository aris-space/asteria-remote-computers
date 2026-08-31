use datatypes::status::SensorStatus;
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::util::time::MissionTime;
use messages::datapoints::dp_engine_control_board::EngineControlBoardStatus;
use messages::datapoints::dp_flight_computer::FlightComputerStatus;
use messages::datapoints::dp_fuel_control_board::FuelControlBoardStatus;
use messages::datapoints::dp_oxidizer_control_board::OxidizerControlBoardStatus;
use std::time::Duration;
use watchable::Watchable;

use crate::io::input::{BoardsHealthyAsOf, StatusReport};

/// Checks whether a board's status report is fresh, recent enough, and healthy.
fn check_board<T: Clone + Default + BoardHealthy>(
    report: &Watchable<StatusReport<T>>,
    name: &str,
    now: MissionTime,
    outdated_threshold: Duration,
) -> bool {
    let report = report.read();
    if !report.fresh {
        tracing::warn!("Failing Diagnostics: no response yet from {name}");
        return false;
    }
    if now > report.timestamp + outdated_threshold {
        tracing::warn!("Failing Diagnostics: {name} hasn't responded lately");
        return false;
    }
    if !report.current_status.is_healthy() {
        tracing::warn!("Failing Diagnostics: Last response from {name} was unhealthy");
        return false;
    }
    true
}

/// This state checks the health of the pss boards in the rocket and distinguishes between three possibilities:
/// 1. One of the boards unhealthy => Log the problem and go to the `on_tests_fail` state.
/// 2. If the last report from the board is more than `outdated_threshold` ago => Log the problem and go to the
///    `on_tests_fail` state.
/// 3. Boards healthy => Go to the `on_tests_pass` state.
#[derive(Debug)]
pub struct PSSDiagnosticsGate {
    pub(crate) outdated_threshold: Duration,
    pub(crate) on_tests_pass: StateName,
    pub(crate) on_tests_fail: StateName,
}

#[async_trait::async_trait]
impl StateLogic for PSSDiagnosticsGate {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::PSSDiagnosticsGate::run", skip_all, fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!("Entered the PSS Diagnostics gate...");

        let BoardsHealthyAsOf {
            time: _,
            flight_computer,
            fuel_control_board,
            oxidizer_control_board,
            engine_control_board,
        } = &input.rocket_health;

        let now = input.time.get();
        let healths = [
            check_board(
                flight_computer,
                "flight computer",
                now,
                self.outdated_threshold,
            ),
            check_board(
                fuel_control_board,
                "fuel control board",
                now,
                self.outdated_threshold,
            ),
            check_board(
                oxidizer_control_board,
                "oxidizer control board",
                now,
                self.outdated_threshold,
            ),
            check_board(
                engine_control_board,
                "engine control board",
                now,
                self.outdated_threshold,
            ),
        ];
        if healths.iter().all(|&x| x) {
            self.on_tests_pass.clone()
        } else {
            self.on_tests_fail.clone()
        }
    }
}

/// Whether a board's status report indicates the board is currently healthy.
pub trait BoardHealthy {
    fn is_healthy(&self) -> bool;
}

/// The time at which a board was last observed to be healthy, if ever.
pub fn healthy_as_of<T: BoardHealthy>(report: &StatusReport<T>) -> Option<MissionTime> {
    (report.fresh && report.current_status.is_healthy()).then_some(report.timestamp)
}

impl BoardHealthy for FlightComputerStatus {
    fn is_healthy(&self) -> bool {
        self.common.errors == 0
    }
}

impl BoardHealthy for FuelControlBoardStatus {
    fn is_healthy(&self) -> bool {
        self.common.errors == 0
            && self.thermocouple_status == SensorStatus::Online
            && self.pressure_bus == SensorStatus::Online
    }
}

impl BoardHealthy for OxidizerControlBoardStatus {
    fn is_healthy(&self) -> bool {
        self.common.errors == 0
            && self.thermocouple_status == SensorStatus::Online
            && self.pressure_bus == SensorStatus::Online
    }
}

impl BoardHealthy for EngineControlBoardStatus {
    fn is_healthy(&self) -> bool {
        self.common.errors == 0 && self.thermocouple_status == SensorStatus::Online
    }
}
