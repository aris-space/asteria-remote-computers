use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::util::time::MissionTime;
use std::time::Duration;
use watchable::Watchable;

use crate::io::input::BoardsHealthyAsOf;

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
        let check = |health: &Watchable<Option<MissionTime>>, name: &str| {
            if let Some(t) = health.get() {
                if input.time.get() > t + self.outdated_threshold {
                    tracing::warn!("Failing Diagnostics: {name} hasn't responded lately");
                    false
                } else {
                    true
                }
            } else {
                tracing::warn!("Failing Diagnostics: Last response from {name} was unhealthy");
                false
            }
        };

        tracing::info!("Entered the PSS Diagnostics gate...");

        let BoardsHealthyAsOf {
            flight_computer,
            fuel_control_board,
            oxidizer_control_board,
            engine_control_board,
        } = &input.rocket_health;

        let healths = [
            check(flight_computer, "flight computer"),
            check(fuel_control_board, "fuel control board"),
            check(oxidizer_control_board, "oxidizer control board"),
            check(engine_control_board, "engine control board"),
        ];
        if healths.iter().all(|&x| x) {
            self.on_tests_pass.clone()
        } else {
            self.on_tests_fail.clone()
        }
    }
}
