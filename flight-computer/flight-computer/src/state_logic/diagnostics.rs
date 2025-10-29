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

        tracing::info!("Entered the Diagnostics state...");

        let healths = [
            check(&input.boards_health.sensor_carrier, "sensor carrier"),
            check(&input.boards_health.power_board, "power board"),
            check(&input.boards_health.recovery_board, "recovery board"),
            self.check_pss_boards
                && check(
                    &input.boards_health.fuel_control_board,
                    "fuel control board",
                ),
            self.check_pss_boards
                && check(
                    &input.boards_health.oxidizer_control_board,
                    "oxidizer control board",
                ),
            self.check_pss_boards
                && check(
                    &input.boards_health.engine_control_board,
                    "engine control board",
                ),
        ];
        if healths.iter().all(|&x| x) {
            self.on_tests_pass.clone()
        } else {
            self.on_tests_fail.clone()
        }
    }
}
