use crate::util::timer::Timer;
use lib_mission::reactive::{IntoWatched, WatchedExt};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;
use std::time::Duration;

/// This state transitions in three cases:
/// 1. When the altitude above sea level in meters is less
///    or equal to `height_limit`, we transition to `on_height_limit_reached`
/// 2. When the time since the transition into this state is larger
///    than `time_limit`, we transition to `on_time_limit_reached`
/// 3. It has received a manual command
#[derive(Debug)]
pub struct HeightOrTimeTrigger {
    pub(crate) height_limit: f32,
    pub(crate) time_limit: Duration,
    pub(crate) on_time_limit_reached: StateName,
    pub(crate) on_height_limit_reached: StateName,
    pub(crate) manual_transitions: ManualTransitions,
}

#[async_trait::async_trait]
impl StateLogic for HeightOrTimeTrigger {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::HeightOrTimeTrigger::run", skip_all, fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        let mut timer = Timer::new(input.time.watched(), self.time_limit);
        let mut height_watcher = input.location.watched();

        tokio::select! {
            next = self.manual_transitions.next(input, _p) => {
                tracing::info!(cause = "manual", "transitioning");
                return next;
            },
            _ = height_watcher.next_satisfying(|h| h.location_hamsl <= self.height_limit) => {
                tracing::info!(cause = "height trigger", "transitioning");
                return self.on_height_limit_reached.clone();
            },
            _ = timer.wait() => {
                tracing::info!(cause = "time trigger", "transitioning");
                return self.on_time_limit_reached.clone();
            },
        }
    }
}
