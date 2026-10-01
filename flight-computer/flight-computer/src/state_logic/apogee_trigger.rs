use lib_mission::reactive::{IntoWatched, WatchedExt};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;
use std::time::Duration;

use crate::io::types::SeparationState;

/// This state detects apogee in three cases and transitions in `on_apogee_reached`:
/// 1. When the down velocity gets positive (rocket starts descending).
/// 2. When the time to apogee is shorter than `trigger_before_apogee_time`. The time is predicted
///    with a simple model only taking gravity into account.
/// 3. The CATS backup flight computer detects apogee first and signals separation to the fc,
///    in which case we transition to `on_cats_separation`.
#[derive(Debug)]
pub struct ApogeeTrigger {
    pub(crate) trigger_before_apogee_time: Duration,
    pub(crate) on_apogee_reached: StateName,
    pub(crate) on_cats_separation: StateName,
    pub(crate) manual_transitions: ManualTransitions,
}

#[async_trait::async_trait]
impl StateLogic for ApogeeTrigger {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::ApogeeTrigger::run", skip_all, fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        let mut velocity_watcher = input.velocity.watched();
        let mut cats_watcher = input.cats_separation.watched();
        // The velocity is positive in down (NED) direction. Therefore, apogee gets triggered the latest if the down velocity gets positive.
        // The main trigger is based on a prediction of the time before apogee. Based on a simple model, not taking drag into account the velocity
        // is divided by gravity to predict time to apogee. If this time goes below `trigger_before_apogee_time` apogee is triggered.
        // This leads to the logical expression (v.down > 0.0 || v.down.abs() / 9.81 < trigger_threshold) that can be simplified to
        // (v.down > -self.trigger_before_apogee_time.as_secs_f32() * 9.81) for precomputing some constants.
        // TODO: Could probably be improved with taking the a drag coefficient or even the whole drag curve from simulations into account.
        let simplified_trigger_threshold = -self.trigger_before_apogee_time.as_secs_f32() * 9.81;

        tokio::select! {
            next = self.manual_transitions.next(input, _p) => {
                tracing::info!(cause = "manual", "transitioning");
                return next;
            },
            _ = velocity_watcher.next_satisfying(|v| v.velocity_down > simplified_trigger_threshold) => {
                tracing::info!(cause = "fc apogee trigger", "transitioning");
                return self.on_apogee_reached.clone();
            },
            _ = cats_watcher.next_satisfying(|s| matches!(s, SeparationState::Separated)) => {
                tracing::info!(cause = "cats separation", "transitioning");
                return self.on_cats_separation.clone();
            },
        }
    }
}
