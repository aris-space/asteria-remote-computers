use lib_mission::reactive::{IntoWatched, WatchedExt};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;

/// This state waits until the firing completes or aborts or if something goes wrong, an apogee is reached.
/// When the down velocity gets positive (rocket starts descending more than `abort_downward_velocity_m_s` m/s down).
#[derive(Debug)]
pub struct Thrust {
    pub(crate) abort_downward_velocity_m_s: f32,
    pub(crate) on_apogee_reached: StateName,
    #[allow(dead_code)] // todo
    pub(crate) on_firing_finished: StateName,
    pub(crate) manual_transitions: ManualTransitions,
}

#[async_trait::async_trait]
impl StateLogic for Thrust {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::Thrust::run", skip_all, fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        let mut velocity_watcher = input.velocity.watched();

        tokio::select! {
            next = self.manual_transitions.next(input, _p) => {
                tracing::info!(cause = "manual", "transitioning");
                return next;
            },
            _ = velocity_watcher.next_satisfying(|v| v.velocity_down > self.abort_downward_velocity_m_s) => {
                tracing::info!(cause = "fc apogee trigger", "transitioning");
                return self.on_apogee_reached.clone();
            },
            // TODO: monitor firing sequence and move to on_firing_finished if done
        }
    }
}
