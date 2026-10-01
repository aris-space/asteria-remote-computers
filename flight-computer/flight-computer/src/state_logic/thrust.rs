use crate::io::types::SeparationState;
use lib_mission::reactive::{IntoWatched, WatchedExt};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;

/// This state waits until the firing is finished, which is detected by the N2 pressure dropping to or below
/// `n2_pressure_limit_barg`, until an apogee is reached, or until CATS signals separation (`on_cats_separation`).
/// Apogee is detected when the rocket starts descending more than `abort_downward_velocity_m_s` m/s down.
#[derive(Debug)]
pub struct Thrust {
    pub(crate) abort_downward_velocity_m_s: f32,
    pub(crate) on_apogee_reached: StateName,
    pub(crate) on_firing_finished: StateName,
    pub(crate) n2_pressure_limit_barg: f32,
    pub(crate) on_cats_separation: StateName,
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
        let mut cats_watcher = input.cats_separation.watched();
        let mut n2_pressure_watcher = input.pss_sensors.pressurization_line_pressure.watched();

        tokio::select! {
            next = self.manual_transitions.next(input, _p) => {
                tracing::info!(cause = "manual", "transitioning");
                return next;
            },
            _ = velocity_watcher.next_satisfying(|v| v.velocity_down > self.abort_downward_velocity_m_s) => {
                tracing::info!(cause = "fc apogee trigger", "transitioning");
                return self.on_apogee_reached.clone();
            },
            _ = cats_watcher.next_satisfying(|s| matches!(s, SeparationState::Separated)) => {
                tracing::info!(cause = "cats separation", "transitioning");
                return self.on_cats_separation.clone();
            },
            _ = n2_pressure_watcher.next_satisfying(|p| p.0 <= self.n2_pressure_limit_barg) => {
                tracing::info!(cause = "n2 depleted", "transitioning");
                return self.on_firing_finished.clone();
            },
        }
    }
}
