use lib_mission::reactive::IntoWatched;
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use std::time::Duration;

use crate::io::types::SeparationState;
use crate::util::timer::Timer;

/// This state separates the nosecone, which pulls out the drogue, then waits
/// for `deploy_time` time before transiting into the `on_after_separation` state.
/// With `deploy_time` equals to zero, it immediately transitions.
#[derive(Debug)]
pub struct Separate {
    pub(crate) deploy_time: Duration,
    pub(crate) on_after_separation: StateName,
}

#[async_trait::async_trait]
impl StateLogic for Separate {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::Separation::run", skip_all, fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        output.separate_drogue.replace(SeparationState::Separated);

        let mut timer = Timer::new(input.time.watched(), self.deploy_time);
        timer.wait().await;
        self.on_after_separation.clone()
    }
}
