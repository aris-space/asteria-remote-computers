use lib_mission::reactive::IntoWatched;
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use messages::datapoints::dp_recovery_board::DeploymentState;
use std::time::Duration;

use crate::util::timer::Timer;

/// This state deploys the chute and then waits for `deploy_time` time
/// before transiting into the `on_after_deployment` state
#[derive(Debug)]
pub struct MainChuteDeployment {
    pub(crate) deploy_time: Duration,
    pub(crate) on_after_deployment: StateName,
}

#[async_trait::async_trait]
impl StateLogic for MainChuteDeployment {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::MainChuteDeployment::run", skip_all, fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        output.deploy_main_chute.replace(DeploymentState::Deployed);

        let mut timer = Timer::new(input.time.watched(), self.deploy_time);
        timer.wait().await;
        self.on_after_deployment.clone()
    }
}
