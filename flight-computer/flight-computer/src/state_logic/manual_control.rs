use std::sync::Arc;

use lib_mission::actuator_control::command::{ActuatorControl, NeverReturns};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;
use pss_drivers::output::ValveName;

/// This state gives manual control for the configured actuators,
/// and otherwise waits for a manual transition.
///
/// This also enters active mode for the FC.
#[derive(Debug)]
pub struct ManualControl {
    pub(crate) manual_transitions: ManualTransitions,
    pub(crate) actuator_control: Arc<ActuatorControl<ValveName>>,
}

#[async_trait::async_trait]
impl StateLogic for ManualControl {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::ManualControl::run", skip_all, fields(state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!("Entering a manual control state");

        // Prepares a background task to listen for (and act upon) manual actuator inputs.
        let actuator_future = self.actuator_control.act(
            input.state.current_state.get().0 as usize,
            &input.actuator_command,
            output.valves.ok(),
        );
        assert_future::<NeverReturns>(&actuator_future);

        // Then waits for a manual transition. This will drop the actuator future and cancels its functionality
        tokio::select! {
            next_state = self.manual_transitions.next(input, param) => next_state,
            _ = actuator_future => unreachable!("act cannot return normally"),
        }
    }
}

/// Just a helper function to ensure the futures we're using have the right type
fn assert_future<T>(_future: &impl std::future::Future<Output = T>) {}
