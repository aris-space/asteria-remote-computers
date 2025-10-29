use crate::io::output::ActuatorName;
use lib_mission::actuator_control::command::{ActuatorControl, NeverReturns};
use lib_mission::actuator_control::valves::NormallyOpenValve;
use lib_mission::reactive::{IntoWatched, WatchedExt as _};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;
use uremote_electronics::types::SparkPlug;

/// This state enable the ECU to start ignition if the FC and RFS are ready.
/// Once it happens, it transitions to `on_ignition`. Otherwise to `on_not_ready` or any manual transition.
/// Note that after this a [`super::DisarmIgniter`] state should occur soon, to reset the software arming state.
///
/// This is used to disallow the ECU to control to the igniter and stop it should it already has started ignition.
/// While waiting for ignition this state also allows manual control. This is used to start purging manually before ignition.
#[derive(Debug)]
pub struct WaitForIgnition {
    pub on_ignition: StateName,
    pub on_not_ready: StateName,
    pub manual_transitions: ManualTransitions,
    pub actuator_control: ActuatorControl<ActuatorName>,
}

#[async_trait::async_trait]
impl StateLogic for WaitForIgnition {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::WaitForIgnition::run", skip_all, fields(?self, state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        if matches!(
            input.flight_computer_status.get().is_software_armed,
            hermes_can::messages::board_status::ArmingState::Safe
        ) {
            tracing::warn!("FC isn't armed, preventing arming state");
            return self.on_not_ready.clone();
        }
        if !input.uremote_sensors.igniter_armed.get() {
            tracing::warn!("Igniter circuit isn't armed, preventing arming state");
            return self.on_not_ready.clone();
        }

        tracing::info!("Arming the igniter");
        output.is_software_armed.replace(true);
        output
            .rfs_actuators
            .h2_bleed_valve
            .replace(NormallyOpenValve::Closed);
        output
            .rfs_actuators
            .gox_bleed_valve
            .replace(NormallyOpenValve::Closed);

        // Prepares a background task to listen for (and act upon) manual actuator inputs.
        let actuator_future = self.actuator_control.act(
            param.current_state().1,
            &input.actuator_command,
            output.as_settables(),
        );
        assert_future::<NeverReturns>(&actuator_future);

        let mut spark_plug_watcher = input.uremote_actuators.igniter_spark.watched();
        let mut fc_watcher = input.flight_computer_status.watched();
        let mut ign_watcher = input.uremote_sensors.igniter_armed.watched();

        tokio::select! {
            _ = spark_plug_watcher.next_satisfying(|s| matches!(s, SparkPlug::On)) => {
                tracing::info!("Spark plug turned on, continueing");
                self.on_ignition.clone()
            },
            _ = fc_watcher.next_satisfying(|s| matches!(s.is_software_armed, hermes_can::messages::board_status::ArmingState::Safe)) => {
                tracing::info!("FC got disarmed, disarming");
                self.on_not_ready.clone()
            },
            _ = ign_watcher.next_satisfying(|s| !s) => {
                tracing::info!("Spark plug got disarmed, disarming");
                self.on_not_ready.clone()
            },
            next_state = self.manual_transitions.next(input, param) => next_state,
            _ = actuator_future => unreachable!("act cannot return normally"),
        }
    }
}

/// Just a helper function to ensure the futures we're using have the right type
fn assert_future<T>(_future: &impl Future<Output = T>) {}
