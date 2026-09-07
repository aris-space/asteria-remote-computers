use datatypes::actuator::DigitalOut;
use datatypes::sensor::DigitalIn;
use datatypes::status::ArmingState;
use lib_mission::reactive::{IntoWatched, WatchedExt as _};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;

/// This state enables the FC to start ignition if the FC and RFS are ready.
/// Once it happens, it transitions to `on_ignition`. Otherwise to `on_not_ready` or any manual transition.
///
/// Note that after this a [`super::DisarmIgniter`] state should occur soon, to reset the software arming state.
/// That is used to disallow the FC to control the igniter and stop it should it already have started ignition.
#[derive(Debug)]
pub struct WaitForIgnition {
    pub on_ignition: StateName,
    pub on_not_ready: StateName,
    pub manual_transitions: ManualTransitions,
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
        _output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        let fc_status = input.rocket_health.flight_computer.get();
        if fc_status.fresh
            && matches!(
                fc_status.current_status.is_software_armed,
                ArmingState::Safe
            )
        {
            tracing::warn!("FC isn't armed, preventing arming state");
            return self.on_not_ready.clone();
        }
        if matches!(input.uremote_sensors.igniter_armed.get(), DigitalIn::Off) {
            tracing::warn!("Igniter circuit isn't armed, preventing arming state");
            return self.on_not_ready.clone();
        }

        tracing::info!("Arming the igniter");
        param
            .parameters
            .rfs_computer_status
            .write()
            .is_ignition_armed = ArmingState::Armed;

        let mut spark_plug_watcher = input.uremote_actuators.igniter_spark.watched();
        let mut fc_watcher = input.rocket_health.flight_computer.watched();
        let mut ign_watcher = input.uremote_sensors.igniter_armed.watched();

        tokio::select! {
            _ = spark_plug_watcher.next_satisfying(|s| matches!(s, DigitalOut::On)) => {
                tracing::info!("Spark plug turned on, continueing");
                self.on_ignition.clone()
            },
            _ = fc_watcher.next_satisfying(|s| matches!(s.current_status.is_software_armed, ArmingState::Safe)) => {
                tracing::info!("FC got disarmed, disarming");
                self.on_not_ready.clone()
            },
            _ = ign_watcher.next_satisfying(|s| matches!(s, DigitalIn::Off)) => {
                tracing::info!("Spark plug got disarmed, disarming");
                self.on_not_ready.clone()
            },
            next_state = self.manual_transitions.next(input, param) => next_state,
        }
    }
}
