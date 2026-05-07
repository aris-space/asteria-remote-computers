use datatypes::actuator::NormallyClosedValve;
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

/// This state sets all PSS valves to normal(*) position.
/// Then proceeds to the `on_done` state.
///
/// *: Normal state refers to the state where the mains are closed, vents are open, and the DPR are disabled.
/// Note that if the FC is in passive mode, the this has no effect.
///
/// This is used in various places to restore a known state.
#[derive(Debug)]
pub struct SetNormalState {
    pub on_done: StateName,
}

#[async_trait::async_trait]
impl StateLogic for SetNormalState {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::SetNormalState::run", skip_all, fields(?self, state = %param.current_state().0))]
    #[allow(clippy::default_trait_access)] // I specifically want this, since the restoring default behaviour is the main intention.
    async fn run(
        &self,
        _input: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!("Setting normal state of all valves");

        output.valves.set_oxidizer_vent(NormallyClosedValve::Open);
        output.valves.set_pressurization_vent(Default::default());
        output.valves.set_fuel_pressurization(Default::default());
        output.valves.set_fuel_vent(Default::default());
        output
            .valves
            .set_oxidizer_pressurization(Default::default());
        output.valves.set_fuel_main(Default::default());
        output.valves.set_oxidizer_main(Default::default());

        self.on_done.clone()
    }
}
