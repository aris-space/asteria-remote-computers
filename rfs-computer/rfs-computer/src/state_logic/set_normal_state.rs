use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

/// This state sets all RFS (and PSS if enabled) valves to normal(*) position.
/// Then proceeds to the `on_done` state.
///
/// (*) Normal state affects only valves, the decouplers are decouplers deliberatly skipped.
/// Additionally, the throttable valves are closed (which they wouldn't do on power loss, but it is their initial state).
///
/// This is used in various places to restore a known state.
#[derive(Debug)]
pub struct SetNormalState {
    pub on_done: StateName,
    pub include_pss: bool,
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

        let uremote_electronics::output::Actuators {
            maxon_board_power: _, // not affected
            custom1: _,           // not affected
            custom2: _,           // not affected
            custom3: _,           // not affected
            gox_purge_valve,
            h2_purge_valve,
            n2_bleed_valve,
            lox_purge_valve,
            gox_bleed_valve,
            n2_purge_valve,
            n2_fill_valve,
            h2_bleed_valve,
            lox_bleed_valve,
            booster_valve,
            h2_igniter_valve,
            gox_igniter_valve,
            igniter_spark,
        } = &output.rfs_actuators;
        let controllino::output::Actuators {
            rocket_power: _,                // not affected
            rocket_heating: _,              // not affected
            lox_decoupler_extend_valve: _,  // not affected
            lox_decoupler_retract_valve: _, // not affected
            n2_decoupler_extend_valve: _,   // not affected
            n2_decoupler_retract_valve: _,  // not affected
            n2_shower_valve,
            n2_engine_purge_valve,
        } = &output.controllino_actuators;

        // RFS
        output.lox_fill_valve.replace(Default::default());
        lox_bleed_valve.replace(Default::default());
        gox_purge_valve.replace(Default::default());
        h2_purge_valve.replace(Default::default());
        n2_bleed_valve.replace(Default::default());
        lox_purge_valve.replace(Default::default());
        gox_bleed_valve.replace(Default::default());
        n2_purge_valve.replace(Default::default());
        n2_fill_valve.replace(Default::default());
        h2_bleed_valve.replace(Default::default());
        booster_valve.replace(Default::default());
        h2_igniter_valve.replace(Default::default());
        gox_igniter_valve.replace(Default::default());
        igniter_spark.replace(Default::default());
        // Controllino
        n2_shower_valve.set(Default::default());
        n2_engine_purge_valve.set(Default::default());

        // PSS
        if self.include_pss {
            output.pss_valves.set_oxidizer_vent(Default::default());
            output
                .pss_valves
                .set_pressurization_vent(Default::default());
            output
                .pss_valves
                .set_fuel_pressurization(Default::default());
            output.pss_valves.set_fuel_vent(Default::default());
            output
                .pss_valves
                .set_oxidizer_pressurization(Default::default());
            output.pss_valves.set_fuel_main(Default::default());
            output.pss_valves.set_oxidizer_main(Default::default());
        }
        self.on_done.clone()
    }
}
