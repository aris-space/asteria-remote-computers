use lib_mission::actuator_control::valves::NormallyClosedValve;
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

/// This state sets all RFS and PSS valves to normal(*) position.
/// Then proceeds to the `on_done` state.
///
/// Note that if the RFS is in passive mode, the PSS valves have no effect, and only RFS valves are affected.
/// (*) Normal state affects only valves, the decouplers are decouplers deliberatly skipped.
/// Additionally, the throttable valves are closed (which they wouldn't do on power loss, but it is their initial state)
/// and the rockets OSS vent valve is opened despite its normally closed state.
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

        let uremote_electronics::output::Actuators {
            rocket_power: _,            // not affected
            maxon_board_power: _,       // not affected
            fss_heating: _,             // not affected
            pressure_sensor_heating: _, // not affected
            eth_pump_power: _,          // not affected
            custom1: _,                 // not affected
            custom2: _,                 // not affected
            custom3: _,                 // not affected
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
            lox_decoupler_extend_valve: _,  // not affected
            lox_decoupler_retract_valve: _, // not affected
            n2_decoupler_extend_valve: _,   // not affected
            n2_decoupler_retract_valve: _,  // not affected
            n2_release_valve,
            h2_igniter_valve,
            gox_igniter_valve,
            igniter_spark,
        } = &output.rfs_actuators;

        let pss_drivers::output::PSSValves {
            pressurization_vent_valve,
            fuel_pressurization_valve,
            fuel_vent_valve,
            oxidizer_pressurization_valve,
            oxidizer_vent_valve,
            fuel_main_valve,
            oxidizer_main_valve,
            .. // catches the private `is_active` field
        } = &output.pss_valves;

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
        n2_release_valve.replace(Default::default());
        h2_igniter_valve.replace(Default::default());
        gox_igniter_valve.replace(Default::default());
        igniter_spark.replace(Default::default());
        // PSS
        pressurization_vent_valve.replace(Default::default());
        fuel_pressurization_valve.replace(Default::default());
        fuel_vent_valve.replace(Default::default());
        oxidizer_pressurization_valve.replace(Default::default());
        oxidizer_vent_valve.replace(NormallyClosedValve::Open);
        fuel_main_valve.replace(Default::default());
        oxidizer_main_valve.replace(Default::default());

        self.on_done.clone()
    }
}
