use datatypes::actuator::NormallyClosedValve;
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

/// This state sets all RFS valves to normal position, except for the lox bleed valve, which is opened.
/// Then proceeds to the `on_done` state.
///
/// Used for aborting before there is LOX in the system or the RFS is connected to PSS.
#[derive(Debug)]
pub struct DepressurizeRFS {
    pub on_done: StateName,
}

#[async_trait::async_trait]
impl StateLogic for DepressurizeRFS {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::DepressurizeRFS::run", skip_all, fields(?self, state = %param.current_state().0))]
    #[allow(clippy::default_trait_access)] // I specifically want this, since the restoring default behaviour is the main intention.
    async fn run(
        &self,
        _input: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!("Depressurizing the RFS");

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
            n2_shower_valve,
            n2_engine_purge_valve,
            h2_igniter_valve,
            gox_igniter_valve,
            igniter_spark,
        } = &output.rfs_actuators;

        // Open the bleed valve
        lox_bleed_valve.replace(NormallyClosedValve::Open);
        // Set rest to normal state (Default state)
        output.lox_fill_valve.replace(Default::default());
        gox_purge_valve.replace(Default::default());
        h2_purge_valve.replace(Default::default());
        n2_bleed_valve.replace(Default::default());
        lox_purge_valve.replace(Default::default());
        gox_bleed_valve.replace(Default::default());
        n2_purge_valve.replace(Default::default());
        n2_fill_valve.replace(Default::default());
        h2_bleed_valve.replace(Default::default());
        booster_valve.replace(Default::default());
        n2_shower_valve.replace(Default::default());
        n2_engine_purge_valve.replace(Default::default());
        h2_igniter_valve.replace(Default::default());
        gox_igniter_valve.replace(Default::default());
        igniter_spark.replace(Default::default());

        self.on_done.clone()
    }
}
