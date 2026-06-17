use datatypes::{
    actuator::ThrottleableValve,
    units::{BarG, Celsius},
};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use watchable::Watchable;

/// This state checks the health of the sensor readings
///
/// If one of the sensors is unhealthy, it logs the problem and goes to the `on_tests_fail` state.
/// Else if all sensors are healthy, it goes to the `on_tests_pass` state.
#[derive(Debug)]
pub struct DiagnosticsGate {
    pub on_tests_pass: StateName,
    pub on_tests_fail: StateName,
}

#[async_trait::async_trait]
impl StateLogic for DiagnosticsGate {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::DiagnosticsGate::run", skip_all, fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        _: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        /// All temperature readings must be non-NaN, thus the sensor must have been reached last time it was polled.
        fn check_temp(reading: &Watchable<Celsius>, name: &str) -> bool {
            let good = reading.get().0.is_finite();
            if !good {
                tracing::warn!("Unhealthy sensor: Temperature reading from {name} is unavailable");
            }
            good
        }
        fn check_pres(reading: &Watchable<BarG>, name: &str) -> bool {
            let good = reading.get().0.is_finite();
            if !good {
                tracing::warn!("Unhealthy sensor: Pressure reading from {name} is unavailable");
            }
            good
        }
        fn check_valve(reading: &Watchable<ThrottleableValve>, name: &str) -> bool {
            let good = (0..=100).contains(&reading.get().openness_percent);
            if !good {
                tracing::warn!(
                    "Unhealthy valve: Current valve position from {name} is unavailable"
                );
            }
            good
        }

        tracing::info!("Entered the Diagnostics state...");

        let uremote_electronics::input::Sensors {
            lox_temperature,
            igniter_temperature,
            cabinet_temperature: _,     // not important
            ambient_temperature: _,     // not important
            lox_decoupler_proximity: _, // nothing to check
            n2_decoupler_proximity: _,  // nothing to check
            valves_armed: _,            // nothing to check
            igniter_armed: _,           // nothing to check
        } = &input.uremote_sensors;

        let healths = [
            check_pres(&input.n2_pre_booster_pressure, "n2_pre_booster 23SX Ch1"),
            check_pres(&input.h2_purge_pressure, "h2_purge 23SX Ch1"),
            check_pres(&input.lox_pressure, "lox 23SX Ch1"),
            check_pres(&input.gox_pressure, "gox 23SX Ch1"),
            check_pres(&input.purge_pressure, "purge 23SX Ch1"),
            check_pres(&input.pneumatic_pressure, "pneumatic 23SX Ch1"),
            check_pres(&input.n2_post_booster_pressure, "n2_post_bstr 23SX Ch1"),
            check_pres(&input.h2_pressure, "h2 23SX Ch1"),
            check_pres(&input.igniter_pressure, "igniter 23SX Ch1"),
            check_temp(&input.uremote_sensors.lox_temperature, "lox Thermocouple"),
            check_temp(
                &input.uremote_sensors.igniter_temperature,
                "igniter Thermocouple",
            ),
            check_valve(&input.lox_fill_valve, "lox fill valve"),
            // Channel 2 is disabled
            // check_pres(&n2_pre_booster.pressure_value_2, "n2_pre_bstr 23SX Ch2"),
            // check_pres(&h2_purge.pressure_value_2, "h2_purge 23SX Ch2"),
            // check_pres(&lox.pressure_value_2, "lox 23SX Ch2"),
            // check_pres(&gox.pressure_value_2, "gox 23SX Ch2"),
            // check_pres(&purge.pressure_value_2, "purge 23SX Ch2"),
            // check_pres(&pneumatic.pressure_value_2, "pneumatic 23SX Ch2"),
            // check_pres(&n2_post_booster.pressure_value_2, "n2_post_bstr 23SX Ch2"),
            // check_pres(&h2.pressure_value_2, "h2 23SX Ch2"),
            // check_pres(&igniter.pressure_value_2, "ingiter 23SX Ch2"),
            // Temperature readouts are disabled
            // check_temp(&n2_pre_booster.temperature_value_1, "n2_pre_bstr 23SX Ch1"),
            // check_temp(&h2_purge.temperature_value_1, "h2_purge 23SX Ch1"),
            // check_temp(&lox.temperature_value_1, "lox 23SX Ch1"),
            // check_temp(&gox.temperature_value_1, "gox 23SX Ch1"),
            // check_temp(&purge.temperature_value_1, "purge 23SX Ch1"),
            // check_temp(&pneumatic.temperature_value_1, "pneumatic 23SX Ch1"),
            // check_temp(&n2_post_booster.temperature_value_1, "n2_post_b 23SX Ch1"),
            // check_temp(&h2.temperature_value_1, "h2 23SX Ch1"),
            // check_temp(&igniter.temperature_value_1, "igniter 23SX Ch1"),
            // Channel 2 is disabled
            // check_temp(&n2_pre_booster.temperature_value_2, "n2_pre_bstr 23SX Ch2"),
            // check_temp(&h2_purge.temperature_value_2, "h2_purge 23SX Ch2"),
            // check_temp(&lox.temperature_value_2, "lox 23SX Ch2"),
            // check_temp(&gox.temperature_value_2, "gox 23SX Ch2"),
            // check_temp(&purge.temperature_value_2, "purge 23SX Ch2"),
            // check_temp(&pneumatic.temperature_value_2, "pneumatic 23SX Ch2"),
            // check_temp(&n2_post_booster.temperature_value_2, "n2_post_b 23SX Ch2"),
            // check_temp(&h2.temperature_value_2, "h2 23SX Ch2"),
            // check_temp(&igniter.temperature_value_2, "igniter 23SX Ch2"),
            check_temp(lox_temperature, "lox Thermocouple"),
            check_temp(igniter_temperature, "igniter Thermocouple"),
            check_valve(&input.lox_fill_valve, "lox fill valve"),
        ];
        if healths.iter().all(|&x| x) {
            self.on_tests_pass.clone()
        } else {
            self.on_tests_fail.clone()
        }
    }
}
