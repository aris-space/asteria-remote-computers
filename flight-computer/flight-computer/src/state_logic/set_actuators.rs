use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

/// This state sets the configured actuators to their values and then continues to `on_done`
#[derive(Debug)]
pub struct SetActuators {
    pub targets: BTreeMap<pss_drivers::output::ValveName, Value>,
    pub on_done: StateName,
}

#[async_trait::async_trait]
impl StateLogic for SetActuators {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::SetActuators::run", skip_all, fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        _input: &Self::Input,
        output: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        let mut settables = output.valves.as_settables();

        for (name, &target) in &self.targets {
            let Some(settable) = settables.get_mut(name) else {
                tracing::error!(
                    "All valid actuator names should have a settable in the output tree"
                );
                continue;
            };
            tracing::info!("Setting {name:?} to {target:?}");
            if settable.set_value(f32::from(target)).is_err() {
                tracing::error!(
                    ?name,
                    ?target,
                    "Configured target value is invalid for the actuator"
                );
            }
        }

        self.on_done.clone()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub enum Value {
    /// Synonym for 1. Used for valves.
    Open,
    /// Synonym for 0. Used for valves.
    Close,
    /// Synonym for NaN. Used for DPR valves.
    DPRDisabled,
    /// Synonym for the inner value. Used for setting a setpoint of DPR valves.
    SetpointBarG(f32),
}

/// To be used as a value for [`datatypes::actuator::Settable`].
impl From<Value> for f32 {
    fn from(value: Value) -> Self {
        match value {
            Value::Close => 0.0,
            Value::Open => 1.0,
            Value::DPRDisabled => f32::NAN,
            Value::SetpointBarG(barg) => barg,
        }
    }
}

#[test]
fn deser_value() {
    fn assert_deser(ser: &str, expected: Value) {
        // Because toml needs objects
        #[derive(Deserialize)]
        struct A {
            x: Value,
        }
        assert_eq!(
            toml::from_str::<A>(&format!("x = {ser}"))
                .expect("failed to parse value")
                .x,
            expected
        );
    }
    assert_deser("\"Open\"", Value::Open);
    assert_deser("\"Close\"", Value::Close);
    assert_deser("\"DPRDisabled\"", Value::DPRDisabled);
    assert_deser("{\"SetpointBarG\" = 1.0 }", Value::SetpointBarG(1.0));
}
