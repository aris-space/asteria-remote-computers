use std::collections::BTreeMap;
use std::time::Duration;

use datatypes::actuator::{NormallyClosedValve, NormallyOpenValve, Settable};
use lib_mission::actuator_control::command::{ActuatorControl, Command};
use serde::{Deserialize, Serialize};
use tokio::time::sleep;
use watchable::Watchable;

#[derive(Debug, Default, Clone)]
pub struct ExampleActuators {
    /// Valve A
    pub valve_a: Watchable<NormallyOpenValve>,
    /// Valve B
    pub valve_b: Watchable<NormallyClosedValve>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
#[allow(non_camel_case_types)]
#[serde(rename_all = "lowercase")]
pub enum ActuatorName {
    A_Vlv,
    B_Vlv,
}

impl std::str::FromStr for ActuatorName {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Self::deserialize(<&str as serde::de::IntoDeserializer>::into_deserializer(s))
            .map_err(|e| e.to_string())
    }
}

impl ExampleActuators {
    pub fn as_settables(&self) -> BTreeMap<ActuatorName, Box<dyn Settable + Send>> {
        BTreeMap::from([
            (
                ActuatorName::A_Vlv,
                Box::new(self.valve_a.clone()) as Box<dyn Settable + Send>,
            ),
            (
                ActuatorName::B_Vlv,
                Box::new(self.valve_b.clone()) as Box<dyn Settable + Send>,
            ),
        ])
    }
}

#[derive(Debug, Default, Clone)]
struct Input(Watchable<Option<Command<ActuatorName>>>);

#[tokio::test]
async fn scenario() {
    let _ = tracing_subscriber::fmt().try_init();

    let input = Input::default();
    let output = ExampleActuators::default();

    let actuators = ActuatorControl::<ActuatorName>::from_strings(["a_vlv", "b_vlv"])
        .expect("must be parseable");
    let task = {
        let output = output.clone();
        let input = input.0.clone();
        tokio::spawn(async move { actuators.act(3, &input, output.as_settables()).await })
    };

    assert!(output.valve_a.get().is_open());
    assert!(output.valve_b.get().is_closed());

    // Close Valve A
    input.0.replace(Some(Command {
        in_state_index: 3,
        actuator: ActuatorName::A_Vlv,
        value: 0.0,
    }));
    sleep(Duration::from_millis(20)).await;

    assert!(output.valve_a.get().is_closed());
    assert!(output.valve_b.get().is_closed());

    // Open Valve A
    input.0.replace(Some(Command {
        in_state_index: 3,
        actuator: ActuatorName::A_Vlv,
        value: 1.0,
    }));
    sleep(Duration::from_millis(20)).await;

    assert!(output.valve_a.get().is_open());
    assert!(output.valve_b.get().is_closed());

    // Open Valve B
    input.0.replace(Some(Command {
        in_state_index: 3,
        actuator: ActuatorName::B_Vlv,
        value: 1.0,
    }));
    sleep(Duration::from_millis(20)).await;

    assert!(output.valve_a.get().is_open());
    assert!(output.valve_b.get().is_open());

    // Try to close Valve A
    input.0.replace(Some(Command {
        in_state_index: 4, // wrong index
        actuator: ActuatorName::A_Vlv,
        value: 0.0,
    }));
    sleep(Duration::from_millis(20)).await;

    assert!(output.valve_a.get().is_open());
    assert!(output.valve_b.get().is_open());

    // Try to close Valve B
    input.0.replace(Some(Command {
        in_state_index: 3,
        actuator: ActuatorName::B_Vlv,
        value: f32::NAN, // bad value
    }));
    sleep(Duration::from_millis(20)).await;

    assert!(output.valve_a.get().is_open());
    assert!(output.valve_b.get().is_open());

    task.abort();
}

#[tokio::test]
async fn unconfigured() {
    let _ = tracing_subscriber::fmt().try_init();

    let input = Input::default();
    let output = ExampleActuators::default();

    let actuators =
        ActuatorControl::<ActuatorName>::from_strings(["b_vlv"]).expect("must be parseable");
    let task = {
        let output = output.clone();
        let input = input.clone();
        tokio::spawn(async move { actuators.act(3, &input.0, output.as_settables()).await })
    };

    assert!(output.valve_a.get().is_open());
    assert!(output.valve_b.get().is_closed());

    // Open Valve B
    input.0.replace(Some(Command {
        in_state_index: 3,
        actuator: ActuatorName::B_Vlv,
        value: 1.0,
    }));
    sleep(Duration::from_millis(20)).await;

    assert!(output.valve_a.get().is_open());
    assert!(output.valve_b.get().is_open());

    // Try to close Valve A
    input.0.replace(Some(Command {
        in_state_index: 3,
        actuator: ActuatorName::A_Vlv, // not allowed
        value: 0.0,
    }));
    sleep(Duration::from_millis(20)).await;

    assert!(output.valve_a.get().is_open());
    assert!(output.valve_b.get().is_open());

    task.abort();
}
