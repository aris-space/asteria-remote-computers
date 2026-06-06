use std::{collections::BTreeMap, ops::ControlFlow, time::Duration};

use datatypes::units::{BarG, Celsius};
use lib_mission::reactive::{IntoWatched as _, WatchedExt as _};
use serde::{Deserialize, Serialize};

use crate::io::{input::InputTree, output::ActuatorName};

/// A struct which can be configured with certain sensor thresholds.
///
/// The function [`Self::watch`] will observe and only return when a violation is found to any configured threshold.
#[allow(clippy::unsafe_derive_deserialize)] // wtf kind of false positive is this
#[derive(Debug, Default, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct ThresholdWatcher {
    /// A violation is raised if any of the pressures are at or above the given values in barg.
    #[serde(default)]
    upper_thresholds_barg: PressureThresholds,
    /// A violation is raised if any of the temperatures are at or above the given values in degrees celsius.
    #[serde(default)]
    upper_thresholds_celsius: TemperatureThresholds,
}

/// Parts of [`crate::io::input::PressureSensors`], see that.
/// Referrs to pressure channel 1
/// Can be extended as needed.
#[derive(Debug, serde::Deserialize, serde::Serialize)]
struct PressureThresholds {
    #[serde(default = "nan")]
    gox: f32,
    #[serde(default = "nan")]
    h2: f32,
    #[serde(default = "nan")]
    igniter: f32,
}

impl Default for PressureThresholds {
    fn default() -> Self {
        Self {
            gox: f32::NAN,
            h2: f32::NAN,
            igniter: f32::NAN,
        }
    }
}

/// Parts of [`uremote_electronics::input::Sensors`], see that.
#[derive(Debug, serde::Deserialize, serde::Serialize)]
struct TemperatureThresholds {
    #[serde(default = "nan")]
    igniter: f32,
}

impl Default for TemperatureThresholds {
    fn default() -> Self {
        Self { igniter: f32::NAN }
    }
}

/// What has been violated.
#[derive(Debug)]
pub struct Violation(pub String);

impl std::fmt::Display for Violation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

/// Because `serde(default)` requires a function
fn nan() -> f32 {
    f32::NAN
}

impl ThresholdWatcher {
    /// Returns only if a violation of a threshold has been observed, with a description of that violation.
    #[allow(clippy::similar_names)] // don't care
    pub async fn watch(&self, input_tree: &InputTree) -> Violation {
        // Somewhat ugly having to define all watchers.
        // Because comparision with NAN is always false, unspecified thresholds should never trigger a violation
        let mut gox_watcher = input_tree.gox_pressure.watched();
        let mut h2_watcher = input_tree.h2_pressure.watched();
        let mut igniter_p_watcher = input_tree.igniter_pressure.watched();
        let mut igniter_t_watcher = input_tree.uremote_sensors.igniter_temperature.watched();

        tokio::select! {
            p = gox_watcher.next_satisfying(|&BarG(p)| p > self.upper_thresholds_barg.gox) => {
                Violation(format!("gox pressure ({p:?}) rose above threshold"))
            }
            p = h2_watcher.next_satisfying(|&BarG(p)| p > self.upper_thresholds_barg.h2) => {
                Violation(format!("h2 pressure ({p:?}) rose above threshold"))
            }
            p = igniter_p_watcher.next_satisfying(|&BarG(p)| p > self.upper_thresholds_barg.igniter) => {
                Violation(format!("igniter pressure ({p:?}) rose above threshold"))
            }
            t = igniter_t_watcher.next_satisfying(|&Celsius(t)| t > self.upper_thresholds_celsius.igniter) => {
                Violation(format!("igniter temperature ({t:?}) rose above threshold"))
            }
        }
    }
}

/// An action to perform as part of a sequence.
///
/// ```
/// # #[derive(serde::Deserialize)]
/// # struct Seq {
/// #   sequence: Vec<rfs_computer::state_logic::helpers::Action>,
/// # }
/// let toml_ser = r#"
/// [[sequence]]
/// action = "SetActuators"
/// targets = { "gox_prg_vlv" = "Open" }
/// "#;
/// # toml::from_str::<Seq>(toml_ser).expect("must parse example");
/// ```
#[derive(Debug, PartialEq, Deserialize, Serialize)]
#[serde(tag = "action")]
#[serde(deny_unknown_fields)]
pub enum Action {
    /// This action sets all the valves defined in `targets` to the respective values.
    SetActuators {
        targets: BTreeMap<ActuatorName, Value>,
    },
    /// An action that simply waits for `duration_sec` seconds.
    Wait { duration_sec: f32 },
    /// This action waits for at most `patience_sec` until the igniter pressure reaches above `pressure_lower_thresh_barg`.
    IgnitionDetection {
        pressure_lower_thresh_barg: f32,
        patience_sec: f32,
    },
    /// This action just monitors the igniter pressure for `duration_sec` and if it falls under `min_pressure_barg` cancels,
    /// otherwise waits out the duration.
    SustainedIgnition {
        min_pressure_barg: f32,
        duration_sec: f32,
    },
    /// This action waits for at most `patience_sec` until the rpn3 and n2 tank pressure reaches above `pressure_lower_thresh_barg`.
    N2FillDetection {
        pressure_lower_thresh_barg: f32,
        patience_sec: f32,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub enum Value {
    /// Synonym for 1. Used for valves.
    Open,
    /// Synonym for 0. Used for valves.
    Close,
    /// Synonym for 1. Used for spark plug.
    On,
    /// Synonym for 0. Used for spark plug.
    Off,
    /// Synonym for NaN. Used for DPR valves.
    DPRDisabled,
    /// Synonym for the inner value. Used for setting a setpoint of DPR valves.
    SetpointBarG(f32),
}

/// To be used as a value for `Settable`.
impl From<Value> for f32 {
    fn from(value: Value) -> Self {
        match value {
            Value::Close | Value::Off => 0.0,
            Value::On | Value::Open => 1.0,
            Value::DPRDisabled => f32::NAN,
            Value::SetpointBarG(barg) => barg,
        }
    }
}

/// Executes a sequence of actions, see [`Action`].
#[allow(clippy::too_many_lines)] // whatever
pub async fn run_sequence(
    sequence: impl IntoIterator<Item = &Action>,
    output: &super::Output,
    input: &super::Input,
) -> ControlFlow<()> {
    let mut settables = output.as_settables();
    for (i, action) in sequence.into_iter().enumerate() {
        match action {
            Action::SetActuators { targets } => {
                for (name, &target) in targets {
                    let Some(settable) = settables.get_mut(name) else {
                        tracing::error!(
                            "All valid actuator names should have a settable in the output tree"
                        );
                        continue;
                    };
                    tracing::info!("Action {i}: Setting {name:?} to {target:?}");
                    if settable.set_value(f32::from(target)).is_err() {
                        tracing::error!(
                            ?name,
                            ?target,
                            "Configured target value is invalid for the actuator"
                        );
                    }
                }
            }
            Action::Wait { duration_sec } => {
                tracing::info!("Action {i}: Waiting for {duration_sec:.2}s");
                let duration = Duration::from_secs_f32(*duration_sec);
                let start = input.time.get();
                let mut t_watcher = input.time.watched();
                t_watcher.next_satisfying(|t| *t > start + duration).await;
            }
            &Action::IgnitionDetection {
                pressure_lower_thresh_barg,
                patience_sec,
            } => {
                tracing::info!("Action {i}: Detecting ignition in the next {patience_sec:.2}s");
                let patience = Duration::from_secs_f32(patience_sec);
                let start = input.time.get();
                let mut t_watcher = input.time.watched();
                let patience_timeout = t_watcher.next_satisfying(|t| *t > start + patience);
                let mut p_watcher = input.igniter_pressure.watched();
                let thresh_reached =
                    p_watcher.next_satisfying(|&BarG(p)| p > pressure_lower_thresh_barg);

                tokio::select! {
                    BarG(p) = thresh_reached => {
                        tracing::info!("Action {i}: Ignition detected {p:.2}s");
                    }
                    _ = patience_timeout => {
                        tracing::warn!("Action {i}: Timeout for ignition detection reached, aborting");
                        return ControlFlow::<()>::Break(());
                    }
                }
            }
            &Action::SustainedIgnition {
                min_pressure_barg,
                duration_sec,
            } => {
                tracing::info!("Action {i}: Sustaining ignition for {duration_sec:.2}s");
                let duration = Duration::from_secs_f32(duration_sec);
                let start = input.time.get();
                let mut t_watcher = input.time.watched();
                let duration_reached = t_watcher.next_satisfying(|t| *t > start + duration);

                let mut p_watcher = input.igniter_pressure.watched();
                let thresh_reached = p_watcher.next_satisfying(|&BarG(p)| p < min_pressure_barg);

                tokio::select! {
                    _ = duration_reached => {
                        tracing::info!("Action {i}: Ignition done");
                    }
                    BarG(p) = thresh_reached => {
                        tracing::warn!(%p, "Action {i}: Lower threshhold reached, aborting");
                        return ControlFlow::<()>::Break(());
                    }
                }
            }
            &Action::N2FillDetection {
                pressure_lower_thresh_barg,
                patience_sec,
            } => {
                tracing::info!("Action {i}: Detecting n2 filling in the next {patience_sec:.1}s");
                let patience = Duration::from_secs_f32(patience_sec);
                let start = input.time.get();
                let mut t_watcher = input.time.watched();
                let patience_timeout = t_watcher.next_satisfying(|t| *t > start + patience);
                // FIXME: also depend on rpn3 (n2_post_booster).
                let mut p_watcher = input.pss_sensors.pressurization_line_pressure.watched();
                let thresh_reached =
                    p_watcher.next_satisfying(|&BarG(p)| p > pressure_lower_thresh_barg);

                tokio::select! {
                    BarG(p) = thresh_reached => {
                        tracing::info!("Action {i}: Ignition detected {p:.2}s");
                    }
                    _ = patience_timeout => {
                        tracing::warn!("Action {i}: Timeout for ignition detection reached, aborting");
                        return ControlFlow::<()>::Break(());
                    }
                }
            }
        }
    }
    tracing::info!("Sequence completed");
    ControlFlow::<()>::Continue(())
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
    assert_deser("\"Off\"", Value::Off);
    assert_deser("\"On\"", Value::On);
    assert_deser("\"DPRDisabled\"", Value::DPRDisabled);
    assert_deser("{\"SetpointBarG\" = 1.0 }", Value::SetpointBarG(1.0));
}
