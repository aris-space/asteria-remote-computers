use crate::io::input::InputTree;
use lib_mission::reactive::{IntoWatched, WatchedExt};

/// A struct which can be configured with certain sensor thresholds.
///
/// The function [`Self::watch`] will observe and only return when a violation is found to any configured threshold.
#[allow(clippy::unsafe_derive_deserialize)] // wtf kind of false positive is this
#[derive(Debug, Default, serde::Deserialize, serde::Serialize)]
pub struct ThresholdWatcher {
    /// A violation is raised if any of the pressures are at or above the given values in barg.
    #[serde(default)]
    upper_thresholds_barg: PressureThresholds,
}

/// Parts of [`pss_drivers::input::PSSSensors`], see that.
/// Can be extended as needed.
#[derive(Debug, serde::Deserialize, serde::Serialize)]
struct PressureThresholds {
    #[serde(default = "nan")]
    fuel_tank_pressure: f32,
    #[serde(default = "nan")]
    fuel_tank_pressure_1: f32,
    #[serde(default = "nan")]
    fuel_tank_pressure_2: f32,
    #[serde(default = "nan")]
    oxidizer_tank_pressure: f32,
    #[serde(default = "nan")]
    oxidizer_tank_pressure_1: f32,
    #[serde(default = "nan")]
    oxidizer_tank_pressure_2: f32,
}

impl Default for PressureThresholds {
    fn default() -> Self {
        Self {
            fuel_tank_pressure: f32::NAN,
            fuel_tank_pressure_1: f32::NAN,
            fuel_tank_pressure_2: f32::NAN,
            oxidizer_tank_pressure: f32::NAN,
            oxidizer_tank_pressure_1: f32::NAN,
            oxidizer_tank_pressure_2: f32::NAN,
        }
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
        // Because comparision with NAN is always false, unspecified thresholds should never trigger a violation
        let mut fuel_tank_pressure_watcher =
            input_tree.pss_sensors.fuel_tank_pressure_filtered.watched();
        let mut oxidizer_tank_pressure_watcher = input_tree
            .pss_sensors
            .oxidizer_tank_pressure_sensor1
            .watched();

        tokio::select! {
            p = fuel_tank_pressure_watcher.next_satisfying(|p| p.0 > self.upper_thresholds_barg.fuel_tank_pressure) => {
                Violation(format!("fuel tank pressure ({p:?}) rose above threshold"))
            }
            p = oxidizer_tank_pressure_watcher.next_satisfying(|p| p.0 > self.upper_thresholds_barg.oxidizer_tank_pressure) => {
                Violation(format!("oxidizer tank pressure ({p:?}) rose above threshold"))
            }
        }
    }
}
