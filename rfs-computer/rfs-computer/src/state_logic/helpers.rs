use datatypes::units::{BarG, Celsius};
use lib_mission::reactive::{IntoWatched as _, WatchedExt as _};

use crate::io::input::InputTree;

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

/// Parts of [`crate::io::input::InputTree`], see that.
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
        let mut igniter_p_watcher = input_tree.controllino_sensors.igniter_pressure.watched();
        let mut igniter_t_watcher = input_tree.controllino_sensors.igniter_temperature.watched();

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

