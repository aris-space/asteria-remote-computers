use crate::io::input::InputTree;
use datatypes::units::BarG;
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
#[derive(Debug, serde::Deserialize, serde::Serialize, Default)]
struct PressureThresholds {
    fuel_tank_pressure: BarG,
    fuel_tank_pressure_1: BarG,
    fuel_tank_pressure_2: BarG,
    oxidizer_tank_pressure: BarG,
    oxidizer_tank_pressure_1: BarG,
    oxidizer_tank_pressure_2: BarG,
}

/// What has been violated.
#[derive(Debug)]
pub struct Violation(pub String);

impl std::fmt::Display for Violation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
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
            p = fuel_tank_pressure_watcher.next_satisfying(|p| p.0 > self.upper_thresholds_barg.fuel_tank_pressure.0) => {
                Violation(format!("fuel tank pressure ({p:?}) rose above threshold"))
            }
            p = oxidizer_tank_pressure_watcher.next_satisfying(|p| p.0 > self.upper_thresholds_barg.oxidizer_tank_pressure.0) => {
                Violation(format!("oxidizer tank pressure ({p:?}) rose above threshold"))
            }
        }
    }
}
