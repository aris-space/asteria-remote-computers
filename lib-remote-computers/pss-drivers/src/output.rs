use std::{collections::BTreeMap, time::Duration};

use datatypes::actuator::Settable;
use datatypes::actuator::{DPRValve, NormallyClosedValve, NormallyOpenValve};
use lib_mission::reactive::{IntoWatched as _, Watched};
use tokio::task::AbortHandle;
use watchable::Watchable;

/// The valves in the propellant supply system.
#[derive(Debug, Default, Clone)]
pub struct PSSValves {
    /// Whether this tree is the active controller of these valves,
    /// thus the outputs should be propogated via CAN to the boards.
    /// This switches twice beteen the RFS and FC during the filling phase.
    ///
    /// *Warning*: setting this to true without first setting all the valves will restore the last remembered state,
    /// which is almost never desired.
    pub(crate) is_active: Watchable<bool>,

    /// The target state for the Pressurization Vent Valve (`PRZ_VNT_VLV`).
    pub pressurization_vent_valve: Watchable<NormallyOpenValve>,
    /// The target state for the Fuel Pressurization Valve (DPR) (`FSS_PRZ_VLV`).
    pub fuel_pressurization_valve: Watchable<DPRValve>,
    /// The target state for the Fuel Vent Valve (`FSS_VNT_VLV`).
    pub fuel_vent_valve: Watchable<NormallyOpenValve>,
    /// The target state for the Oxidizer Pressurization Valve (DPR) (`OSS_PRZ_VLV`).
    pub oxidizer_pressurization_valve: Watchable<DPRValve>,
    /// The target state for the Oxidizer Vent Valve (`OSS_VNT_VLV`).
    /// Note that this should be opened in an abort, despite being normally closed.
    pub oxidizer_vent_valve: Watchable<NormallyClosedValve>,
    /// The target state for the Fuel Main Valve (`FSS_MNL_VLV`).
    pub fuel_main_valve: Watchable<NormallyClosedValve>,
    /// The target state for the Oxidizer Main Valve (`OSS_MNL_VLV`).
    pub oxidizer_main_valve: Watchable<NormallyClosedValve>,
}

impl PSSValves {
    /// Sets this output to passive, all drivers for the actuators will stop sending data until it is activated again.
    ///
    /// Returns the active state before this call.
    pub fn set_passive(&self) -> bool {
        self.is_active.update(false).unwrap_or_else(|b| b)
    }

    /// This function will reactivate the outputs by setting all target to the current state as observed.
    /// It will then wait `50ms` to fully propagate the activation.
    /// The resets will most likely not be sent, but any successive changes should be sent again.
    pub async fn set_active(&self, current_state: &crate::input::PSSValves) {
        let _ = self
            .pressurization_vent_valve
            .update(current_state.pressurization_vent_valve.get());
        let _ = self
            .fuel_pressurization_valve
            .update(current_state.fuel_pressurization_valve.get());
        let _ = self
            .fuel_vent_valve
            .update(current_state.fuel_vent_valve.get());
        let _ = self
            .oxidizer_pressurization_valve
            .update(current_state.oxidizer_pressurization_valve.get());
        let _ = self
            .oxidizer_vent_valve
            .update(current_state.oxidizer_vent_valve.get());
        let _ = self
            .fuel_main_valve
            .update(current_state.fuel_main_valve.get());
        let _ = self
            .oxidizer_main_valve
            .update(current_state.oxidizer_main_valve.get());

        // Make sure to not send old values (shouldn't happen anyway).
        tokio::time::sleep(Duration::from_millis(25)).await;
        let _ = self.is_active.update(true);
        // And make sure that successive changes will be sent.
        tokio::time::sleep(Duration::from_millis(25)).await;
    }

    /// Returns a [`Watched`] to see the active state of this struct.
    pub fn watch_active(&self) -> impl Watched<bool> + Send + 'static {
        self.is_active.watched()
    }
}

/// See the matching types in [`PSSValves`].
///
/// ```
/// use pss_drivers::output::ValveName;
/// assert_eq!("oss_vnt_vlv".parse::<ValveName>().unwrap(), ValveName::oss_vnt_vlv)
/// ```
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Deserialize, serde::Serialize,
)]
#[allow(non_camel_case_types)]
pub enum ValveName {
    /// The name of [`PSSValves::pressurization_vent_valve`].
    prz_vnt_vlv,
    /// The name of [`PSSValves::fuel_pressurization_valve`].
    fss_prz_vlv,
    /// The name of [`PSSValves::fuel_vent_valve`].
    fss_vnt_vlv,
    /// The name of [`PSSValves::oxidizer_pressurization_valve`].
    oss_prz_vlv,
    /// The name of [`PSSValves::oxidizer_vent_valve`].
    oss_vnt_vlv,
    /// The name of [`PSSValves::fuel_main_valve`].
    fss_mnl_vlv,
    /// The name of [`PSSValves::oxidizer_main_valve`].
    oss_mnl_vlv,
}
impl std::str::FromStr for ValveName {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        use serde::Deserialize as _;
        Self::deserialize(<&str as serde::de::IntoDeserializer>::into_deserializer(s))
            .map_err(|e| e.to_string())
    }
}

impl PSSValves {
    pub fn as_settables(&self) -> BTreeMap<ValveName, Box<dyn Settable + Send>> {
        macro_rules! boxx {
            ($e:expr) => {
                Box::new($e.clone()) as Box<dyn Settable + Send>
            };
        }
        #[allow(clippy::enum_glob_use)] // rustfmt is nice then
        use ValveName::*;
        BTreeMap::from([
            (prz_vnt_vlv, boxx!(self.pressurization_vent_valve)),
            (fss_prz_vlv, boxx!(self.fuel_pressurization_valve)),
            (fss_vnt_vlv, boxx!(self.fuel_vent_valve)),
            (oss_prz_vlv, boxx!(self.oxidizer_pressurization_valve)),
            (oss_vnt_vlv, boxx!(self.oxidizer_vent_valve)),
            (fss_mnl_vlv, boxx!(self.fuel_main_valve)),
            (oss_mnl_vlv, boxx!(self.oxidizer_main_valve)),
        ])
    }
}

impl PSSValves {
    /// Starts logging valve timeseries to csv files in `folder`.
    /// Used for data analysis.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    pub async fn start_logging(
        &self,
        folder: &std::path::Path,
        time: Watchable<lib_mission::util::time::MissionTime>,
    ) -> Result<[AbortHandle; 8], Box<dyn std::error::Error>> {
        use lib_mission::reactive::IntoWatched as _;
        macro_rules! spawn_serializable_logger {
            ($name:literal, $e:expr, $headers:expr,) => {
                lib_mission::util::log::spawn_watchable_logger(
                    lib_mission::util::log::make_timeseries_writer(folder, $name, &$headers)
                        .await?,
                    $e.watched(),
                    time.watched(),
                )
            };
        }

        let PSSValves {
            is_active,
            pressurization_vent_valve,
            fuel_pressurization_valve,
            fuel_vent_valve,
            oxidizer_pressurization_valve,
            oxidizer_vent_valve,
            fuel_main_valve,
            oxidizer_main_valve,
        } = self;

        // Log valve target states
        Ok([
            spawn_serializable_logger!("pss_mode", is_active, ["active [bool]"],),
            spawn_serializable_logger!(
                "target_state_PRZ_VNT_VLV",
                pressurization_vent_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_FSS_PRZ_VLV",
                fuel_pressurization_valve,
                ["target state setpoint or NaN [barg?]"],
            ),
            spawn_serializable_logger!(
                "target_state_FSS_VNT_VLV",
                fuel_vent_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_OSS_PRZ_VLV",
                oxidizer_pressurization_valve,
                ["target state setpoint or NaN [barg?]"],
            ),
            spawn_serializable_logger!(
                "target_state_OSS_VNT_VLV",
                oxidizer_vent_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_FSS_MNL_VLV",
                fuel_main_valve,
                ["target state [openness %]"],
            ),
            spawn_serializable_logger!(
                "target_state_OSS_MNL_VLV",
                oxidizer_main_valve,
                ["target state [openness %]"],
            ),
        ])
    }
}
