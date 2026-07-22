use watchable::Watchable;

use crate::{input, output};

impl output::Actuators {
    /// Starts logging valve timeseries to csv files in `folder`.
    /// Used for data analysis.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    #[allow(clippy::too_many_lines)] // It is fairly readable despite this and also can't really be reduced without proc macros.
    pub async fn start_logging(
        &self,
        folder: &std::path::Path,
        time: &Watchable<lib_mission::util::time::MissionTime>,
    ) -> Result<[tokio::task::AbortHandle; 17], Box<dyn std::error::Error>> {
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

        let Self {
            maxon_board_power,
            custom1,
            custom2,
            custom3,
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
            h2_igniter_valve,
            gox_igniter_valve,
            igniter_spark,
        } = self;

        // Log valve target states
        Ok([
            spawn_serializable_logger!(
                "target_state_maxon_board_power",
                maxon_board_power,
                ["target state [On|Off]"],
            ),
            spawn_serializable_logger!("target_state_custom1", custom1, ["target state [On|Off]"],),
            spawn_serializable_logger!("target_state_custom2", custom2, ["target state [On|Off]"],),
            spawn_serializable_logger!("target_state_custom3", custom3, ["target state [On|Off]"],),
            spawn_serializable_logger!(
                "target_state_gox_prg_vlv",
                gox_purge_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_h2_prg_vlv",
                h2_purge_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_n2_bld_vlv",
                n2_bleed_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_lox_prg_vlv",
                lox_purge_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_gox_bld_vlv",
                gox_bleed_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_n2_prg_vlv",
                n2_purge_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_n2_fll_vlv",
                n2_fill_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_h2_bld_vlv",
                h2_bleed_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_lox_bld_vlv",
                lox_bleed_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_n2_bst_vlv",
                booster_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_h2_ign_vlv",
                h2_igniter_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_gox_ign_vlv",
                gox_igniter_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_ign_spk_plg",
                igniter_spark,
                ["target state [On|Off]"],
            ),
        ])
    }
}

impl input::Actuators {
    /// Starts logging valve timeseries to csv files in `folder`.
    /// Used for data analysis.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    #[allow(clippy::too_many_lines)] // It is fairly readable despite this and also can't really be reduced without proc macros.
    pub async fn start_logging(
        &self,
        folder: &std::path::Path,
        time: &Watchable<lib_mission::util::time::MissionTime>,
    ) -> Result<[tokio::task::AbortHandle; 17], Box<dyn std::error::Error>> {
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

        let Self {
            maxon_board_power,
            custom1,
            custom2,
            custom3,
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
            h2_igniter_valve,
            gox_igniter_valve,
            igniter_spark,
        } = self;

        // Log valve observed states
        Ok([
            spawn_serializable_logger!(
                "observed_state_maxon_board_power",
                maxon_board_power,
                ["observed state [On|Off]"],
            ),
            spawn_serializable_logger!(
                "observed_state_custom1",
                custom1,
                ["observed state [On|Off]"],
            ),
            spawn_serializable_logger!(
                "observed_state_custom2",
                custom2,
                ["observed state [On|Off]"],
            ),
            spawn_serializable_logger!(
                "observed_state_custom3",
                custom3,
                ["observed state [On|Off]"],
            ),
            spawn_serializable_logger!(
                "observed_state_gox_prg_vlv",
                gox_purge_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_h2_prg_vlv",
                h2_purge_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_n2_bld_vlv",
                n2_bleed_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_lox_prg_vlv",
                lox_purge_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_gox_bld_vlv",
                gox_bleed_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_n2_prg_vlv",
                n2_purge_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_n2_fll_vlv",
                n2_fill_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_h2_bld_vlv",
                h2_bleed_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_lox_bld_vlv",
                lox_bleed_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_n2_bst_vlv",
                booster_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_h2_ign_vlv",
                h2_igniter_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_gox_ign_vlv",
                gox_igniter_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_ign_spk_plg",
                igniter_spark,
                ["observed state [On|Off]"],
            ),
        ])
    }
}

impl input::Sensors {
    /// Starts logging sensors timeseries to csv files in `folder`.
    /// Used for data analysis.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    pub async fn start_logging(
        &self,
        folder: &std::path::Path,
        time: &Watchable<lib_mission::util::time::MissionTime>,
    ) -> Result<[tokio::task::AbortHandle; 5], Box<dyn std::error::Error>> {
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

        let Self {
            lox_temperature,
            cabinet_temperature,
            ambient_temperature,
            valves_armed,
            igniter_armed,
        } = self;

        Ok([
            spawn_serializable_logger!("sensor_rtl1", lox_temperature, ["lox_temperature [°C]"],),
            spawn_serializable_logger!(
                "sensor_cabinet",
                cabinet_temperature,
                ["cabinet_temperature [°C]"],
            ),
            spawn_serializable_logger!(
                "sensor_ambient",
                ambient_temperature,
                ["ambient_temperature [°C]"],
            ),
            spawn_serializable_logger!("arming_valve_power", valves_armed, ["are armed [bool]"],),
            spawn_serializable_logger!("arming_igniter_power", igniter_armed, ["is armed [bool]"],),
        ])
    }
}
