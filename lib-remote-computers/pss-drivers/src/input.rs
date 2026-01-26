use datatypes::actuator::{DPRValve, NormallyClosedValve, NormallyOpenValve};
use lib_mission::util::types::{BarG, Celsius, Percent};
use watchable::Watchable;

/// The feedback for the valves in the propellant supply system.
#[derive(Debug, Default, Clone)]
pub struct PSSValves {
    /// The current state for the Pressurization Vent Valve (`PRZ_VNT_VLV`).
    pub pressurization_vent_valve: Watchable<NormallyOpenValve>,
    /// The current state for the Fuel Pressurization Valve (DPR) (`FSS_PRZ_VLV`).
    pub fuel_pressurization_valve: Watchable<DPRValve>,
    /// The current state for the Fuel Vent Valve (`FSS_VNT_VLV`).
    pub fuel_vent_valve: Watchable<NormallyOpenValve>,
    /// The current state for the Oxidizer Pressurization Valve (DPR) (`OSS_PRZ_VLV`).
    pub oxidizer_pressurization_valve: Watchable<DPRValve>,
    /// The current state for the Oxidizer Vent Valve (`OSS_VNT_VLV`).
    /// Note that this should be opened in an abort, despite being normally closed.
    pub oxidizer_vent_valve: Watchable<NormallyClosedValve>,
    /// The current state for the Fuel Main Valve (`FSS_MNL_VLV`).
    pub fuel_main_valve: Watchable<NormallyClosedValve>,
    /// The current state for the Oxidizer Main Valve (`OSS_MNL_VLV`).
    pub oxidizer_main_valve: Watchable<NormallyClosedValve>,
}

#[derive(Debug, Default, Clone)]
pub struct PSSSensors {
    /// Pressurization Line Pressure in barg `PRZ_MNL_P`
    pub pressurization_line_pressure: Watchable<BarG>,
    /// Fuel Tank Pressure filtered from both sensors in barg.
    pub fuel_tank_pressure: Watchable<BarG>,
    /// Fuel Tank Pressure Sensor 1 in barg `FSS_TNK_P1`
    pub fuel_tank_pressure_1: Watchable<BarG>,
    /// Fuel Tank Pressure Sensor 2 in barg `FSS_TNK_P2`
    pub fuel_tank_pressure_2: Watchable<BarG>,
    /// Oxidizer Tank Pressure filtered from both sensors in barg.
    pub oxidizer_tank_pressure: Watchable<BarG>,
    /// Oxidizer Tank Pressure Sensor 1 in barg `OSS_TNK_P1`
    pub oxidizer_tank_pressure_1: Watchable<BarG>,
    /// Oxidizer Tank Pressure Sensor 2 in barg `OSS_TNK_P2`
    pub oxidizer_tank_pressure_2: Watchable<BarG>,
    /// Injector fuel pressure in barg `FSS_INJ_P`
    pub injector_fuel_pressure: Watchable<BarG>,
    /// Injector oxidizer pressure in barg `OSS_INJ_P`
    pub injector_oxidizer_pressure: Watchable<BarG>,
    /// Engine chamber pressure in barg `ENG_CC_P`
    pub engine_chamber_pressure: Watchable<BarG>,

    /// Oxidizer tank tevel in percent, 0 means empty, 100 means full, increments of 1%.
    pub oxidizer_fill_fraction: Watchable<Percent>,

    /// Fuel Tank Temperature Sensor `FSS_TNK_T`
    pub fuel_tank_temperature: Watchable<Celsius>,
    /// Oxidizer Tank bottom Temperature Sensor `OSS_TNK_T`
    pub oxidizer_tank_temperature: Watchable<Celsius>,
    /// Oxidizer Runline Temperature Sensor `OSS_RNL_T`
    pub oxidizer_runline_temperature: Watchable<Celsius>,
}

impl PSSValves {
    /// Starts logging valve timeseries to csv files in `folder`.
    /// Used for data analysis.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    pub async fn start_logging(
        &self,
        folder: &std::path::Path,
        time: &Watchable<lib_mission::util::time::MissionTime>,
    ) -> Result<[tokio::task::AbortHandle; 7], Box<dyn std::error::Error>> {
        use lib_mission::reactive::IntoWatched as _;
        /// Helper macro to spawn a task for a single valve, capturing `time` and `folder`.
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
            pressurization_vent_valve,
            fuel_pressurization_valve,
            fuel_vent_valve,
            oxidizer_pressurization_valve,
            oxidizer_vent_valve,
            fuel_main_valve,
            oxidizer_main_valve,
        } = self;

        // Log valve observed states
        Ok([
            spawn_serializable_logger!(
                "observed_state_PRZ_VNT_VLV",
                pressurization_vent_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_FSS_PRZ_VLV",
                fuel_pressurization_valve,
                ["observed state setpoint or NaN [barg?]"],
            ),
            spawn_serializable_logger!(
                "observed_state_FSS_VNT_VLV",
                fuel_vent_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_OSS_PRZ_VLV",
                oxidizer_pressurization_valve,
                ["observed state setpoint or NaN [barg?]"],
            ),
            spawn_serializable_logger!(
                "observed_state_OSS_VNT_VLV",
                oxidizer_vent_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_FSS_MNL_VLV",
                fuel_main_valve,
                ["observed state [openness %]"],
            ),
            spawn_serializable_logger!(
                "observed_state_OSS_MNL_VLV",
                oxidizer_main_valve,
                ["observed state [openness %]"],
            ),
        ])
    }
}

impl PSSSensors {
    /// Starts logging sensors timeseries to csv files in `folder`.
    /// Used for data analysis.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    #[allow(clippy::too_many_lines)] // but enough structure to be readable
    pub async fn start_logging(
        &self,
        folder: &std::path::Path,
        time: &Watchable<lib_mission::util::time::MissionTime>,
    ) -> Result<[tokio::task::AbortHandle; 14], Box<dyn std::error::Error>> {
        use lib_mission::reactive::IntoWatched as _;
        /// Helper macro to spawn a task for a single sensor, capturing `time` and `folder`.
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
            pressurization_line_pressure,
            fuel_tank_pressure,
            fuel_tank_pressure_1,
            fuel_tank_pressure_2,
            oxidizer_tank_pressure,
            oxidizer_tank_pressure_1,
            oxidizer_tank_pressure_2,
            injector_fuel_pressure,
            injector_oxidizer_pressure,
            engine_chamber_pressure,
            oxidizer_fill_fraction,
            fuel_tank_temperature,
            oxidizer_tank_temperature,
            oxidizer_runline_temperature,
        } = self;

        Ok([
            spawn_serializable_logger!(
                "sensor_PRZ_MNL_P",
                pressurization_line_pressure,
                ["pressurization_line_pressure [barg?]"],
            ),
            spawn_serializable_logger!(
                "sensor_FSS_TNK_P",
                fuel_tank_pressure,
                ["fuel_tank_pressure [barg?]"],
            ),
            spawn_serializable_logger!(
                "sensor_FSS_TNK_P1",
                fuel_tank_pressure_1,
                ["fuel_tank_pressure_1 [barg?]"],
            ),
            spawn_serializable_logger!(
                "sensor_FSS_TNK_P2",
                fuel_tank_pressure_2,
                ["fuel_tank_pressure_2 [barg?]"],
            ),
            spawn_serializable_logger!(
                "sensor_OSS_TNK_P",
                oxidizer_tank_pressure,
                ["oxidizer_tank_pressure [barg?]"],
            ),
            spawn_serializable_logger!(
                "sensor_OSS_TNK_P1",
                oxidizer_tank_pressure_1,
                ["oxidizer_tank_pressure_1 [barg?]"],
            ),
            spawn_serializable_logger!(
                "sensor_OSS_TNK_P2",
                oxidizer_tank_pressure_2,
                ["oxidizer_tank_pressure_2 [barg?]"],
            ),
            spawn_serializable_logger!(
                "sensor_FSS_INJ_P",
                injector_fuel_pressure,
                ["injector_fuel_pressure [barg?]"],
            ),
            spawn_serializable_logger!(
                "sensor_OSS_INJ_P",
                injector_oxidizer_pressure,
                ["injector_oxidizer_pressure [barg?]"],
            ),
            spawn_serializable_logger!(
                "sensor_ENG_CC_P",
                engine_chamber_pressure,
                ["engine_chamber_pressure [barg?]"],
            ),
            spawn_serializable_logger!(
                "sensor_OSS_TNK_LVL",
                oxidizer_fill_fraction,
                ["oxidizer_fill_fraction [%?]"],
            ),
            spawn_serializable_logger!(
                "sensor_FSS_TNK_T",
                fuel_tank_temperature,
                ["fuel_tank_temperature [°C?]"],
            ),
            spawn_serializable_logger!(
                "sensor_OSS_TNK_T",
                oxidizer_tank_temperature,
                ["oxidizer_tank_temperature [°C?]"],
            ),
            spawn_serializable_logger!(
                "sensor_OSS_RNL_T",
                oxidizer_runline_temperature,
                ["oxidizer_runline_temperature [°C?]"],
            ),
        ])
    }
}
