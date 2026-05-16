use datatypes::{
    actuator::{DPRValve, NormallyClosedValve, NormallyOpenValve},
    units::{BarG, Celsius},
};
use messages::datapoints::dp_engine_control_board::Message as ECBMessage;
use messages::datapoints::dp_fuel_control_board::Message as FCBMessage;
use messages::datapoints::dp_oxidizer_control_board::Message as OCBMessage;

use io_utils::{Broadcast, Collector};
use messages::Message;
use watchable::Watchable;

/// The feedback for the valves in the propellant supply system.
#[derive(Debug, Default, Clone, Broadcast, Collector)]
#[broadcast(message_type = "Message")]
#[collector(
    message_type = "Message",
    update_expr = "let _ = #field.update(#value);"
)]
pub struct PSSValves {
    /// The current state for the Pressurization Vent Valve (`prz_copv_vnt`).
    #[datapoint(
        pattern = "Message::FuelControlBoard(FCBMessage::PressurizationVentValveState(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub pressurization_vent_valve: Watchable<NormallyOpenValve>,
    /// The current state for the Fuel Pressurization Valve (DPR) (`prz_fss_dpr`).
    #[datapoint(pattern = "Message::FuelControlBoard(FCBMessage::FuelDprValveState(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub fuel_pressurization_valve: Watchable<DPRValve>,
    /// The current state for the Fuel Vent Valve (`fss_tnk_vnt`).
    #[datapoint(pattern = "Message::FuelControlBoard(FCBMessage::FuelVentValveState(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub fuel_vent_valve: Watchable<NormallyOpenValve>,
    /// The current state for the Oxidizer Pressurization Valve (DPR) (`prz_oss_dpr`).
    #[datapoint(
        pattern = "Message::OxidizerControlBoard(OCBMessage::OxidizerDprValveState(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub oxidizer_pressurization_valve: Watchable<DPRValve>,
    /// The current state for the Oxidizer Vent Valve (`oss_tnk_vnt`).
    /// Note that this should be opened in an abort, despite being normally closed.
    #[datapoint(
        pattern = "Message::OxidizerControlBoard(OCBMessage::OxidizerVentValveState(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub oxidizer_vent_valve: Watchable<NormallyClosedValve>,
    /// The current state for the Fuel Main Valve (`fss_ml_vlv`).
    #[datapoint(pattern = "Message::EngineControlBoard(ECBMessage::FuelMainValveState(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub fuel_main_valve: Watchable<NormallyClosedValve>,
    /// The current state for the Oxidizer Main Valve (`oss_ml_vlv`).
    #[datapoint(
        pattern = "Message::EngineControlBoard(ECBMessage::OxidizerMainValveState(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub oxidizer_main_valve: Watchable<NormallyClosedValve>,
}

#[derive(Debug, Default, Clone, Broadcast, Collector)]
#[broadcast(message_type = "Message")]
#[collector(
    message_type = "Message",
    update_expr = "let _ = #field.update(#value);"
)]
pub struct PSSSensors {
    /// Pressurization Line Pressure Sensor in barg `prz_copv_p`.
    #[datapoint(
        pattern = "Message::FuelControlBoard(FCBMessage::PressurizationLinePressure(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub pressurization_line_pressure: Watchable<BarG>,
    /// Fuel Tank Pressure Sensor 1 in barg `fss_tnk_p1`.
    #[datapoint(
        pattern = "Message::FuelControlBoard(FCBMessage::FuelTankPressureSensor1(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub fuel_tank_pressure1: Watchable<BarG>,
    /// Fuel Tank Pressure Sensor 2 in barg `fss_tnk_p2`.
    #[datapoint(
        pattern = "Message::FuelControlBoard(FCBMessage::FuelTankPressureSensor2(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub fuel_tank_pressure2: Watchable<BarG>,
    /// Fuel Tank Pressure Filtered in barg `fss_tnk_p_flt`.
    #[datapoint(
        pattern = "Message::FuelControlBoard(FCBMessage::FuelTankPressureFiltered(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub fuel_tank_pressure_filtered: Watchable<BarG>,

    /// Oxidizer Tank Pressure Sensor 1 in barg `oss_tnk_p1`.
    #[datapoint(
        pattern = "Message::OxidizerControlBoard(OCBMessage::OxidizerTankPressureSensor1(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub oxidizer_tank_pressure1: Watchable<BarG>,
    /// Oxidizer Tank Pressure Sensor 2 in barg `oss_tnk_p2`.
    #[datapoint(
        pattern = "Message::OxidizerControlBoard(OCBMessage::OxidizerTankPressureSensor2(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub oxidizer_tank_pressure2: Watchable<BarG>,
    /// Oxidizer Tank Differential Pressure Sensor in barg `oss_tnk_dps`.
    #[datapoint(
        pattern = "Message::OxidizerControlBoard(OCBMessage::OxidizerTankDifferentialPressure(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub oxidizer_tank_differential_pressure: Watchable<BarG>,

    /// Engine Chamber Pressure Sensor in barg `eng_cc_p`.
    #[datapoint(pattern = "Message::EngineControlBoard(ECBMessage::ChamberPressure(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub engine_chamber_pressure: Watchable<BarG>,
    /// Engine Injector Fuel Pressure Sensor in barg `eng_inj_p`.
    #[datapoint(pattern = "Message::EngineControlBoard(ECBMessage::InjectorFuelPressure(#value))")]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub engine_injector_fuel_pressure: Watchable<BarG>,
    /// Engine Injector Oxidizer Pressure Sensor in barg `oss_inj_p`.
    #[datapoint(
        pattern = "Message::EngineControlBoard(ECBMessage::InjectorOxidizerPressure(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub engine_injector_oxidizer_pressure: Watchable<BarG>,
    /// Engine Injector Fuel Temperature Sensor in °C `eng_inj_t`.
    #[datapoint(
        pattern = "Message::EngineControlBoard(ECBMessage::InjectorFuelTemperature(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub engine_injector_fuel_temperature: Watchable<Celsius>,
    /// Engine Oxidizer Tank Temperature Sensor in °C `oss_tnk_t`.
    #[datapoint(
        pattern = "Message::EngineControlBoard(ECBMessage::OxidizerTankTemperature(#value))"
    )]
    #[broadcast(min_freq_hz = 5., max_freq_hz = 15.)]
    pub engine_oxidizer_tank_temperature: Watchable<Celsius>,
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
                "observed_state_prz_copv_vnt",
                pressurization_vent_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_prz_fss_dpr",
                fuel_pressurization_valve,
                ["observed state setpoint or NaN [barg?]"],
            ),
            spawn_serializable_logger!(
                "observed_state_fss_tnk_vnt",
                fuel_vent_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_prz_oss_dpr",
                oxidizer_pressurization_valve,
                ["observed state setpoint or NaN [barg?]"],
            ),
            spawn_serializable_logger!(
                "observed_state_oss_tnk_vnt",
                oxidizer_vent_valve,
                ["observed state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "observed_state_fss_ml_vlv",
                fuel_main_valve,
                ["observed state [openness %]"],
            ),
            spawn_serializable_logger!(
                "observed_state_oss_ml_vlv",
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
    ) -> Result<[tokio::task::AbortHandle; 5], Box<dyn std::error::Error>> {
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
            fuel_tank_pressure1,
            fuel_tank_pressure2,
            fuel_tank_pressure_filtered,

            oxidizer_tank_pressure1,
            oxidizer_tank_pressure2,
            oxidizer_tank_differential_pressure,

            engine_injector_fuel_pressure,
            engine_injector_oxidizer_pressure,
            engine_chamber_pressure,
            engine_oxidizer_tank_temperature,
            engine_injector_fuel_temperature,
        } = self;

        Ok([
            spawn_serializable_logger!(
                "sensor_prz_copv_p",
                pressurization_line_pressure,
                ["pressurization_line_pressure [barg?]"],
            ),
            spawn_serializable_logger!(
                "sensors_fuel_tank_p",
                fuel_tank_pressure,
                [
                    "fuel_tank_pressure_sensor_1 [barg?]",
                    "fuel_tank_pressure_sensor_2 [barg?]",
                    "fuel_tank_pressure_filtered [barg?]"
                ],
            ),
            spawn_serializable_logger!(
                "sensors_oxidizer_tank_p",
                oxidizer_tank_pressure,
                [
                    "oxidizer_tank_pressure1 [barg?]",
                    "oxidizer_tank_pressure1 [barg?]",
                    "oxidizer_diff_tank_pressure [barg?]",
                ],
            ),
            spawn_serializable_logger!(
                "sensors_engine_p",
                engine_chamber_pressure,
                ["eng_cc_p [barg?]", "eng_inj_p [barg?]", "oss_inj_p [barg?]",],
            ),
            spawn_serializable_logger!(
                "sensors_engine_t",
                engine_bay_temperature,
                ["eng_inj_t [°C?]", "oss_tnk_t [°C?]"],
            ),
        ])
    }
}
