use hermes_can::messages::board_status::ValveState;
use hermes_can::{CanMessage as _, messages};
use lib_mission::actuator_control::Energizable;
use lib_mission::actuator_control::valves::{NormallyClosedValve, NormallyOpenValve};
use lib_mission::can::ConsumerRegistry;
use lib_mission::util::types::{BarG, Celsius, Percent};
use tokio::task::AbortHandle;
use tracing::Instrument as _;
use watchable::Watchable;

use crate::input::{PSSSensors, PSSValves};
use crate::types::DPRValve;

/// Spawns the CAN drivers listening to the bus and setting input tree values.
pub fn register_and_spawn_input_drivers(
    actuators: &PSSValves,
    sensors: &PSSSensors,
    registry: &mut ConsumerRegistry,
) -> [AbortHandle; 3] {
    let PSSValves {
        pressurization_vent_valve,
        fuel_pressurization_valve,
        fuel_vent_valve,
        oxidizer_pressurization_valve,
        oxidizer_vent_valve,
        fuel_main_valve,
        oxidizer_main_valve,
    } = actuators.clone();
    let PSSSensors {
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
    } = sensors.clone();
    [
        spawn_and_register_oxidizer_control_board_driver(
            registry,
            oxidizer_pressurization_valve,
            oxidizer_vent_valve,
            oxidizer_tank_pressure,
            oxidizer_tank_pressure_1,
            oxidizer_tank_pressure_2,
            oxidizer_fill_fraction,
        ),
        spawn_and_register_fuel_control_board_driver(
            registry,
            fuel_pressurization_valve,
            pressurization_vent_valve,
            fuel_vent_valve,
            pressurization_line_pressure,
            fuel_tank_pressure,
            fuel_tank_pressure_1,
            fuel_tank_pressure_2,
            fuel_tank_temperature,
        ),
        spawn_and_register_engine_control_board_driver(
            registry,
            fuel_main_valve,
            oxidizer_main_valve,
            injector_fuel_pressure,
            injector_oxidizer_pressure,
            engine_chamber_pressure,
            oxidizer_tank_temperature,
            oxidizer_runline_temperature,
        ),
    ]
}

/// Listens for messages from the Oxidizer Control Board.
fn spawn_and_register_oxidizer_control_board_driver(
    registry: &mut ConsumerRegistry,
    oxidizer_pressurization_valve: Watchable<DPRValve>,
    oxidizer_vent_valve: Watchable<NormallyClosedValve>,
    oxidizer_tank_pressure: Watchable<BarG>,
    oxidizer_tank_pressure_1: Watchable<BarG>,
    oxidizer_tank_pressure_2: Watchable<BarG>,
    oxidizer_fill_fraction: Watchable<Percent>,
) -> AbortHandle {
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
    if let Err(()) = registry.register(
        messages::board_status::OxidizerControlBoardValveStates::ID,
        tx.clone(),
    ) {
        tracing::error!("kicking out other consumer");
    }
    if let Err(()) = registry.register(messages::sensor_data::OxidizerTankLevel::ID, tx.clone()) {
        tracing::error!("kicking out other consumer");
    }
    if let Err(()) = registry.register(messages::sensor_data::OxidizerTankPressure::ID, tx.clone())
    {
        tracing::error!("kicking out other consumer");
    }

    tokio::spawn(
        async move {
            while let Some(msg) = rx.recv().await {
                // don't care about the old values, use `let _ = _.update();
                match msg {
                    messages::Message::OxidizerTankPressure(p) => {
                        let _ = oxidizer_tank_pressure.update(BarG(p.oss_tnk_p_filtered));
                        let _ = oxidizer_tank_pressure_1.update(BarG(p.oss_tnk_p1));
                        let _ = oxidizer_tank_pressure_2.update(BarG(p.oss_tnk_p2));
                    }
                    messages::Message::OxidizerTankLevel(l) => {
                        let _ = oxidizer_fill_fraction.update(Percent(l.fill_fraction));
                    }
                    messages::Message::OxidizerControlBoardValveStates(state) => {
                        let _ =
                            oxidizer_vent_valve.update(conv_from_valve_state(state.oss_vnt_vlv));
                        let _ = oxidizer_pressurization_valve.update(state.oss_dpr.into());
                    }
                    _ => tracing::warn!("Received message it hasn't subscribed to"),
                }
            }
            tracing::warn!("Shutting down");
        }
        .instrument(tracing::debug_span!("drivers::OxidizerControlBoard")),
    )
    .abort_handle()
}

/// Listens for messages from the Fuel Control Board.
#[allow(clippy::too_many_arguments)] // Agreed, whatever
fn spawn_and_register_fuel_control_board_driver(
    registry: &mut ConsumerRegistry,
    fuel_pressurization_valve: Watchable<DPRValve>,
    pressurization_vent_valve: Watchable<NormallyOpenValve>,
    fuel_vent_valve: Watchable<NormallyOpenValve>,
    pressurization_line_pressure: Watchable<BarG>,
    fuel_tank_pressure: Watchable<BarG>,
    fuel_tank_pressure_1: Watchable<BarG>,
    fuel_tank_pressure_2: Watchable<BarG>,
    fuel_tank_temperature: Watchable<Celsius>,
) -> AbortHandle {
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
    if let Err(()) = registry.register(
        messages::board_status::FuelControlBoardValveStates::ID,
        tx.clone(),
    ) {
        tracing::error!("kicking out other consumer");
    }
    if let Err(()) = registry.register(messages::sensor_data::FuelTankTemperature::ID, tx.clone()) {
        tracing::error!("kicking out other consumer");
    }
    if let Err(()) = registry.register(
        messages::sensor_data::PressurizationLinePressure::ID,
        tx.clone(),
    ) {
        tracing::error!("kicking out other consumer");
    }
    if let Err(()) = registry.register(messages::sensor_data::FuelTankPressure::ID, tx.clone()) {
        tracing::error!("kicking out other consumer");
    }

    tokio::spawn(
        async move {
            while let Some(msg) = rx.recv().await {
                // don't care about the old values, use `let _ = _.update();
                match msg {
                    messages::Message::FuelControlBoardValveStates(state) => {
                        let _ = fuel_vent_valve.update(conv_from_valve_state(state.fss_vnt_vlv));
                        let _ = pressurization_vent_valve
                            .update(conv_from_valve_state(state.prz_vnt_vlv));
                        let _ = fuel_pressurization_valve.update(state.fss_dpr.into());
                    }
                    messages::Message::FuelTankTemperature(t) => {
                        let _ = fuel_tank_temperature.update(Celsius(t.fss_tnk_t));
                    }
                    messages::Message::PressurizationLinePressure(p) => {
                        let _ = pressurization_line_pressure.update(BarG(p.prz_mnl_p));
                    }
                    messages::Message::FuelTankPressure(p) => {
                        let _ = fuel_tank_pressure.update(BarG(p.fss_tnk_p_filtered));
                        let _ = fuel_tank_pressure_1.update(BarG(p.fss_tnk_p1));
                        let _ = fuel_tank_pressure_2.update(BarG(p.fss_tnk_p2));
                    }
                    _ => tracing::warn!("Received message it hasn't subscribed to"),
                }
            }
            tracing::warn!("Shutting down");
        }
        .instrument(tracing::debug_span!("drivers::FuelControlBoard")),
    )
    .abort_handle()
}

/// Listens for messages from the Engine Control Board.
#[allow(clippy::too_many_arguments)] // agreed, whatever
fn spawn_and_register_engine_control_board_driver(
    registry: &mut ConsumerRegistry,
    fuel_main_valve: Watchable<NormallyClosedValve>,
    oxidizer_main_valve: Watchable<NormallyClosedValve>,
    injector_fuel_pressure: Watchable<BarG>,
    injector_oxidizer_pressure: Watchable<BarG>,
    engine_chamber_pressure: Watchable<BarG>,
    oxidizer_tank_temperature: Watchable<Celsius>,
    oxidizer_runline_temperature: Watchable<Celsius>,
) -> AbortHandle {
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
    for id in [
        messages::board_status::EngineControlBoardValveStates::ID,
        messages::sensor_data::EnginePressure::ID,
        messages::sensor_data::EngineBayTemperature::ID,
    ] {
        if let Err(()) = registry.register(id, tx.clone()) {
            tracing::error!("kicking out other consumer");
        }
    }

    tokio::spawn(
        async move {
            while let Some(msg) = rx.recv().await {
                // don't care about the old values, use `let _ = _.update();
                match msg {
                    messages::Message::EngineControlBoardValveStates(state) => {
                        let _ = fuel_main_valve.update(conv_from_valve_state(state.fss_mnl_vlv));
                        let _ =
                            oxidizer_main_valve.update(conv_from_valve_state(state.oss_mnl_vlv));
                    }
                    messages::Message::EnginePressure(pressures) => {
                        let _ = engine_chamber_pressure.update(BarG(pressures.eng_cc_p));
                        let _ = injector_fuel_pressure.update(BarG(pressures.fss_inj_p));
                        let _ = injector_oxidizer_pressure.update(BarG(pressures.oss_inj_p));
                    }
                    messages::Message::EngineBayTemperature(temps) => {
                        let _ = oxidizer_tank_temperature.update(Celsius(temps.oss_tnk_t));
                        let _ = oxidizer_runline_temperature.update(Celsius(temps.oss_rnl_t));
                    }
                    _ => tracing::warn!("Received message it hasn't subscribed to"),
                }
            }
            tracing::warn!("Shutting down");
        }
        .instrument(tracing::debug_span!("drivers::EngineControlBoard")),
    )
    .abort_handle()
}

#[allow(clippy::needless_pass_by_value)] // `ValveState` will be `Copy`
fn conv_from_valve_state<T: Energizable>(valve_state: ValveState) -> T {
    T::from_active(matches!(valve_state, ValveState::Active))
}
