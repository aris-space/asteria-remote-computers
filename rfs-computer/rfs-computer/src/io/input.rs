//! Contains [`InputTree`]

use hermes_can::messages::board_status::FlightComputerStatus;
use lib_mission::telemetry::TelemetryHealth;
use lib_mission::util::time::MissionTime;
use lib_mission::util::types::{BarG, Celsius};
use lib_mission::{actuator_control, transition};
use pss_drivers::input::{PSSSensors, PSSValves};
use pss_drivers::types::ThrottleableValve;
use uremote_electronics::input as uremote;
use watchable::Watchable;

use super::output::ActuatorName;

/// Holds all information we gather from the rest of the system and environment.
#[derive(Debug, Default, Clone)]
pub struct InputTree {
    /// The current time, as as unix timestamp wrapper.
    pub time: Watchable<MissionTime>,
    /// The newest transition command from the ground station.
    pub transition_command: Watchable<transition::Command>,
    /// The newest actuator command from the ground station.
    pub actuator_command: Watchable<Option<actuator_control::Command<ActuatorName>>>,
    /// Statistics about the health of the telemetry uplink.
    pub telemetry_health: Watchable<TelemetryHealth>,
    /// `true` if can is connected. `false` by default and is set if messages are received.
    pub is_can_connected: Watchable<bool>,
    /// The processed health report for the rocket boards.
    pub rocket_health: BoardsHealthyAsOf,
    /// The last received status of the flight computer.
    pub flight_computer_status: Watchable<FlightComputerStatus>,
    /// The sensors connected to the uremote.
    pub uremote_sensors: uremote::Sensors,
    /// The state of the actuators connected to the uremote.
    pub uremote_actuators: uremote::Actuators,
    /// The state of the pressure sensors that communicate on the serial bus.
    pub serial_pressure_sensors: PressureSensors,
    /// The current state for the lox fill valve (`lox_fll_vlv`).
    pub lox_fill_valve: Watchable<ThrottleableValve>,
    /// The valve states of the PSS in the rocket.
    pub pss_valves: PSSValves,
    /// The sensor readings from the PSS in the rocket.
    pub pss_sensors: PSSSensors,
}

#[derive(Debug, Default, Clone)]
pub struct PressureSensors {
    /// `R-P-N-1`
    pub n2_pre_booster: SensorValues,
    /// `R-P-N-2`
    pub h2_purge: SensorValues,
    /// `R-P-N-3`
    pub n2_post_booster: SensorValues,
    /// `R-P-L-1`
    pub lox: SensorValues,
    /// `R-P-P-1`
    pub purge: SensorValues,
    /// `R-P-P-2`
    pub pneumatic: SensorValues,
    // Igniter system:
    /// `I-P-G-1`
    pub gox: SensorValues,
    /// `I-P-H-1`
    pub h2: SensorValues,
    /// `I-P-I-1`
    pub igniter: SensorValues,
}

#[derive(Debug, Default, Clone)]
pub struct SensorValues {
    pub pressure_value_1: Watchable<BarG>,
    pub temperature_value_1: Watchable<Celsius>,
    pub pressure_value_2: Watchable<BarG>,
    pub temperature_value_2: Watchable<Celsius>,
}

/// The processed health report for the rocket boards.
///
/// If `Some(t)` the board was last observed to be healthy, which happend at `t`.
/// If `None` the board was last observed unhealthy or not yet observed.
#[derive(Debug, Default, Clone)]
pub struct BoardsHealthyAsOf {
    pub flight_computer: Watchable<Option<MissionTime>>,
    pub fuel_control_board: Watchable<Option<MissionTime>>,
    pub oxidizer_control_board: Watchable<Option<MissionTime>>,
    pub engine_control_board: Watchable<Option<MissionTime>>,
}

/// Allow common code to read the ground station transition command.
impl transition::GetTransitionCommand for InputTree {
    fn transition_cmd_watcher(
        &self,
    ) -> impl lib_mission::reactive::IntoWatched<transition::Command> + Send {
        &self.transition_command
    }
}
