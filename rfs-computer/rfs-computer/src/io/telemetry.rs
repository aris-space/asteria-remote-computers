//! Sets up telemetry over Mavlink.

use std::sync::Arc;
use std::time::Duration;

use crate::config::{DownLinkRateConfig, TelemetryConfig};
use crate::io::input::InputTree;
use crate::io::output::OutputTree;

use lib_mission::actuator_control::Energizable;
use lib_mission::reactive::{IntoWatched as _, Watched};
use lib_mission::state_machine::StateMachine;
use lib_mission::telemetry::{
    downlink_actor, hermes_mavlink, read_uplink, spawn_periodic_telemetry_streamer,
};
use mavlink_core::MavConnection;
use tokio::task::AbortHandle;

/// Sets up the telemetry connection
pub fn init_telemetry(
    config: &TelemetryConfig,
    input_tree: &InputTree,
    output_tree: &OutputTree,
    state_machine: &StateMachine<InputTree, OutputTree, ()>,
) -> Result<(), Box<dyn std::error::Error>> {
    // Setup connection
    let addr = &config.mavlink_address;
    tracing::trace!("waiting for connection with {addr}");
    // The connection can be simultaneously used for sending and receiving, but only one of each.
    let conn: Arc<dyn MavConnection<hermes_mavlink::MavMessage> + Send + Sync> =
        mavlink_core::connect::<hermes_mavlink::MavMessage>(addr)?.into();
    tracing::info!(
        "connected to MavLink {v:?} at {addr}",
        v = conn.get_protocol_version()
    );

    // Spawn uplink actor
    {
        let conn = conn.clone();
        let command_watchable = input_tree.transition_command.clone();
        let actuator_cmd_watchable = input_tree.actuator_command.clone();
        let health_watchable = input_tree.telemetry_health.clone();
        let clock = input_tree.time.clone();
        let states = state_machine.state_names();
        std::thread::spawn(move || {
            read_uplink(
                conn.as_ref(),
                &command_watchable,
                &actuator_cmd_watchable,
                &health_watchable,
                &clock.watched(),
                &states,
                hermes_mavlink::RemoteSystem::REMOTE_FILLING_STATION,
            );
        });
    }
    // Spawn downlink actor
    {
        let conn = conn.clone();
        let (tx, rx) = std::sync::mpsc::channel::<hermes_mavlink::MavMessage>();
        std::thread::spawn(move || {
            downlink_actor(
                conn.as_ref(),
                rx,
                hermes_mavlink::RemoteSystem::REMOTE_FILLING_STATION,
            );
        });
        start_io_telemetry_streaming(input_tree, output_tree, &tx, &config.downlink_rates);
        state_machine.start_streaming_state(
            tx,
            input_tree.time.watched(),
            config.downlink_rates.state_hz,
        );
    }

    Ok(())
}

/// Spawns tokio tasks which send mavlink messages periodically to the channel.
// TODO: Figure out how to make this neat instead of allowing too many lines
#[allow(clippy::too_many_lines)]
fn start_io_telemetry_streaming(
    input_tree: &InputTree,
    output_tree: &OutputTree,
    conn: &std::sync::mpsc::Sender<hermes_mavlink::MavMessage>,
    rates: &DownLinkRateConfig,
) -> Vec<AbortHandle> {
    let status_period = Duration::from_secs_f32(1. / rates.state_hz); // Use the same period for status and state
    let actuator_period = Duration::from_secs_f32(1. / rates.actuator_feedback_hz);
    let temperature_period = Duration::from_secs_f32(1. / rates.temperature_sensors_hz);
    let pressure_period = Duration::from_secs_f32(1. / rates.pressure_sensors_hz);

    let InputTree {
        time,
        transition_command: _, // Isn't streamed down
        actuator_command: _,   // Isn't streamed down
        telemetry_health: _,   // Might be done further along
        is_can_connected: _,   // isn't streamed down. TODO: maybe later
        rocket_health,
        flight_computer_status,
        uremote_sensors,
        uremote_actuators,
        serial_pressure_sensors,
        pss_valves,
        pss_sensors,
        lox_fill_valve,
    } = input_tree.clone();
    let ur_acts = uremote_actuators;
    let ur_sens = uremote_sensors;
    let ser_devs_values = serial_pressure_sensors;
    let lox_fill_valve = lox_fill_valve;
    let sfw_armed = output_tree.is_software_armed.clone();
    let pss_active = output_tree.pss_valves.watch_active();
    let time1 = time.clone();
    let time2 = time.clone();
    let time3 = time.clone();
    let time4 = time.clone();
    let time5 = time.clone();

    let mut handles = vec![
        // Status
        spawn_periodic_telemetry_streamer(
            move || {
                let now_ms = time1.get().as_ms_timestamp();
                hermes_mavlink::MavMessage::RFS_STATUS(hermes_mavlink::RFS_STATUS_DATA {
                    time_unix_ms: now_ms,
                    valves_armed: armed(ur_sens.valves_armed.get()),
                    igniter_armed: armed(ur_sens.igniter_armed.get()),
                    software_armed: armed(sfw_armed.get()),
                    fc_armed: conv_armed(&flight_computer_status.get().is_software_armed),
                    has_pss_control: pss_active.current().into(),
                    fc_has_pss_control: flight_computer_status.get().has_pss_control.into(),
                    time_since_fc_msg: rocket_health.flight_computer.get().map_or(255, |t| {
                        ((now_ms - t.as_ms_timestamp()) / 100).clamp(0, 255) as u8
                    }), // deci-seconds in a byte
                    time_since_fcb_msg: rocket_health.fuel_control_board.get().map_or(255, |t| {
                        ((now_ms - t.as_ms_timestamp()) / 100).clamp(0, 255) as u8
                    }), // deci-seconds in a byte,
                    time_since_ocb_msg: rocket_health
                        .oxidizer_control_board
                        .get()
                        .map_or(255, |t| {
                            ((now_ms - t.as_ms_timestamp()) / 100).clamp(0, 255) as u8
                        }), // deci-seconds in a byte,
                    time_since_ecb_msg: rocket_health.engine_control_board.get().map_or(255, |t| {
                        ((now_ms - t.as_ms_timestamp()) / 100).clamp(0, 255) as u8
                    }), // deci-seconds in a byte,
                })
            },
            status_period,
            conn.clone(),
        ),
        // Actuator Feedback.
        // For each actuator, it reads the openness state and turns that boolean into an u8 for mavlink.
        spawn_periodic_telemetry_streamer(
            move || {
                hermes_mavlink::MavMessage::RFS_ACTUATORS_STATES(
                    hermes_mavlink::RFS_ACTUATORS_STATES_DATA {
                        time_unix_ms: time2.get().as_ms_timestamp(),
                        lox_fill_valve: {
                            let p = lox_fill_valve.get().openness_percent;
                            if p <= 100 {
                                f32::from(p) / 100.
                            } else {
                                f32::NAN
                            }
                        },
                        gox_purge_valve: ur_acts.gox_purge_valve.get().is_open().into(),
                        h2_purge_valve: ur_acts.h2_purge_valve.get().is_open().into(),
                        n2_bleed_valve: ur_acts.n2_bleed_valve.get().is_open().into(),
                        lox_purge_valve: ur_acts.lox_purge_valve.get().is_open().into(),
                        gox_bleed_valve: ur_acts.gox_bleed_valve.get().is_open().into(),
                        n2_purge_valve: ur_acts.n2_purge_valve.get().is_open().into(),
                        n2_fill_valve: ur_acts.n2_fill_valve.get().is_open().into(),
                        h2_bleed_valve: ur_acts.h2_bleed_valve.get().is_open().into(),
                        lox_bleed_valve: ur_acts.lox_bleed_valve.get().is_open().into(),
                        booster_valve: ur_acts.booster_valve.get().is_open().into(),
                        lox_decoupler_ret: ur_acts
                            .lox_decoupler_retract_valve
                            .get()
                            .is_open()
                            .into(),
                        lox_decoupler_ext: ur_acts
                            .lox_decoupler_extend_valve
                            .get()
                            .is_open()
                            .into(),
                        lox_decoupler_prox: ur_sens.lox_decoupler_proximity.get().into(),
                        n2_decoupler_ret: ur_acts.n2_decoupler_retract_valve.get().is_open().into(),
                        n2_decoupler_ext: ur_acts.n2_decoupler_extend_valve.get().is_open().into(),
                        n2_decoupler_prox: ur_sens.n2_decoupler_proximity.get().into(),
                        n2_release_valve: ur_acts.n2_release_valve.get().is_open().into(),
                        h2_igniter_valve: ur_acts.h2_igniter_valve.get().is_open().into(),
                        gox_igniter_valve: ur_acts.gox_igniter_valve.get().is_open().into(),
                        igniter_spark: ur_acts.igniter_spark.get().is_active().into(),
                    },
                )
            },
            actuator_period,
            conn.clone(),
        ),
        spawn_periodic_telemetry_streamer(
            move || {
                hermes_mavlink::MavMessage::RFS_OUTPUTS_STATES(
                    hermes_mavlink::RFS_OUTPUTS_STATES_DATA {
                        time_unix_ms: time5.get().as_ms_timestamp(),
                        rocket_power: ur_acts.rocket_power.get().into(),
                        maxon_board_power: ur_acts.maxon_board_power.get().into(),
                        fss_heating: ur_acts.fss_heating.get().into(),
                        pressure_sensor_heating: ur_acts.pressure_sensor_heating.get().into(),
                        eth_pump_power: ur_acts.eth_pump_power.get().into(),
                        custom1: ur_acts.custom1.get().into(),
                        custom2: ur_acts.custom2.get().into(),
                        custom3: ur_acts.custom3.get().into(),
                    },
                )
            },
            actuator_period,
            conn.clone(),
        ),
        // Pressure sensors
        spawn_periodic_telemetry_streamer(
            move || {
                hermes_mavlink::MavMessage::RFS_PRESSURE(hermes_mavlink::RFS_PRESSURE_DATA {
                    time_unix_ms: time3.get().as_ms_timestamp(),
                    lox: ser_devs_values.lox.pressure_value_1.get().as_f32(),
                    n2_pre_booster: ser_devs_values
                        .n2_pre_booster
                        .pressure_value_1
                        .get()
                        .as_f32(),
                    h2_purge: ser_devs_values.h2_purge.pressure_value_1.get().as_f32(),
                    n2_post_booster: ser_devs_values
                        .n2_post_booster
                        .pressure_value_1
                        .get()
                        .as_f32(),
                    purge: ser_devs_values.purge.pressure_value_1.get().as_f32(),
                    pneumatic: ser_devs_values.pneumatic.pressure_value_1.get().as_f32(),
                    h2: ser_devs_values.h2.pressure_value_1.get().as_f32(),
                    gox: ser_devs_values.gox.pressure_value_1.get().as_f32(),
                    igniter: ser_devs_values.igniter.pressure_value_1.get().as_f32(),
                })
            },
            pressure_period,
            conn.clone(),
        ),
        // Temperature sensors
        spawn_periodic_telemetry_streamer(
            move || {
                hermes_mavlink::MavMessage::RFS_TEMPERATURE(hermes_mavlink::RFS_TEMPERATURE_DATA {
                    time_unix_ms: time4.get().as_ms_timestamp(),
                    lox: ur_sens.lox_temperature.get().as_f32(),
                    igniter: ur_sens.igniter_temperature.get().as_f32(),
                    cabinet: ur_sens.cabinet_temperature.get().as_f32(),
                    ambient: ur_sens.ambient_temperature.get().as_f32(),
                })
            },
            temperature_period,
            conn.clone(),
        ),
    ];
    // PSS stuff
    handles.push(pss_valves.start_telemetry_streaming(
        conn,
        Duration::from_secs_f32(1. / rates.actuator_feedback_hz),
        time.watched(),
    ));
    handles.extend(pss_sensors.start_telemetry_streaming(
        conn,
        Duration::from_secs_f32(1. / rates.pressure_sensors_hz),
        Duration::from_secs_f32(1. / rates.temperature_sensors_hz),
        Duration::from_secs_f32(1. / rates.fill_levels_hz),
        &&time,
    ));
    handles
}

fn armed(is_armed: bool) -> hermes_mavlink::Armed {
    if is_armed {
        hermes_mavlink::Armed::ARMED
    } else {
        hermes_mavlink::Armed::DISARMED
    }
}

fn conv_armed(is_armed: &hermes_can::messages::board_status::ArmingState) -> hermes_mavlink::Armed {
    match is_armed {
        hermes_can::messages::board_status::ArmingState::Safe => hermes_mavlink::Armed::DISARMED,
        hermes_can::messages::board_status::ArmingState::Armed => hermes_mavlink::Armed::ARMED,
    }
}
