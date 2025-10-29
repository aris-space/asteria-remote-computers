//! Sets up telemetry over Mavlink.

use std::sync::Arc;
use std::time::Duration;

use crate::config::{DownLinkRateConfig, TelemetryConfig};
use crate::io::input::{InputTree, RawUnprocessedData};
use crate::io::output::OutputTree;

use lib_mission::reactive::{IntoWatched as _, Repeatedly, Watched as _, WatchedExt as _};
use lib_mission::state_machine::StateMachine;
use lib_mission::telemetry::{
    downlink_actor, hermes_mavlink, read_uplink, spawn_telemetry_streamer,
};
use lib_mission::util::time::MissionTime;
use mavlink_core::MavConnection;
use tokio::task::AbortHandle;

/// Sets up the telemetry connection
///
/// Returns the handles to tokio tasks spawned. Note that this also spawns two OS threads which aren't included.
pub fn init_telemetry(
    config: &TelemetryConfig,
    input_tree: &InputTree,
    output_tree: &OutputTree,
    state_machine: &StateMachine<InputTree, OutputTree, ()>,
) -> Result<Vec<AbortHandle>, Box<dyn std::error::Error>> {
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
        let actuator_watchable = input_tree.actuator_command.clone();
        let health_watchable = input_tree.telemetry_health.clone();
        let clock = input_tree.time.clone();
        let states = state_machine.state_names();
        std::thread::spawn(move || {
            read_uplink(
                conn.as_ref(),
                &command_watchable,
                &actuator_watchable,
                &health_watchable,
                &clock.watched(),
                &states,
                hermes_mavlink::RemoteSystem::ROCKET,
            );
        });
    }
    // Spawn downlink actor
    let mut sender_handles = vec![];
    {
        let conn = conn.clone();
        let (tx, rx) = std::sync::mpsc::channel::<hermes_mavlink::MavMessage>();
        std::thread::spawn(move || {
            downlink_actor(conn.as_ref(), rx, hermes_mavlink::RemoteSystem::ROCKET);
        });
        sender_handles.extend(start_telemetry_streaming(
            input_tree,
            output_tree,
            &tx,
            &config.downlink_rates,
        ));
        sender_handles.push(state_machine.start_streaming_state(
            tx,
            input_tree.time.watched(),
            config.downlink_rates.state_hz,
        ));
    }

    Ok(sender_handles)
}

/// Spawns tokio tasks which send mavlink messages periodically to the channel.
#[allow(clippy::too_many_lines)] // Agreed, whatever
fn start_telemetry_streaming(
    input_tree: &InputTree,
    output_tree: &OutputTree,
    conn: &std::sync::mpsc::Sender<hermes_mavlink::MavMessage>,
    rates: &DownLinkRateConfig,
) -> Vec<AbortHandle> {
    let kinematics_period = Duration::from_secs_f32(1. / rates.kinematics_hz);
    let av_stack_data_period = Duration::from_secs_f32(1. / rates.av_stack_data_hz);
    let steering_period = Duration::from_secs_f32(1. / rates.steering_data_hz);
    let status_period = Duration::from_secs_f32(1. / rates.state_hz);

    let InputTree {
        time,
        transition_command: _, // not streamed down to mission control
        actuator_command: _,   // not streamed down to mission control
        telemetry_health,
        boards_health,
        rfs_computer_status,
        recovery_armed,
        ecu_armed,
        raw_unprocessed_data:
            RawUnprocessedData {
                environmental_data,
                rail_status_24v,
                rail_status_5v,
            },
        motors_position,
        cats_separation,
        cats_deployment,
        location,
        velocity,
        acceleration,
        orientation,
        pss_valves,
        pss_sensors,
        ecu_firing_state: _, // not streamed down, part of the state machine
        fuel_pressurization_state: _, // not streamed down, part of the state machine
        oxidizer_pressurization_state: _, // not streamed down, part of the state machine
    } = input_tree.clone();
    let OutputTree {
        recovery_actuators_armed,
        ..
    } = output_tree.clone();
    let pss_active = output_tree.valves.watch_active();

    let mut handles = vec![
        // Status/Health
        spawn_telemetry_streamer(
            Repeatedly::new(time.watched(), status_period).map(move |time| {
                let (rfs_status, rfs_time) = rfs_computer_status.get();
                hermes_mavlink::MavMessage::FC_STATUS(hermes_mavlink::FC_STATUS_DATA {
                    time_unix_ms: time.as_ms_timestamp(),
                    recovery_armed: conv_armed(&recovery_armed.get()),
                    ecu_armed: conv_armed(&ecu_armed.get()),
                    software_armed: armed(
                        recovery_actuators_armed.get()
                            == crate::io::types::RecoveryArmingState::Armed,
                    ),
                    rfs_armed: conv_armed(&rfs_status.is_ignition_armed),
                    has_pss_control: pss_active.current().into(),
                    rfs_has_pss_control: rfs_status.has_pss_control.into(),
                    cats_sep_signal: (cats_separation.get()
                        == crate::io::types::SeparationState::Separated)
                        .into(),
                    cats_depl_signal: (cats_deployment.get()
                        == crate::io::types::MainDeploymentState::Deployed)
                        .into(),
                    time_since_telemetry_msg: conv_board_health(
                        time,
                        Some(telemetry_health.get().time_of_last_msg),
                    ),
                    time_since_rfs_msg: ((time.as_ms_timestamp() - rfs_time.as_ms_timestamp())
                        / 100)
                        .clamp(0, 65535) as u16,
                    time_since_sc_msg: conv_board_health(time, boards_health.sensor_carrier.get()),
                    time_since_pwb_msg: conv_board_health(time, boards_health.power_board.get()),
                    time_since_rec_msg: conv_board_health(time, boards_health.recovery_board.get()),
                    time_since_fcb_msg: conv_board_health(
                        time,
                        boards_health.fuel_control_board.get(),
                    ),
                    time_since_ocb_msg: conv_board_health(
                        time,
                        boards_health.oxidizer_control_board.get(),
                    ),
                    time_since_ecb_msg: conv_board_health(
                        time,
                        boards_health.engine_control_board.get(),
                    ),
                    log: [0; 20], // TODO: store in output tree
                })
            }),
            conn.clone(),
        ),
        // Location
        spawn_telemetry_streamer(
            Repeatedly::new(time.watched(), kinematics_period).map(move |time| {
                let value = location.get();
                hermes_mavlink::MavMessage::LOCATION(hermes_mavlink::LOCATION_DATA {
                    time_unix_ms: time.as_ms_timestamp(),
                    location_latitude: value.latitude,
                    location_longitude: value.longitude,
                    location_altitude: value.altitude,
                    horizontal_accuracy: value.horizontal_accuracy,
                    vertical_accuracy: value.vertical_accuracy,
                })
            }),
            conn.clone(),
        ),
        // Velocity
        spawn_telemetry_streamer(
            Repeatedly::new(time.watched(), kinematics_period).map(move |time| {
                let value = velocity.get();
                hermes_mavlink::MavMessage::VELOCITY(hermes_mavlink::VELOCITY_DATA {
                    time_unix_ms: time.as_ms_timestamp(),
                    velocity_north: value.north,
                    velocity_east: value.east,
                    velocity_down: value.down,
                })
            }),
            conn.clone(),
        ),
        // Acceleration
        spawn_telemetry_streamer(
            Repeatedly::new(time.watched(), kinematics_period).map(move |time| {
                let value = acceleration.get();
                hermes_mavlink::MavMessage::ACCELERATION(hermes_mavlink::ACCELERATION_DATA {
                    time_unix_ms: time.as_ms_timestamp(),
                    acceleration_north: value.north,
                    acceleration_east: value.east,
                    acceleration_down: value.down,
                    acceleration_x: value.x,
                    acceleration_y: value.y,
                    acceleration_z: value.z,
                })
            }),
            conn.clone(),
        ),
        // Orientation
        spawn_telemetry_streamer(
            Repeatedly::new(time.watched(), kinematics_period).map(move |time| {
                let value = orientation.get();
                hermes_mavlink::MavMessage::ORIENTATION(hermes_mavlink::ORIENTATION_DATA {
                    time_unix_ms: time.as_ms_timestamp(),
                    orientation_w: value.orientation_w,
                    orientation_x: value.orientation_x,
                    orientation_y: value.orientation_y,
                    orientation_z: value.orientation_z,
                })
            }),
            conn.clone(),
        ),
        // Env data
        spawn_telemetry_streamer(
            Repeatedly::new(time.watched(), av_stack_data_period).map(move |time| {
                let value = environmental_data.get();
                hermes_mavlink::MavMessage::ENVIRONMENTAL_DATA(
                    hermes_mavlink::ENVIRONMENTAL_DATA_DATA {
                        time_unix_ms: time.as_ms_timestamp(),
                        temperature: value.temperature,
                        humidity: value.humidity,
                        pressure: value.pressure,
                    },
                )
            }),
            conn.clone(),
        ),
        spawn_telemetry_streamer(
            Repeatedly::new(time.watched(), av_stack_data_period).map(move |time| {
                let value = rail_status_24v.get();
                hermes_mavlink::MavMessage::RAIL_STATUS_24V(hermes_mavlink::RAIL_STATUS_24V_DATA {
                    time_unix_ms: time.as_ms_timestamp(),
                    voltage: value.data.voltage,
                    current: value.data.current,
                    power: value.data.power,
                })
            }),
            conn.clone(),
        ),
        spawn_telemetry_streamer(
            Repeatedly::new(time.watched(), av_stack_data_period).map(move |time| {
                let value = rail_status_5v.get();
                hermes_mavlink::MavMessage::RAIL_STATUS_5V(hermes_mavlink::RAIL_STATUS_5V_DATA {
                    time_unix_ms: time.as_ms_timestamp(),
                    voltage: value.data.voltage,
                    current: value.data.current,
                    power: value.data.power,
                })
            }),
            conn.clone(),
        ),
        // Steering data
        spawn_telemetry_streamer(
            Repeatedly::new(time.watched(), steering_period).map(move |time| {
                let value = motors_position.get();
                hermes_mavlink::MavMessage::STEERING_ACTUAL_POSITIONS(
                    // We never turn more 7 revs per direction
                    #[allow(clippy::cast_possible_truncation)]
                    hermes_mavlink::STEERING_ACTUAL_POSITIONS_DATA {
                        time_unix_ms: time.as_ms_timestamp(),
                        left_pos: value.left_motor_steps as i16,
                        right_pos: value.right_motor_steps as i16,
                    },
                )
            }),
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
        &&time, // yeah double refs are a bit sad, but makes clippy happy
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

/// Returns the number of deciseconds from now to `last_healthy` or saturates at 255.
fn conv_board_health(now: MissionTime, last_healthy: Option<MissionTime>) -> u8 {
    last_healthy.map_or(255, |t| {
        ((now.as_ms_timestamp() - t.as_ms_timestamp()) / 100).clamp(0, 255) as u8
    })
}
