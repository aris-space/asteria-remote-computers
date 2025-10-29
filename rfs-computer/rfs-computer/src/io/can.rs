use lib_mission::can::{ConsumerRegistry, read_can, run_can_transmit_actor};
use lib_mission::reactive::IntoWatched;
use tokio::task::AbortHandle;
use watchable::Watchable;

use crate::config::CANConfig;

use super::input::InputTree;
use super::output::OutputTree;

mod input_drivers;
mod output_drivers;

/// Initializes CAN and its drivers
pub fn init_can_drivers(
    config: &CANConfig,
    input_tree: &InputTree,
    output_tree: &OutputTree,
) -> Result<Vec<AbortHandle>, Box<dyn std::error::Error>> {
    // Setup actual receiver on linux
    #[cfg(target_os = "linux")]
    let (receiver, transmitter) = {
        use socketcan::Socket;
        use std::os::fd::AsFd;

        let sock_adr = &config.socket_address;
        // Duplicating the can socket descriptor, one used for receiving, one for transmitting.
        // Pretty certain this is fine, even for properly automatically closing the socket on shutdown.
        let sock_rx = socketcan::CanFdSocket::open(sock_adr)?;
        let sock_tx = socketcan::CanFdSocket::from(sock_rx.as_fd().try_clone_to_owned()?);
        tracing::info!("Opened CAN socket {sock_adr}");
        (sock_rx, sock_tx)
    };
    // And a dummy receiver which never returns on other platforms
    #[cfg(not(target_os = "linux"))]
    #[allow(unreachable_code)] // intentional
    let (receiver, transmitter) = {
        let _ = config;
        tracing::warn!("Will not attempt connect to CAN on non-linux!");
        (lib_mission::can::DummyCan, lib_mission::can::DummyCan)
    };

    // Initialize the driver tasks
    let mut consumers = ConsumerRegistry::new();
    let input_handles = init_input_drivers(input_tree, output_tree, &mut consumers);
    let (tx, rx1) = std::sync::mpsc::channel();
    let (tx1, rx) = std::sync::mpsc::channel();

    let output_handles = init_output_drivers(output_tree, input_tree, &tx);

    // Spawn the receiving thread
    std::thread::spawn(move || {
        let mut receiver = receiver;
        read_can(&mut receiver, &consumers);
    });
    // Spawn the transmitting thread
    let can_gate = input_tree.is_can_connected.clone();
    std::thread::spawn(move || channel_gate(tx1, rx1, can_gate));
    std::thread::spawn(move || {
        let mut transmitter = transmitter;
        run_can_transmit_actor(&mut transmitter, rx);
    });

    Ok([input_handles, output_handles].concat())
}

/// Initializes all the CAN drivers writing to the input tree by spawning tasks to consume CAN messages.
/// The ECU drivers also write to the output tree.
fn init_input_drivers(
    input_tree: &InputTree,
    output_tree: &OutputTree,
    consumers: &mut ConsumerRegistry,
) -> Vec<AbortHandle> {
    let InputTree {
        time,
        is_can_connected,
        rocket_health,
        flight_computer_status,
        pss_valves,
        pss_sensors,
        ..
    } = input_tree;
    let mut handles = vec![
        input_drivers::spawn_and_register_rocket_health_driver(
            consumers,
            rocket_health.clone(),
            flight_computer_status.clone(),
            is_can_connected.clone(),
            time.watched(),
        ),
        input_drivers::spawn_and_register_ecu_igniter_valve_control(
            consumers,
            output_tree.is_software_armed.clone(),
            output_tree.rfs_actuators.h2_igniter_valve.clone(),
            output_tree.rfs_actuators.gox_igniter_valve.clone(),
            output_tree.rfs_actuators.h2_purge_valve.clone(),
            output_tree.rfs_actuators.gox_purge_valve.clone(),
            output_tree.rfs_actuators.igniter_spark.clone(),
        ),
    ];
    handles.extend(
        pss_drivers::can_input_drivers::register_and_spawn_input_drivers(
            pss_valves,
            pss_sensors,
            consumers,
        ),
    );
    handles
}

/// Initializes all the CAN drivers reading from the output tree by spawning tasks to produce CAN messages.
/// Also reads some stuff from the serial input tree and forwards it to CAN.
fn init_output_drivers(
    output_tree: &OutputTree,
    input_tree: &InputTree,
    tx: &std::sync::mpsc::Sender<hermes_can::messages::Message>,
) -> Vec<AbortHandle> {
    let OutputTree {
        pss_valves,
        is_software_armed,
        ..
    } = output_tree;
    // Register the emitters
    let mut handles = vec![
        output_drivers::spawn_rfsc_status_driver(
            pss_valves.watch_active(),
            is_software_armed.watched(),
            tx.clone(),
        ),
        output_drivers::spawn_rfsc_debug_driver(crate::build_info::THIS_BUILD, tx.clone()),
        output_drivers::spawn_ecu_igniter_pressure_driver(
            input_tree
                .serial_pressure_sensors
                .igniter
                .pressure_value_1
                .watched(),
            tx.clone(),
        ),
        output_drivers::spawn_ecu_igniter_temperature_driver(
            input_tree.uremote_sensors.igniter_temperature.watched(),
            tx.clone(),
        ),
        output_drivers::spawn_ecu_valve_state_driver(
            input_tree.uremote_actuators.h2_igniter_valve.watched(),
            input_tree.uremote_actuators.gox_igniter_valve.watched(),
            input_tree.uremote_actuators.h2_purge_valve.watched(),
            input_tree.uremote_actuators.gox_purge_valve.watched(),
            input_tree.uremote_actuators.igniter_spark.watched(),
            tx.clone(),
        ),
    ];
    handles.extend(pss_valves.spawn_output_drivers_rfs(tx));
    handles
}

/// A function which forwards messages in a channel, but only if the `gate` is `true`.
/// Can be used as a spawned function, to only send CAN messages if the CAN bus is up, resulting in less errors.
#[allow(clippy::needless_pass_by_value)] // to be used in `move` closure
#[tracing::instrument(level = "TRACE", skip_all)]
fn channel_gate<T>(
    tx: std::sync::mpsc::Sender<T>,
    rx: std::sync::mpsc::Receiver<T>,
    gate: Watchable<bool>,
) {
    for msg in rx {
        if gate.get() {
            if let Err(_e) = tx.send(msg) {
                tracing::warn!("failed to forward msg, channel was closed");
            }
        } else {
            tracing::trace!("dropping msg");
        }
    }
}
