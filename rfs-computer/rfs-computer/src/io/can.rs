use lib_mission::can::{ConsumerRegistry, read_can, run_can_transmit_actor};
use lib_mission::reactive::IntoWatched;
use tokio::task::AbortHandle;

use crate::config::CANConfig;
use crate::state::State;

use super::input::InputTree;
use super::output::OutputTree;

mod output_drivers;

/// Initializes CAN and its drivers
pub fn init_can_drivers(
    config: &CANConfig,
    input_tree: &InputTree,
    _output_tree: &OutputTree,
    state_tree: &State,
    tx: &std::sync::mpsc::Sender<messages::Message>,
    rx: std::sync::mpsc::Receiver<messages::Message>,
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
    let output_handles = init_output_drivers(state_tree, input_tree, tx);
    let mut consumers = ConsumerRegistry::new().with_collector(input_tree.clone());

    // Spawn the receiving thread
    std::thread::spawn(move || {
        let mut receiver = receiver;
        read_can(&mut receiver, &mut consumers);
    });
    // Spawn the transmitting thread
    std::thread::spawn(move || {
        let mut transmitter = transmitter;
        run_can_transmit_actor(&mut transmitter, rx);
    });

    Ok(output_handles)
}

/// Initializes all the CAN drivers reading from the output tree by spawning tasks to produce CAN messages.
/// Also reads some stuff from the serial input tree and forwards it to CAN.
fn init_output_drivers(
    state_tree: &State,
    input_tree: &InputTree,
    tx: &std::sync::mpsc::Sender<messages::Message>,
) -> Vec<AbortHandle> {
    let State {
        rfs_computer_status,
        ..
    } = state_tree;
    // Register the emitters
    let handles = vec![
        output_drivers::spawn_rfsc_status_driver(rfs_computer_status.clone(), tx.clone()),
        output_drivers::spawn_rfsc_debug_driver(crate::build_info::THIS_BUILD, tx.clone()),
        output_drivers::spawn_fc_igniter_pressure_driver(
            input_tree.controllino_sensors.igniter_pressure.watched(),
            tx.clone(),
        ),
    ];
    handles
}
