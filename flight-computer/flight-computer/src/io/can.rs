use lib_mission::can::{read_can, run_can_transmit_actor, ConsumerRegistry};
use lib_mission::reactive::IntoWatched;
use lib_mission::util::time::MissionTime;
use tokio::task::AbortHandle;

use crate::config::CANConfig;
use crate::io::input::InputTree;

use super::output::OutputTree;

mod output_drivers;

/// Initializes CAN and its drivers
pub fn init_can_drivers(
    config: &CANConfig,
    input_tree: &InputTree,
    output_tree: &OutputTree,
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
    let output_handles = init_output_drivers(&input_tree.time, output_tree, tx);
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

/// Initializes all the drivers reading from the output tree by spawning tasks to produce CAN messages.
///
/// Additionally, `thrust_curve` is sent to the ECU after 5 seconds.
fn init_output_drivers<C>(
    clock: C,
    output_tree: &OutputTree,
    tx: &std::sync::mpsc::Sender<messages::Message>,
) -> Vec<AbortHandle>
where
    C: IntoWatched<MissionTime> + Copy,
    C::Watched: Send + 'static,
{
    let OutputTree {
        separate_drogue,
        deploy_main_chute,
        steering_motor_target_deflections,
        recovery_actuators_armed,
        camera_powered_on,
        valves: _,
        hash_config: _,
        fc_status,
    } = output_tree;
    // Register the emitters
    let handles = vec![
        output_drivers::spawn_flight_computer_status_driver(
            recovery_actuators_armed.watched(),
            fc_status.clone(),
            tx.clone(),
        ),
        output_drivers::spawn_flight_computer_debug_driver(
            crate::build_info::THIS_BUILD,
            tx.clone(),
        ),
        output_drivers::spawn_steering_motor_driver(
            steering_motor_target_deflections.watched(),
            clock.watched(),
            tx.clone(),
        ),
        output_drivers::spawn_recovery_arming_driver(
            recovery_actuators_armed.watched(),
            clock.watched(),
            tx.clone(),
        ),
        output_drivers::spawn_camera_driver(camera_powered_on.watched(), tx.clone()),
        output_drivers::spawn_deployment_driver(
            deploy_main_chute.watched(),
            clock.watched(),
            tx.clone(),
        ),
        output_drivers::spawn_separation_driver(
            separate_drogue.watched(),
            clock.watched(),
            tx.clone(),
        ),
    ];
    handles
}
