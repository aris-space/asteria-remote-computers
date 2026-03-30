//! Sets up telemetry over Mavlink.

use lib_mission::telemetry::{
    start_downlink_actor, start_uplink_actor, PeriodicLoop, RemoteSystem,
};
use std::net::UdpSocket;
use std::thread::JoinHandle;

use crate::config::TelemetryConfig;
use crate::io::input::InputTree;
use crate::io::output::OutputTree;

use io_utils::{Broadcast, NeverReturns};
use lib_mission::reactive::IntoWatched as _;
use lib_mission::state_machine::StateMachine;
use tokio::task::AbortHandle;

#[derive(Debug)]
pub struct Handles {
    pub uplink: JoinHandle<NeverReturns>,
    pub downlink: JoinHandle<()>,
    pub streamers: Vec<AbortHandle>,
}

/// Sets up the telemetry connection
///
/// Returns the handles to tokio tasks spawned. Note that this also spawns two OS threads which aren't included.
pub fn init_telemetry(
    config: &TelemetryConfig,
    input_tree: &InputTree,
    _output_tree: &OutputTree,
    state_machine: &StateMachine<InputTree, OutputTree, ()>,
) -> Result<Handles, Box<dyn std::error::Error>> {
    // Setup connection
    let socket = UdpSocket::bind("127.0.0.1:0")?;
    socket.connect(&config.mavlink_address)?;
    tracing::trace!(
        "setup UDP at {:?} to {:?}",
        socket.local_addr(),
        socket.peer_addr()
    );

    // Spawn uplink actor
    let uplink = start_uplink_actor(
        socket.try_clone()?,
        input_tree.transition_command.clone(),
        input_tree.actuator_command.clone(),
        input_tree.telemetry_health.clone(),
        input_tree.time.watched(),
        state_machine.state_names(),
        RemoteSystem::FlightComputer,
    );

    // Spawn downlink actor
    let (tx, rx) = std::sync::mpsc::channel::<messages::Message>();
    let downlink = start_downlink_actor(socket, rx, RemoteSystem::FlightComputer);

    // spawns tokio tasks which send mavlink messages periodically to the channel.
    let input_streamers = input_tree.start_broadcasting::<PeriodicLoop>(tx.clone());

    // TODO: current state and FC status

    Ok(Handles {
        uplink,
        downlink,
        streamers: input_streamers,
    })
}
