//! Sets up telemetry over Mavlink.

use std::net::{TcpListener};
use mavio::io::{StdIoReader, StdIoWriter};
use mavio::prelude::V2;
use mavio::{Endpoint, MavLinkId, Receiver, Sender};

use crate::config::{DownLinkRateConfig, TelemetryConfig};
use crate::io::input::InputTree;
use crate::io::output::OutputTree;
use messages::Message;

use lib_mission::state_machine::StateMachine;
use tokio::task::AbortHandle;
use io_utils::Broadcast;

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
    let listener = TcpListener::bind(addr)?;
    let (stream, _) = listener.accept()?;
    let sender_stream = stream.try_clone()?;
    let _receiver = Receiver::versioned(StdIoReader::new(stream), V2);
    tracing::info!("connected to MavLink at {addr}");

    // Spawn downlink actor
    let mut sender_handles = vec![];
    {
        //let conn = conn.clone();
        let (tx, rx) = std::sync::mpsc::channel::<messages::Message>();
        std::thread::spawn(move || {
            let mut sender = Sender::new(StdIoWriter::new(sender_stream));
            let endpoint = Endpoint::v2(MavLinkId::new(1, 1));
            for msg in rx {
                let frame = endpoint.next_frame(&msg).expect("must encode frame");
                sender.send(&frame).expect("must send frame");
            }
        });
        sender_handles.extend(start_telemetry_streaming(
            input_tree,
            output_tree,
            &tx,
            &config.downlink_rates,
        ));
    }

    Ok(sender_handles)
}

/// Spawns tokio tasks which send mavlink messages periodically to the channel.
#[allow(clippy::too_many_lines)]
fn start_telemetry_streaming(
    input_tree: &InputTree,
    output_tree: &OutputTree,
    conn: &std::sync::mpsc::Sender<messages::Message>,
    rates: &DownLinkRateConfig,
) -> Vec<AbortHandle> {
    let mut handles = input_tree.start_broadcasting(conn.clone());


    handles
}
