//! Binary for sending a single mavlink actuator command.
//!
//! E.g. `cargo run --package rfs-computer --example send_command -- 0 "n2_prg_vlv" 1.0`

#![allow(clippy::unwrap_used)]

use lib_mission::telemetry::hermes_mavlink::{
    COMMAND_MANUAL_ACTUATOR_DATA, MavMessage, RemoteSystem,
};
use mavlink_core::{MavConnection, MavHeader, connect};
use std::process::exit;
use std::str;
use std::sync::Arc;

const USAGE: &str = "Usage: `./binary <state idx: u32> <actuator name: str> <value: float>`";

fn main() {
    // Connect to localhost at the specified port
    let addr = "udpin:127.0.0.1:5761";
    let conn: Arc<dyn MavConnection<MavMessage> + Send + Sync> = connect(addr)
        .expect("Failed to connect to MAVLink router")
        .into();

    let [_, ref path_index, ref actuator, ref value] = std::env::args().collect::<Vec<_>>()[..]
    else {
        eprintln!("Insufficient arg count!\n{USAGE}");
        exit(1);
    };
    let Ok(path_index) = path_index.parse::<u32>() else {
        eprintln!("state index is not a number!\n{USAGE}");
        exit(1);
    };
    if actuator.len() > 20 {
        eprintln!("actuator id is too long!\n{USAGE}");
        exit(1);
    }
    let mut actuator_id = [0u8; 20];
    actuator_id[..actuator.len()].copy_from_slice(actuator.as_bytes());
    let Ok(value) = value.parse::<f32>() else {
        eprintln!("value is not a number!\n{USAGE}");
        exit(1);
    };

    // Prepare the message
    let data = COMMAND_MANUAL_ACTUATOR_DATA {
        target_system: RemoteSystem::REMOTE_FILLING_STATION,
        actuator_id,
        value,
        path_index,
    };

    let message = MavMessage::COMMAND_MANUAL_ACTUATOR(data);
    let header = MavHeader {
        system_id: 202,  // The sending system (GS uses high values)
        component_id: 1, // The sending component (non-standardized)
        sequence: 0,     // This is ignored by the library
    };

    // Needed to initailize udpin
    println!("Waiting to establish connection...");
    if let Err(e) = conn.recv() {
        eprintln!("Failed to receive an update: {e}");
        exit(1);
    }

    // Send the message
    match conn.send(&header, &message) {
        Ok(_) => println!("Message sent."),
        Err(e) => eprintln!("Error sending message: {e:?}"),
    }
}
