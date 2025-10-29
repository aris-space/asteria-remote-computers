#![allow(clippy::unwrap_used)]

use hermes_mavlink::{COMMAND_MANUAL_TRANSITION_DATA, MavMessage, RemoteSystem};
use mavlink_core::{MavConnection, MavHeader, connect};
use std::io::{self, Write};
use std::str;
use std::sync::Arc;

fn main() {
    // Ask for the UDP port
    print!("Enter the UDP port to connect to: ");
    io::stdout().flush().unwrap();
    let mut port = String::new();
    io::stdin()
        .read_line(&mut port)
        .expect("Failed to read line");
    let port: u16 = port.trim().parse().expect("Invalid port number");

    // Connect to localhost at the specified port
    let addr = format!("udpin:127.0.0.1:{port}");
    let conn: Arc<dyn MavConnection<MavMessage> + Send + Sync> = connect(&addr)
        .expect("Failed to connect to MAVLink router")
        .into();
    println!(
        "Connected to MAVLink {:?} router at {addr}",
        conn.get_protocol_version()
    );

    // Downlink
    {
        let conn = conn.clone();
        std::thread::spawn(move || {
            print_downlink(conn.as_ref());
        });
    }

    // Uplink
    prompt_uplink(conn.as_ref());
}

fn prompt_uplink(router: &(dyn MavConnection<MavMessage> + Send + Sync)) -> ! {
    let mut sequence: u8 = 1;
    loop {
        // Ask for the index and name
        print!("> Enter index and name (separated by space): ");
        io::stdout().flush().unwrap();
        let mut input = String::new();
        io::stdin()
            .read_line(&mut input)
            .expect("Failed to read line");

        let parts: Vec<&str> = input.trim().splitn(2, ' ').collect();
        if parts.len() != 2 {
            println!("Invalid input. Please enter index and name separated by space.");
            continue;
        }

        let Ok(index) = parts[0].parse::<u32>() else {
            println!("Invalid input. Please enter index and name separated by space.");
            continue;
        };
        let name = parts[1];

        // Prepare the message
        let mut state = [0u8; 30];
        let name_bytes = name.as_bytes();
        let len = name_bytes.len().min(30);
        state[..len].copy_from_slice(&name_bytes[..len]);

        let data = COMMAND_MANUAL_TRANSITION_DATA {
            target_system: RemoteSystem::ROCKET,
            transition_index: index,
            state,
        };

        let message = MavMessage::COMMAND_MANUAL_TRANSITION(data);
        let header = MavHeader {
            system_id: 201,  // The sending system (GS uses high values)
            component_id: 2, // The sending component (non-standard)
            sequence,        // The component should increment this once per sent message
        };

        // Send the message
        match router.send(&header, &message) {
            Ok(_) => println!("Message sent."),
            Err(e) => println!("Error sending message: {e:?}"),
        }

        sequence = sequence.wrapping_add(1);
    }
}

fn print_downlink(router: &(dyn MavConnection<MavMessage> + Send + Sync)) -> ! {
    loop {
        let Ok((header, msg)) = router.recv() else {
            eprintln!("couldn't receive msg, trying again in 1sec");
            std::thread::sleep(std::time::Duration::from_secs(1));
            continue;
        };

        println!();
        println!(" |received message");
        println!(" |header = {header:?}");
        println!(" |msg = {msg:#?}");
        print!("> (Enter index and name): ");
        io::stdout().flush().unwrap();
    }
}
