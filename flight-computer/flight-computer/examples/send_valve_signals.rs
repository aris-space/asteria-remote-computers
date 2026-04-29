//! Just sends some CAN messages for PSS valves for quick and dirty testing.

use std::time::Duration;

use datatypes::actuator::NormallyOpenValve;
use datatypes::status::ConfigHash;
use flight_computer::build_info;
use flight_computer::config::{Config, MissionProfile};
use flight_computer::io::can::init_can_drivers;
use flight_computer::io::input::InputTree;
use flight_computer::io::output::OutputTree;
use flight_computer::io::telemetry::init_telemetry;
use flight_computer::state_logic::control_loop::AnyGncController;
use lib_mission::state_machine::StateMachine;
use lib_mission::util::init::init_binary;
use messages::datapoints::dp_fuel_control_board;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config_path = std::env::args()
        .nth(1)
        .ok_or("Not enought arguments. Usage: ./flight_computer <CONFIG>")?;
    println!("loading config from {config_path}");

    let (config_file, log_folder, run_info) =
        init_binary::<Config<MissionProfile<AnyGncController>>>(
            build_info::THIS_BUILD,
            config_path,
        )
        .await?;

    let mut input_tree = InputTree::default();
    input_tree.boards_status.time = input_tree.time.clone(); // synchronize shared variable

    let output_tree = OutputTree::default();
    let config_hash = ConfigHash {
        hash: run_info.config_hash.as_bytes()[..5]
            .try_into()
            .expect("config hash is hex SHA-1, longer than 5 chars"),
    };
    let _ = output_tree.hash_config.update(config_hash);
    let state_machine = StateMachine::new(config_file.state_machine)?;
    input_tree.state.current_state = state_machine.current_state.clone(); // synchronize shared variable

    flight_computer::io::spawn_system_time_driver(
        input_tree.time.clone(),
        std::time::Duration::from_secs_f32(1. / config_file.system_time_rate_hz),
    );

    input_tree.start_logging(&log_folder).await?;
    output_tree
        .start_logging(&log_folder, input_tree.time.clone())
        .await?;

    init_telemetry(
        &config_file.telemetry,
        &input_tree,
        &output_tree,
        &state_machine,
    )?;

    let (can_tx, _) = init_can_drivers(&config_file.can, &input_tree, &output_tree)?;
    flight_computer::io::gpio::start_status_led(
        build_info::THIS_BUILD.is_release,
        build_info::THIS_BUILD.git_dirty,
    );

    tokio::spawn(send_some_messages(can_tx));

    tokio::select! {
        _ = tokio::signal::ctrl_c() => {
            tracing::info!(shutdown = true, "Interrupt received, shutting down");
        }
        end = state_machine.run(&input_tree, &output_tree, ()) => {
            tracing::error!("Exited with error: {end:?}");
        }
    }
    Ok(())
}

async fn send_some_messages(tx: std::sync::mpsc::Sender<messages::Message>) {
    loop {
        tx.send(messages::Message::FuelControlBoard(
            dp_fuel_control_board::Message::PressurizationVentValveControlFC(
                NormallyOpenValve::Closed,
            ),
        ))
        .expect("failed to send msg");
        sleep(Duration::from_secs(1)).await;
        tx.send(messages::Message::FuelControlBoard(
            dp_fuel_control_board::Message::PressurizationVentValveControlFC(
                NormallyOpenValve::Open,
            ),
        ))
        .expect("failed to send msg");
        sleep(Duration::from_secs(2)).await;
    }
}
