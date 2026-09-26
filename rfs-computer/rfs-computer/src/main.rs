//! The main entry point for the remote filling station computer

use datatypes::status::ConfigHash;
use lib_mission::actuator_control::ActuatorStore as _;
use lib_mission::state_machine::StateMachine;
use lib_mission::util::init::init_binary;
use rfs_computer::build_info;
use rfs_computer::config::Config;
use rfs_computer::io::driver::init_drivers;
use rfs_computer::io::input::InputTree;
use rfs_computer::io::output::OutputTree;
use rfs_computer::io::telemetry::init_telemetry;
use rfs_computer::state::State;
use tracing::Instrument as _;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config_path = std::env::args()
        .nth(1)
        .ok_or("Not enought arguments. Usage: ./rfs_computer <CONFIG>")?;
    println!("> loading config from {config_path}");

    let (config_file, log_folder, run_info) =
        init_binary::<Config>(build_info::THIS_BUILD, config_path).await?;

    let mut input_tree = InputTree::default();
    input_tree.rocket_health.time = input_tree.time.clone(); // synchronize shared variable
    let mut state_tree = State::default();
    let config_hash = ConfigHash {
        hash: run_info.config_hash.as_bytes()[..5]
            .try_into()
            .expect("config hash is hex SHA-1, longer than 5 chars"),
    };
    let _ = state_tree.hash_config.update(config_hash);

    let controllino_driver = controllino::driver::Client::new(
        config_file.controllino.address,
        config_file.controllino.request_timeout,
        state_tree.rfs_computer_status.clone(),
    );

    let (can_tx, can_rx) = std::sync::mpsc::channel();
    let output_tree = OutputTree::new(can_tx.clone(), controllino_driver.clone(), &None);

    let state_machine = StateMachine::new(config_file.state_machine)?;
    state_tree.state_machine_state.current_state = state_machine.current_state.clone(); // synchronize shared variable
    let global_actuator_control =
        lib_mission::actuator_control::command::ActuatorControl::from_strings(
            config_file
                .telemetry
                .always_allowed_actuators
                .iter()
                .map(String::as_str),
        )
        .map_err(lib_mission::Error::malformed_config)?;

    rfs_computer::io::spawn_system_time_driver(
        input_tree.time.clone(),
        std::time::Duration::from_secs_f32(1. / config_file.system_time_rate_hz),
    );

    // probably not nice, but if it worked once, it will continue to work.
    #[allow(clippy::large_futures)]
    input_tree.start_logging(&log_folder).await?;
    output_tree
        .start_logging(&log_folder, input_tree.time.clone())
        .await?;

    init_telemetry(
        &config_file.telemetry,
        &input_tree,
        &state_tree,
        &state_machine,
    )?;

    init_drivers(
        &config_file.uremote,
        &config_file.serial,
        &input_tree,
        &output_tree,
    )
    .await?;
    controllino_driver.start_polling(
        config_file.controllino.pressure_polling_hz,
        config_file.controllino.temperature_polling_hz,
        config_file.controllino.digital_polling_hz,
        config_file.controllino.actuator_polling_hz,
        input_tree.controllino_valves.clone(),
        &input_tree.controllino_sensors,
        None,
    );

    let _can_handles = rfs_computer::io::can::init_can_drivers(
        &config_file.can,
        &input_tree,
        &output_tree,
        &state_tree,
        &can_tx,
        can_rx,
    )?;

    // Set up global actuator commands
    {
        let cmd = input_tree.actuator_command.clone();
        let actuators = output_tree.as_settables();
        tokio::spawn(
            async move {
                global_actuator_control
                    .act_optional(None, false, &cmd, actuators)
                    .await
            }
            .instrument(tracing::info_span!("global_actuators")),
        );
    }

    tokio::select! {
        _ = tokio::signal::ctrl_c() => {
            tracing::info!(shutdown = true, "Interrupt received, shutting down");
        }
        end = state_machine.run(&input_tree, &output_tree, state_tree) => {
            tracing::error!("Exited with error: {end:?}");
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;

    fn assert_config(path: &Path) {
        println!("asserting {path:?} is valid");
        let config_file: Config =
            toml::from_str(&std::fs::read_to_string(path).expect("path must be correct"))
                .expect("file must be parseable");
        let _sm = StateMachine::new(config_file.state_machine)
            .expect("State machine must be buildable from config");
        let _ = lib_mission::actuator_control::command::ActuatorControl::<
            rfs_computer::io::output::ActuatorName,
        >::from_strings(
            config_file
                .telemetry
                .always_allowed_actuators
                .iter()
                .map(String::as_str),
        )
        .expect("global actuators must be parseable");
    }

    #[test]
    fn all_configs_valid() {
        use glob::glob;

        for entry in glob(concat!(env!("CARGO_MANIFEST_DIR"), "/../configs/**/*.toml"))
            .expect("Failed to read glob pattern")
        {
            match entry {
                Ok(path) => assert_config(&path),
                Err(e) => panic!("glob error {e}"),
            }
        }
    }
}
