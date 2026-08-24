//! The main entry point for the flight computer.
//!
//! Currently lacks a bunch of parts, and will need to also spawn input drivers etc.

use datatypes::status::ConfigHash;
use flight_computer::build_info;
use flight_computer::config::{Config, MissionProfile};
use flight_computer::io::can::init_can_drivers;
use flight_computer::io::gpio::start_cats_driver;
use flight_computer::io::input::InputTree;
use flight_computer::io::output::OutputTree;
use flight_computer::io::telemetry::init_telemetry;
use flight_computer::state_logic::control_loop::AnyGncController;
use lib_mission::state_machine::StateMachine;
use lib_mission::util::init::init_binary;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config_path = std::env::args()
        .nth(1)
        .ok_or("Not enought arguments. Usage: ./flight_computer <CONFIG>")?;
    println!("Loading config from {config_path}");

    let config_result = init_binary::<Config<MissionProfile<AnyGncController>>>(
        build_info::THIS_BUILD,
        config_path,
    )
    .await;
    let (config_file, log_folder, run_info) = match config_result {
        Ok(config) => config,
        Err(err) => {
            println!("Error whilst loading config: {err}");
            std::process::exit(1)
        }
    };

    let mut input_tree = InputTree::default();
    input_tree.boards_status.time = input_tree.time.clone(); // synchronize shared variable

    let (can_tx, can_rx) = std::sync::mpsc::channel();
    let output_tree = OutputTree::new(can_tx.clone());
    let config_hash = ConfigHash {
        hash: run_info.config_hash.as_bytes()[..5]
            .try_into()
            .expect("Config hash is hex SHA-1, longer than 5 chars"),
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

    init_can_drivers(&config_file.can, &input_tree, &output_tree, &can_tx, can_rx)?;
    start_cats_driver(
        input_tree.cats_separation.clone(),
        input_tree.cats_deployment.clone(),
    );
    flight_computer::io::gpio::start_status_led(
        build_info::THIS_BUILD.is_release,
        build_info::THIS_BUILD.git_dirty,
    );

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

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;

    fn assert_config(path: &Path) {
        println!("asserting {path:?} is valid");
        let config_file: Config<MissionProfile<AnyGncController>> =
            toml::from_str(&std::fs::read_to_string(path).expect("path must be correct"))
                .expect("file must be parseable");
        let _sm = StateMachine::new(config_file.state_machine)
            .expect("State machine must be buildable from config");
    }

    #[cfg(feature = "gnc")]
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

    #[cfg(feature = "gnc")]
    #[test]
    fn print_gnc_test_data() {
        fn check_type(
            _: &<gnc_interface::adapter::CompatAdapter<
                gnc_algorithms::SafetySpiralGuidance<
                    gnc_algorithms::CompositeController<
                        gnc_algorithms::LimitCycleGuidance,
                        gnc_algorithms::PController,
                    >,
                >,
            > as gnc_interface::CompatGuidanceController>::Config,
        ) {
        }

        let cfg = gnc_configurations::limit_cycle_hil_duebendorf::hil_limit_cycle_compat_adapter_config_with_safety_spiral();
        check_type(&cfg);

        let ser = toml::to_string(&cfg).expect("has to serialize");
        println!("{ser}");
    }
}
