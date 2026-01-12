use hermes_can::messages::event_messages::{ThrustCurveConfig, ThrustPoint};
use lib_mission::can::{ConsumerRegistry, read_can, run_can_transmit_actor};
use lib_mission::reactive::IntoWatched;
use lib_mission::util::time::MissionTime;
use lib_mission::Error;
use tokio::task::AbortHandle;
#[cfg(feature = "benchmark")]
use lib_mission::can::{BenchStats};

use crate::config::CANConfig;

use super::input::InputTree;
use super::output::OutputTree;

#[cfg(feature = "benchmark")]
use std::sync::atomic::{AtomicU64, Ordering};
#[cfg(feature = "benchmark")]
pub static CAN_MESSAGES_SENT: AtomicU64 = AtomicU64::new(0);
#[cfg(feature = "benchmark")]
pub static CAN_TOTAL_LATENCY_NS: AtomicU64 = AtomicU64::new(0);

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

    // Prepare the ECU
    let mut thrust_curve = <[ThrustPoint; 25]>::default();
    if config.ecu_thrust_curve.len() > 25 {
        return Err(Error::malformed_config("thurst curve with too many points").into());
    }
    thrust_curve[..config.ecu_thrust_curve.len()].copy_from_slice(&config.ecu_thrust_curve);
    let thrust_curve = ThrustCurveConfig {
        thrust_curve,
        thrust_curve_length: u8::try_from(config.ecu_thrust_curve.len())
            .expect("no truncate since we checked size above <= 25"),
    };

    // Initialize the driver tasks
    let mut consumers = ConsumerRegistry::new();
    let input_handles = init_input_drivers(input_tree, &mut consumers);
    let (tx, rx) = std::sync::mpsc::channel();
    let output_handles = init_output_drivers(&input_tree.time, output_tree, &tx, thrust_curve);

    // Spawn the receiving thread
    std::thread::spawn(move || {
        let mut receiver = receiver;
        read_can(&mut receiver, &consumers);
    });
    // Spawn the transmitting thread
    std::thread::spawn(move || {
        let mut transmitter = transmitter;
        run_can_transmit_actor(&mut transmitter, rx);
    });

    Ok([input_handles, output_handles].concat())
}

/// Initializes all the drivers writing to the input tree by spawning tasks to consume CAN messages.
fn init_input_drivers(
    input_tree: &InputTree,
    consumers: &mut ConsumerRegistry,
) -> Vec<AbortHandle> {
    // Register the consumers
    let InputTree {
        time,
        transition_command: _, // doesn't originate from CAN
        actuator_command: _,   // doesn't originate from CAN
        telemetry_health: _,   // doesn't originate from CAN
        boards_health,
        rfs_computer_status,
        recovery_armed,
        ecu_armed,
        raw_unprocessed_data,
        motors_position,
        cats_separation: _, // doesn't originate from CAN,
        cats_deployment: _, // doesn't originate from CAN,
        location,
        velocity,
        acceleration,
        orientation,
        pss_valves,
        pss_sensors,
        ecu_firing_state,
        fuel_pressurization_state,
        oxidizer_pressurization_state,
    } = input_tree;

    let mut handles = vec![
        input_drivers::spawn_and_register_board_health_driver(
            consumers,
            boards_health.clone(),
            rfs_computer_status.clone(),
            recovery_armed.clone(),
            ecu_armed.clone(),
            time.watched(),
        ),
        input_drivers::spawn_and_register_kinematics_driver(
            consumers,
            location.clone(),
            velocity.clone(),
            acceleration.clone(),
            orientation.clone(),
        ),
        input_drivers::spawn_and_register_raw_unprocessed_data_driver(
            consumers,
            raw_unprocessed_data.environmental_data.clone(),
            raw_unprocessed_data.rail_status_24v.clone(),
            raw_unprocessed_data.rail_status_5v.clone(),
        ),
        input_drivers::spawn_and_register_actual_steering_driver(
            consumers,
            motors_position.clone(),
        ),
        input_drivers::spawn_and_register_ecu_firing_messages(consumers, ecu_firing_state.clone()),
        input_drivers::spawn_and_register_pressurization_messages(
            consumers,
            fuel_pressurization_state.clone(),
            oxidizer_pressurization_state.clone(),
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

/// Initializes all the drivers reading from the output tree by spawning tasks to produce CAN messages.
///
/// Additionally, `thrust_curve` is sent to the ECU after 5 seconds.
fn init_output_drivers<C>(
    clock: C,
    output_tree: &OutputTree,
    tx: &std::sync::mpsc::Sender<hermes_can::messages::Message>,
    thrust_curve: ThrustCurveConfig,
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
        ignite,
        abort_ignition,
        camera_powered_on,
        pressurize_fuel,
        pressurize_oxidizer,
        abort_fuel_pressurization,
        abort_oxidizer_pressurization,
        valves,
    } = output_tree;
    // Register the emitters
    let mut handles = vec![
        output_drivers::spawn_flight_computer_status_driver(
            valves.watch_active(),
            recovery_actuators_armed.watched(),
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
        output_drivers::spawn_ignition_driver(ignite.watched(), clock.watched(), tx.clone()),
        output_drivers::spawn_abort_ignition_driver(
            abort_ignition.watched(),
            clock.watched(),
            tx.clone(),
        ),
        output_drivers::spawn_fuel_pressurization_driver(
            pressurize_fuel.watched(),
            clock.watched(),
            tx.clone(),
        ),
        output_drivers::spawn_oxidizer_pressurization_driver(
            pressurize_oxidizer.watched(),
            clock.watched(),
            tx.clone(),
        ),
        output_drivers::spawn_abort_fuel_pressurization_driver(
            abort_fuel_pressurization.watched(),
            clock.watched(),
            tx.clone(),
        ),
        output_drivers::spawn_abort_oxidizer_pressurization_driver(
            abort_oxidizer_pressurization.watched(),
            clock.watched(),
            tx.clone(),
        ),
        output_drivers::spawn_thrust_curve_driver(thrust_curve, tx.clone()),
    ];
    handles.extend(valves.spawn_output_drivers_fc(tx));
    handles
}

#[cfg(test)]
//#[cfg(feature = "benchmark")]
mod benchmark {
    use std::time::{Duration, Instant};
    
    use crate::config::CANConfig;
    use crate::io::types::{ECUFiringState, IgnitionState};
    use crate::io::can::init_can_drivers;
    use crate::io::input::InputTree;
    use crate::io::output::OutputTree;
    use lib_mission::reactive::{IntoWatched, Watched};
    use std::sync::mpsc;

    // CAN config
    #[tokio::test]
    #[ignore]
    async fn benchmark_test() {
        let config = CANConfig {
            socket_address: "can0".to_string(),
            ecu_thrust_curve: vec![],
        };
        
        let output_tree = OutputTree::default();
        let input_tree = InputTree::default();
        
        init_can_drivers(&config, &input_tree, &output_tree)
        .expect("Could not initialize CAN drivers");
        
        // create channel for passing timer
        let (tx_start, mut rx_start) = tokio::sync::mpsc::channel::<Instant>(1);
        let (tx_end, mut rx_end) = tokio::sync::mpsc::channel::<Instant>(1);
        
        // listen for changes in input tree
        // end counting
        tokio::spawn(async move {
            let mut o_watcher = output_tree.ignite.watched();
            loop {
                let o_state = o_watcher.next().await;
                if matches!(o_state, IgnitionState::Ignited) {
                    tx_start.send(Instant::now()).await.ok();
                    break;
                }
            }
        });
        
        // check for changes in input tree
        tokio::spawn(async move {
            let mut i_watcher = input_tree.ecu_firing_state.watched();
            loop {
                let i_state = i_watcher.next().await;
                if matches!(i_state, ECUFiringState::Initiated) {
                    tx_end.send(Instant::now()).await.ok();
                    break;
                }
            }  
        });
        
        let t1 = rx_start.recv().await?;
        let t2 = rx_end.recv().await?;
          
        println!("Latency: {:?}", t2 - t1);
        
    }
}