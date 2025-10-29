use hermes_can::CanMessage as _;
use hermes_can::messages;
use lib_mission::actuator_control::Energizable;
use lib_mission::actuator_control::valves::NormallyClosedValve;
use lib_mission::can::ConsumerRegistry;
use lib_mission::reactive::Watched;
use lib_mission::util::time::MissionTime;
use tokio::task::AbortHandle;
use tracing::Instrument as _;
use uremote_electronics::types::SparkPlug;
use watchable::Watchable;

use crate::io::input::BoardsHealthyAsOf;

/// Listens for [`messages::board_status`] messages and updates [`BoardsHealthyAsOf`].
/// Additionally, determines if CAN is connected by writing to `is_can_connected` when a message is received,
/// or a timeout is reached.
pub fn spawn_and_register_rocket_health_driver(
    registry: &mut ConsumerRegistry,
    health: BoardsHealthyAsOf,
    fc_status: Watchable<messages::board_status::FlightComputerStatus>,
    is_can_connected: Watchable<bool>,
    clock: impl Watched<MissionTime> + Send + Sync + 'static,
) -> AbortHandle {
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
    for id in [
        messages::board_status::FlightComputerStatus::ID,
        messages::board_status::EngineControlBoardStatus::ID,
        messages::board_status::FuelControlBoardStatus::ID,
        messages::board_status::OxidizerControlBoardStatus::ID,
    ] {
        if let Err(()) = registry.register(id, tx.clone()) {
            tracing::error!("kicking out other consumer");
        }
    }

    tokio::spawn(
        async move {
            // At least once a second a CAN message should arrive, so after 5 sec of no message, we stop sending messages ourselves.
            let sender =
                make_can_watchdog(std::time::Duration::from_secs(5), is_can_connected.clone());

            while let Some(msg) = rx.recv().await {
                // Reset the watchdog, since we received a message
                let _ = is_can_connected.update(true);
                let _ = sender.try_send(());

                // Depending on the board, there are different messages and conditions. Unify them here.
                let (healthy, output, name) = match &msg {
                    messages::Message::FlightComputerStatus(board_status) => {
                        let healthy = board_status.common.errors == 0;
                        let _ = fc_status.update(board_status.clone());
                        (healthy, &health.flight_computer, "Flight Computer")
                    }
                    messages::Message::EngineControlBoardStatus(_board_status) => {
                        let healthy = true; // just connected is fine idk.
                        (
                            healthy,
                            &health.engine_control_board,
                            "Engine Control Board",
                        )
                    }
                    messages::Message::FuelControlBoardStatus(_board_status) => {
                        let healthy = true; // just connected is fine idk.
                        (healthy, &health.fuel_control_board, "Fuel Control Board")
                    }
                    messages::Message::OxidizerControlBoardStatus(_board_status) => {
                        let healthy = true; // just connected is fine idk.
                        (
                            healthy,
                            &health.oxidizer_control_board,
                            "Oxidizer Control Board",
                        )
                    }
                    _ => {
                        tracing::warn!("Received message it hasn't subscribed to");
                        continue;
                    }
                };
                if healthy {
                    // If healthy, report the current time to the input tree
                    let _ = output.update(Some(clock.current()));
                } else {
                    // Else, report the unhealthyness and log a warning
                    tracing::warn!(?msg, "{name} is unhealthy");
                    let _ = output.update(None);
                }
            }
            tracing::warn!("Shutting down");
        }
        .instrument(tracing::info_span!("drivers::BoardHealth")),
    )
    .abort_handle()
}

/// Listens for [`messages::event_messages`] messages for the igniter valves from the ECU.
/// This is gated by a software-armed flag, such that the state machine is still enforceable.
pub fn spawn_and_register_ecu_igniter_valve_control(
    registry: &mut ConsumerRegistry,
    is_software_armed: Watchable<bool>,
    h2_igniter_valve: Watchable<NormallyClosedValve>,
    gox_igniter_valve: Watchable<NormallyClosedValve>,
    h2_purge_valve: Watchable<NormallyClosedValve>,
    gox_purge_valve: Watchable<NormallyClosedValve>,
    igniter_spark: Watchable<SparkPlug>,
) -> AbortHandle {
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
    for id in [
        messages::event_messages::H2IgniterValveControl::ID,
        messages::event_messages::GoxIgniterValveControl::ID,
        messages::event_messages::IgniterPurgeControl::ID,
        messages::event_messages::IgniterSparkPlugControl::ID,
    ] {
        if let Err(()) = registry.register(id, tx.clone()) {
            tracing::error!("kicking out other consumer");
        }
    }

    tokio::spawn(
        async move {
            while let Some(msg) = rx.recv().await {
                if !is_software_armed.get() {
                    // Disarmed, ignore all commands
                    tracing::trace!("skipping valve control while disarmed");
                    continue;
                }
                match &msg {
                    messages::Message::H2IgniterValveControl(target) => {
                        let target = energizable_from_valve_state(target.h2_ign_vlv);
                        tracing::debug!("setting h2_ign_vlv to {target:?}");
                        h2_igniter_valve.replace(target);
                    }
                    messages::Message::GoxIgniterValveControl(target) => {
                        let target = energizable_from_valve_state(target.gox_ign_vlv);
                        tracing::debug!("setting gox_ign_vlv to {target:?}");
                        gox_igniter_valve.replace(target);
                    }
                    // FIXME: this message is mis-named, should be IgniterPurgeControl
                    messages::Message::GoxPurgeValveControl(target) => {
                        let target = energizable_from_valve_state(target.gox_prg_vlv);
                        tracing::debug!("setting gox_prg_vlv to {target:?}");
                        tracing::debug!("setting h2_prg_vlv to {target:?}");
                        gox_purge_valve.replace(target);
                        h2_purge_valve.replace(target);
                    }
                    messages::Message::IgniterSparkPlugControl(target) => {
                        let target = energizable_from_valve_state(target.ign_spk_plg);
                        tracing::debug!("setting ign_spk_plg to {target:?}");
                        igniter_spark.replace(target);
                    }
                    _ => {
                        tracing::warn!("Received message it hasn't subscribed to");
                    }
                }
            }
            tracing::warn!("Shutting down");
        }
        .instrument(tracing::info_span!("drivers::ECUValveControl")),
    )
    .abort_handle()
}

fn energizable_from_valve_state<T: Energizable>(state: messages::board_status::ValveState) -> T {
    T::from_active(matches!(state, messages::board_status::ValveState::Active))
}

/// Spawn the watchdog actor.
/// Sending to the returned sender will reset the timer and if the timer expires, `false` is written to `expire_out`.
///
/// This is used to not send CAN messages when disconnected.
#[must_use]
fn make_can_watchdog(
    duration: std::time::Duration,
    expire_out: Watchable<bool>,
) -> tokio::sync::mpsc::Sender<()> {
    /// Run the watchdog event loop.
    /// Copied & adapted from <https://github.com/barafael/watchdog>
    async fn event_loop(
        duration: std::time::Duration,
        mut reset_rx: tokio::sync::mpsc::Receiver<()>,
        expire_out: Watchable<bool>,
    ) {
        let sleep = tokio::time::sleep(duration);
        tokio::pin!(sleep);
        loop {
            tokio::select! {
                msg = reset_rx.recv() => {
                    match msg {
                        // on reset: restart sleep.
                        Some(()) => {
                            sleep.as_mut().reset(tokio::time::Instant::now() + duration);
                        }
                        // on channel close: exit watchdog.
                        None => break,
                    }
                }
                () = sleep.as_mut() => {
                    // on sleep expiry: set expire_out, and set the sleep point into the future
                    let _ = expire_out.replace(false);
                    sleep.as_mut().reset(tokio::time::Instant::now() + std::time::Duration::from_secs(300));
                },
            }
        }
    }

    let (reset_tx, reset_rx) = tokio::sync::mpsc::channel(16);
    tokio::spawn(event_loop(duration, reset_rx, expire_out));
    reset_tx
}
