//! CAN consumers that react to received messages.

use datatypes::actuator::{DigitalOut, NormallyClosedValve};
use datatypes::status::ArmingState;
use lib_mission::can::ConsumerRegistry;
use messages::datapoints::dp_rfs_computer;
use messages::datapoints::dp_rfs_computer::Message as RfsMessage;
use tokio::task::AbortHandle;
use tracing::Instrument as _;
use watchable::Watchable;

use crate::io::input::InputTree;

pub fn spawn_and_register_igniter_control(
    registry: &mut ConsumerRegistry<InputTree>,
    rfs_computer_status: Watchable<dp_rfs_computer::RfsComputerStatus>,
    gox_igniter_valve: Watchable<NormallyClosedValve>,
    h2_igniter_valve: Watchable<NormallyClosedValve>,
    gox_purge_valve: Watchable<NormallyClosedValve>,
    h2_purge_valve: Watchable<NormallyClosedValve>,
    igniter_spark: Watchable<DigitalOut>,
) -> AbortHandle {
    use dp_rfs_computer::__message_meta_can_ids as ids;

    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
    for id in [
        ids::GoxIgniterValveControl,
        ids::H2IgniterValveControl,
        ids::IgniterSparkPlugControl,
        ids::H2IgniterPurgeControl,
        ids::GoxIgniterPurgeControl,
    ] {
        if registry.register(id, tx.clone()).is_err() {
            tracing::error!("kicking out other consumer for igniter control");
        }
    }

    tokio::spawn(
        async move {
            while let Some(msg) = rx.recv().await {
                // Gate on the arming state: ignore all commands while disarmed.
                if matches!(
                    rfs_computer_status.get().is_ignition_armed,
                    ArmingState::Safe
                ) {
                    tracing::trace!("ignoring igniter control while disarmed");
                    continue;
                }

                match msg {
                    messages::Message::RfsComputer(RfsMessage::GoxIgniterValveControl(target)) => {
                        tracing::debug!("setting gox_ign_vlv to {target:?}");
                        gox_igniter_valve.replace(target);
                    }
                    messages::Message::RfsComputer(RfsMessage::H2IgniterValveControl(target)) => {
                        tracing::debug!("setting h2_ign_vlv to {target:?}");
                        h2_igniter_valve.replace(target);
                    }
                    messages::Message::RfsComputer(RfsMessage::IgniterSparkPlugControl(target)) => {
                        tracing::debug!("setting ign_spk_plg to {target:?}");
                        igniter_spark.replace(target);
                    }
                    messages::Message::RfsComputer(RfsMessage::GoxIgniterPurgeControl(target)) => {
                        tracing::debug!("setting gox_prg_vlv to {target:?}");
                        gox_purge_valve.replace(target);
                    }
                    messages::Message::RfsComputer(RfsMessage::H2IgniterPurgeControl(target)) => {
                        tracing::debug!("setting h2_prg_vlv to {target:?}");
                        h2_purge_valve.replace(target);
                    }
                    _ => tracing::warn!(?msg, "received unsubscribed message"),
                }
            }
            tracing::warn!("Shutting down");
        }
        .instrument(tracing::info_span!("drivers::IgniterControl")),
    )
    .abort_handle()
}
