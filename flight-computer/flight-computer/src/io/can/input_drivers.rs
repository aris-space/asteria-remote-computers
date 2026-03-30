use dp_recovery_board::SteeringPositions;
use lib_mission::can::ConsumerRegistry;
use messages::datapoints::dp_recovery_board;
use tokio::task::AbortHandle;
use tracing::Instrument as _;
use watchable::Watchable;

/// Listens for `SteeringActualPositions` messages.
pub fn spawn_and_register_actual_steering_driver(
    registry: &mut ConsumerRegistry,
    actual_pos: Watchable<SteeringPositions>,
) -> AbortHandle {
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
    if let Err(()) = registry.register(
        messages::datapoints::dp_recovery_board::__message_meta_can_ids::SteeringActualPositions,
        tx,
    ) {
        tracing::error!("kicking out other consumer");
    }

    tokio::spawn(
        async move {
            while let Some(msg) = rx.recv().await {
                let messages::Message::RecoveryBoard(
                    dp_recovery_board::Message::SteeringActualPositions(pos),
                ) = msg
                else {
                    tracing::warn!("Received message it hasn't subscribed to");
                    continue;
                };

                let _old = actual_pos.update(pos);
            }
            tracing::warn!("Shutting down");
        }
        .instrument(tracing::info_span!("drivers::ActualSteeringPos")),
    )
    .abort_handle()
}
