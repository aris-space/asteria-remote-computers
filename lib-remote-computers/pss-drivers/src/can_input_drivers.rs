//! TODO: This module can be removed once Collector is used

use crate::input::{PSSSensors, PSSValves};
use lib_mission::can::ConsumerRegistry;
use tokio::task::AbortHandle;

/// Spawns the CAN drivers listening to the bus and setting input tree values.
pub fn register_and_spawn_input_drivers(
    _actuators: &PSSValves,
    _sensors: &PSSSensors,
    _registry: &mut ConsumerRegistry,
) -> [AbortHandle; 3] {
    todo!("Remove this in favor of a `Collector` implementation")
}
