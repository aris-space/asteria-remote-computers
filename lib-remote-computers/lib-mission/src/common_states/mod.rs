//! This module contains some [`StateLogic`][crate::state_machine::StateLogic] implementations that are generally usable.

pub mod shutdown;
pub mod waiting;

pub use shutdown::Shutdown;
pub use waiting::Waiting;
