//! This crate is meant to define what *a* state machine is and what it can do.
//!
//! Actual implementations of the state machine are in separate crates.

pub use error::{Error, Result};

pub mod actuator_control;
#[cfg(feature = "can")]
pub mod can;
pub mod common_states;
pub mod config;
pub mod error;
pub mod execution;
pub mod reactive;
pub mod state_machine;
pub mod telemetry;
pub mod transition;
pub mod util;
