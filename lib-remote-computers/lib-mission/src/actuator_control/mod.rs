//! This module provides types and telemetry functionality for actuator control.

pub mod command;

pub use command::Command;
use datatypes::actuator::Settable;
use std::collections::BTreeMap;

/// Trait for a struct that bundles actuators named by `T`
pub trait ActuatorStore<T> {
    /// Returns a map of actuator names with their associated [`Settable`]
    /// that builds and sends the right CAN message when [`Settable::set_value`]
    /// is called.
    fn as_settables(&self) -> BTreeMap<T, Box<dyn Settable + Send>>;
}
