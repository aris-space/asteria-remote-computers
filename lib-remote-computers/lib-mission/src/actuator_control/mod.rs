//! This module provides types and telemetry functionality for actuator control.

pub mod command;
pub mod valves;

pub use command::Command;

#[derive(Debug, Clone, Copy, thiserror::Error)]
#[error("The supplied value is out of range for this settable")]
pub struct OutOfRange;

/// A trait for an actuator, such as a valve, that can be set by a single float value.
/// The value usually is the openness for valves, in the 0-1 range.
pub trait Settable {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange>;
}

/// Allows boxing a Watchable of a settable for a boxed trait object.
impl<T: Settable> Settable for watchable::Watchable<T> {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        self.write().set_value(value)
    }
}

impl<T: Settable> Settable for OutChannel<T> {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        self.send(T::from_value(value))
    }
}

/// A trait for an actuator, which can be energized using a binary signal.
pub trait Energizable {
    fn is_active(&self) -> bool;
    fn is_inactive(&self) -> bool {
        !self.is_active()
    }
    fn from_active(is_active: bool) -> Self;
}
