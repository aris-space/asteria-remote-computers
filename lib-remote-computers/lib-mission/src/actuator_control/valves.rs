//! This modules defines common state representations of valves as actuators to be used in IO trees.

use serde::{Deserialize, Serialize};

use crate::actuator_control::Energizable;

use super::{OutOfRange, Settable};

/// The state of a valve that is normally open.
/// This is a [`Settable`] with the value being 0.0 for closed and 1.0 for open.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NormallyOpenValve {
    #[default]
    Open,
    Closed,
}

/// The state of a valve that is normally closed.
/// This is a [`Settable`] with the value being 0.0 for closed and 1.0 for open.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NormallyClosedValve {
    Open,
    #[default]
    Closed,
}

impl NormallyOpenValve {
    pub fn is_open(&self) -> bool {
        matches!(self, Self::Open)
    }

    pub fn is_closed(&self) -> bool {
        matches!(self, Self::Closed)
    }
}

impl Energizable for NormallyOpenValve {
    fn is_active(&self) -> bool {
        matches!(self, Self::Closed)
    }

    fn from_active(is_active: bool) -> Self {
        if is_active { Self::Closed } else { Self::Open }
    }
}

impl Settable for NormallyOpenValve {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        match value {
            0.0 => {
                *self = NormallyOpenValve::Closed;
                Ok(())
            }
            1.0 => {
                *self = NormallyOpenValve::Open;
                Ok(())
            }
            _ => Err(OutOfRange),
        }
    }
}

impl NormallyClosedValve {
    pub fn is_open(&self) -> bool {
        matches!(self, Self::Open)
    }

    pub fn is_closed(&self) -> bool {
        matches!(self, Self::Closed)
    }
}

impl Energizable for NormallyClosedValve {
    fn is_active(&self) -> bool {
        matches!(self, Self::Open)
    }

    fn from_active(is_active: bool) -> Self {
        if is_active { Self::Open } else { Self::Closed }
    }
}

impl Settable for NormallyClosedValve {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        match value {
            0.0 => {
                *self = NormallyClosedValve::Closed;
                Ok(())
            }
            1.0 => {
                *self = NormallyClosedValve::Open;
                Ok(())
            }
            _ => Err(OutOfRange),
        }
    }
}

/// Waaaay too many tests for such simple logic, but why not.
#[cfg(test)]
mod tests {
    use super::*;

    mod normally_open_valve {
        use super::*;

        #[test]
        fn default_state() {
            let valve = NormallyOpenValve::default();
            assert!(valve.is_open());
            assert!(!valve.is_closed());
            assert_eq!(valve, NormallyOpenValve::Open);
        }

        #[test]
        fn set_value() {
            let mut valve = NormallyOpenValve::default();

            // Test valid values
            assert!(valve.set_value(0.0).is_ok());
            assert!(valve.is_closed());

            assert!(valve.set_value(1.0).is_ok());
            assert!(valve.is_open());

            // Test invalid values, unchanged state
            assert!(valve.set_value(0.5).is_err());
            assert!(valve.is_open());
        }

        #[test]
        fn energizable() {
            // Default is not active (normally open)
            let valve = NormallyOpenValve::default();
            assert!(!valve.is_active());

            // From active
            let active_valve = NormallyOpenValve::from_active(true);
            assert!(active_valve.is_active());
            assert!(active_valve.is_closed());

            let inactive_valve = NormallyOpenValve::from_active(false);
            assert!(!inactive_valve.is_active());
            assert!(inactive_valve.is_open());
        }
    }

    mod normally_closed_valve {
        use super::*;

        #[test]
        fn default_state() {
            let valve = NormallyClosedValve::default();
            assert!(valve.is_closed());
            assert!(!valve.is_open());
            assert_eq!(valve, NormallyClosedValve::Closed);
        }

        #[test]
        fn set_value() {
            let mut valve = NormallyClosedValve::default();

            // Test valid values
            assert!(valve.set_value(0.0).is_ok());
            assert!(valve.is_closed());

            assert!(valve.set_value(1.0).is_ok());
            assert!(valve.is_open());

            // Test invalid values, unchanged state
            assert!(valve.set_value(0.5).is_err());
            assert!(valve.is_open());
        }

        #[test]
        fn energizable() {
            // Default is not active (normally closed)
            let valve = NormallyClosedValve::default();
            assert!(!valve.is_active());

            // From active
            let active_valve = NormallyClosedValve::from_active(true);
            assert!(active_valve.is_open());
            assert!(active_valve.is_active());

            let inactive_valve = NormallyClosedValve::from_active(false);
            assert!(inactive_valve.is_closed());
            assert!(!inactive_valve.is_active());
        }
    }
}
