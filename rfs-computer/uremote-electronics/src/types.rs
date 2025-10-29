//! This modules defines specialized state representations of actuators hooked to the Weidmüller coupler.

use lib_mission::actuator_control::{Energizable, OutOfRange, Settable};
use serde::{Deserialize, Serialize};

/// The state of the spark plug.
/// This is a [`Settable`] with the value being 0.0 for off and 1.0 for on.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SparkPlug {
    #[default]
    Off,
    On,
}

impl Energizable for SparkPlug {
    fn is_active(&self) -> bool {
        matches!(self, Self::On)
    }

    fn from_active(is_active: bool) -> Self {
        if is_active { Self::On } else { Self::Off }
    }
}

impl Settable for SparkPlug {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        match value {
            0.0 => {
                *self = SparkPlug::Off;
                Ok(())
            }
            1.0 => {
                *self = SparkPlug::On;
                Ok(())
            }
            _ => Err(OutOfRange),
        }
    }
}

/// The state of a digital out.
/// This is a [`Settable`] with the value being 0.0 for off and 1.0 for on.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DigitalOut {
    #[default]
    Off,
    On,
}

/// Convert to Mavlink On-ness.
impl From<DigitalOut> for u8 {
    fn from(value: DigitalOut) -> Self {
        match value {
            DigitalOut::Off => 0,
            DigitalOut::On => 1,
        }
    }
}

impl Energizable for DigitalOut {
    fn is_active(&self) -> bool {
        matches!(self, Self::On)
    }

    fn from_active(is_active: bool) -> Self {
        if is_active { Self::On } else { Self::Off }
    }
}

impl Settable for DigitalOut {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        match value {
            0.0 => {
                *self = DigitalOut::Off;
                Ok(())
            }
            1.0 => {
                *self = DigitalOut::On;
                Ok(())
            }
            _ => Err(OutOfRange),
        }
    }
}

/// The state of an inverted digital out.
/// This is a [`Settable`] with the value being 0.0 for off and 1.0 for on.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum InvertedDigitalOut {
    Off,
    #[default]
    On,
}

/// Convert to Mavlink On-ness.
impl From<InvertedDigitalOut> for u8 {
    fn from(value: InvertedDigitalOut) -> Self {
        match value {
            InvertedDigitalOut::Off => 0,
            InvertedDigitalOut::On => 1,
        }
    }
}

impl Energizable for InvertedDigitalOut {
    fn is_active(&self) -> bool {
        matches!(self, Self::Off)
    }

    fn from_active(is_active: bool) -> Self {
        if is_active { Self::Off } else { Self::On }
    }
}

impl Settable for InvertedDigitalOut {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        match value {
            0.0 => {
                *self = InvertedDigitalOut::Off;
                Ok(())
            }
            1.0 => {
                *self = InvertedDigitalOut::On;
                Ok(())
            }
            _ => Err(OutOfRange),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    mod spark_plug {
        use super::*;

        #[test]
        fn default_state() {
            let plug = SparkPlug::default();
            assert!(!plug.is_active());
            assert_eq!(plug, SparkPlug::Off);
        }

        #[test]
        fn set_value() {
            let mut plug = SparkPlug::default();

            // Test valid values
            assert!(plug.set_value(0.0).is_ok());
            assert_eq!(plug, SparkPlug::Off);

            assert!(plug.set_value(1.0).is_ok());
            assert_eq!(plug, SparkPlug::On);

            // Test invalid values, unchanged state
            assert!(plug.set_value(0.5).is_err());
            assert_eq!(plug, SparkPlug::On);
        }

        #[test]
        fn energizable() {
            // Default is not active
            let plug = SparkPlug::default();
            assert!(!plug.is_active());

            // From active
            let active_plug = SparkPlug::from_active(true);
            assert!(active_plug.is_active());
            assert_eq!(active_plug, SparkPlug::On);

            let inactive_plug = SparkPlug::from_active(false);
            assert!(!inactive_plug.is_active());
            assert_eq!(inactive_plug, SparkPlug::Off);
        }
    }

    mod digital_out {
        use super::*;

        #[test]
        fn default_state() {
            let out = DigitalOut::default();
            assert!(!out.is_active());
            assert_eq!(u8::from(out), 0);
            assert_eq!(out, DigitalOut::Off);
        }

        #[test]
        fn set_value() {
            let mut out = DigitalOut::default();

            // Test valid values
            assert!(out.set_value(0.0).is_ok());
            assert_eq!(out, DigitalOut::Off);

            assert!(out.set_value(1.0).is_ok());
            assert_eq!(out, DigitalOut::On);

            // Test invalid values, unchanged state
            assert!(out.set_value(0.5).is_err());
            assert_eq!(out, DigitalOut::On);
        }

        #[test]
        fn energizable() {
            // Default is not active
            let out = DigitalOut::default();
            assert!(!out.is_active());

            // From active
            let active_out = DigitalOut::from_active(true);
            assert!(active_out.is_active());
            assert_eq!(active_out, DigitalOut::On);

            let inactive_out = DigitalOut::from_active(false);
            assert!(!inactive_out.is_active());
            assert_eq!(inactive_out, DigitalOut::Off);
        }
    }

    mod inverted_digital_out {
        use super::*;

        #[test]
        fn default_state() {
            let out = InvertedDigitalOut::default();
            assert!(!out.is_active());
            assert_eq!(u8::from(out), 1);
            assert_eq!(out, InvertedDigitalOut::On);
        }

        #[test]
        fn set_value() {
            let mut out = InvertedDigitalOut::default();

            // Test valid values
            assert!(out.set_value(0.0).is_ok());
            assert_eq!(out, InvertedDigitalOut::Off);
            assert!(out.is_active());

            assert!(out.set_value(1.0).is_ok());
            assert_eq!(out, InvertedDigitalOut::On);
            assert!(!out.is_active());

            // Test invalid values, unchanged state
            assert!(out.set_value(0.5).is_err());
            assert_eq!(out, InvertedDigitalOut::On);
        }

        #[test]
        fn energizable() {
            // Default is not active (inverted)
            let out = InvertedDigitalOut::default();
            assert!(!out.is_active());

            // From active
            let active_out = InvertedDigitalOut::from_active(true);
            assert!(active_out.is_active());
            assert_eq!(active_out, InvertedDigitalOut::Off);

            let inactive_out = InvertedDigitalOut::from_active(false);
            assert!(!inactive_out.is_active());
            assert_eq!(inactive_out, InvertedDigitalOut::On);
        }
    }
}
