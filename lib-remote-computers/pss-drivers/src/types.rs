use lib_mission::actuator_control::{OutOfRange, Settable};
use serde::{Deserialize, Serialize};

/// The state of a DPR valve.
/// This is a [`Settable`] with the value being NaN for disabled and 0.0 to 60.0 for the setpoint.
#[derive(Debug, Default, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "f32")]
#[serde(into = "f32")]
pub enum DPRValve {
    #[default] // FIXME: is this right?
    Disabled,
    Enabled {
        /// 0-60 barg
        setpoint: f32,
    },
}

/// Used for csv deserialization
impl TryFrom<f32> for DPRValve {
    type Error = OutOfRange;

    fn try_from(value: f32) -> Result<Self, Self::Error> {
        let mut ret = DPRValve::default();
        ret.set_value(value)?;
        Ok(ret)
    }
}

/// Used for csv serialization
impl From<DPRValve> for f32 {
    fn from(value: DPRValve) -> Self {
        match value {
            DPRValve::Disabled => f32::NAN,
            DPRValve::Enabled { setpoint } => setpoint,
        }
    }
}

/// Compatiblity with CAN. Could maybe be same type, but difficult as needed trait impl for `Settable`.
impl From<hermes_can::messages::event_messages::DprState> for DPRValve {
    fn from(value: hermes_can::messages::event_messages::DprState) -> Self {
        match value {
            hermes_can::messages::event_messages::DprState::Enabled(s) => {
                Self::Enabled { setpoint: s }
            }
            hermes_can::messages::event_messages::DprState::Disabled => Self::Disabled,
        }
    }
}

/// Compatiblity with CAN.
impl From<DPRValve> for hermes_can::messages::event_messages::DprState {
    fn from(value: DPRValve) -> Self {
        match value {
            DPRValve::Disabled => hermes_can::messages::event_messages::DprState::Disabled,
            DPRValve::Enabled { setpoint } => {
                hermes_can::messages::event_messages::DprState::Enabled(setpoint)
            }
        }
    }
}

impl Settable for DPRValve {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        if value.is_nan() {
            *self = DPRValve::Disabled;
            Ok(())
        } else if (0.0..=60.0).contains(&value) {
            *self = DPRValve::Enabled { setpoint: value };
            Ok(())
        } else {
            Err(OutOfRange)
        }
    }
}

/// The state of a throttleable valve.
/// This is a [`Settable`] with the value being 0.0 for fully closed up to 1.0 for fully open.
///
/// Note that the default behaviour is a bit weird here, as the valves don't reset on powerloss, but they should start closed.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ThrottleableValve {
    /// The openness in percent (0 being fully closed, 100 being fully open)
    pub openness_percent: u8,
}

/// Explicitly closed initially.
#[allow(clippy::derivable_impls)]
impl Default for ThrottleableValve {
    fn default() -> Self {
        Self {
            openness_percent: 0,
        }
    }
}

impl Settable for ThrottleableValve {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        #[allow(clippy::cast_possible_truncation)] // checked by the branch condition
        #[allow(clippy::cast_sign_loss)] // checked by the branch condition
        if (0.0..=1.0).contains(&value) {
            self.openness_percent = (value * 100.).round() as u8;
            Ok(())
        } else {
            Err(OutOfRange)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::float_cmp)]
    fn test_dpr_valve_settable() {
        // Test disabled state
        let mut valve = DPRValve::default();
        assert_eq!(valve, DPRValve::Disabled);
        assert!(f32::from(valve).is_nan());

        // Test valid setpoint
        valve.set_value(30.0).expect("is valid value");
        assert_eq!(valve, DPRValve::Enabled { setpoint: 30.0 });
        assert_eq!(f32::from(valve), 30.0);

        // Test lower bound
        valve.set_value(0.0).expect("is valid value");
        assert_eq!(valve, DPRValve::Enabled { setpoint: 0.0 });
        // Test upper bound
        valve.set_value(60.0).expect("is valid value");
        assert_eq!(valve, DPRValve::Enabled { setpoint: 60.0 });
        // Test back to disabled
        valve.set_value(f32::NAN).expect("is valid value");
        assert_eq!(valve, DPRValve::Disabled);

        // Test out of range errors
        assert!(valve.set_value(-0.1).is_err());
        assert!(valve.set_value(60.1).is_err());
    }

    #[test]
    fn test_throttleable_valve_settable() {
        let mut valve = ThrottleableValve::default();

        // Test valid values
        valve.set_value(0.0).expect("is valid value");
        assert_eq!(valve.openness_percent, 0);
        valve.set_value(0.5).expect("is valid value");
        assert_eq!(valve.openness_percent, 50);
        valve.set_value(1.0).expect("is valid value");
        assert_eq!(valve.openness_percent, 100);

        // Test rounding
        valve.set_value(0.258).expect("is valid value");
        assert_eq!(valve.openness_percent, 26);
        valve.set_value(0.251).expect("is valid value");
        assert_eq!(valve.openness_percent, 25);
        valve.set_value(0.269).expect("is valid value");
        assert_eq!(valve.openness_percent, 27);

        // Test out of range errors
        assert!(valve.set_value(-0.1).is_err());
        assert!(valve.set_value(1.1).is_err());
    }
}
