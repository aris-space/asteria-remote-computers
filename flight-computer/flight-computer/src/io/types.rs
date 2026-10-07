//! Defines all types used in the data structures.

/// Software arming state of the recovery actuators: Steering, Separation and Deployment.
/// Armed means they are powered.
///
/// Also affects what is sent via CAN for the state machine transitions.
#[derive(Default, Debug, Clone, Copy, PartialEq, serde::Deserialize, serde::Serialize)]
pub enum RecoveryArmingState {
    /// The default state where none of the actuators is powered, and software isn't ready for liftoff.
    #[default]
    Unarmed,
    /// Here the steering motors are powered, but the separation and deployment not yet. Also not ready for liftoff.
    SteeringMotorsLocked,
    /// All actuators are receiving power and software is ready for liftoff.
    Armed,
}

#[derive(Default, Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub enum CameraPowerState {
    #[default]
    Off,
    On,
}

impl CameraPowerState {
    pub fn is_on(self) -> bool {
        matches!(self, CameraPowerState::On)
    }
}

// TODO: replace with REC datapoint type
/// Steps of the steering motor, starting from 0 each during power-up
/// and increasing by 1 for pulling the line a 1/4096th of a full rotation,
/// decreasing analogously for relaxing the line.
/// At most 7 revolutions in each direction are allowed.
#[derive(Default, Debug, Clone, PartialEq)]
pub struct SteeringMotorRevolutions {
    /// Steps of the left motor.
    pub left_motor_steps: i32,
    /// Steps of the right motor.
    pub right_motor_steps: i32,
}

impl SteeringMotorRevolutions {
    pub const STEPS_PER_REVOLUTION: i32 = 4096;

    /// Tries to create steps from full revolutions,
    /// failing if the floats are not finite.
    /// Rounds to the nearest number of steps.
    pub fn try_from_revolutions(left: f64, right: f64) -> Option<Self> {
        if !left.is_finite() || !right.is_finite() || (left.abs() >= 7.0) || (right.abs() >= 7.0) {
            return None;
        }
        #[allow(clippy::cast_possible_truncation)] // Ensured above to fit. Also no TryFrom...
        let left_motor_steps = (left * f64::from(Self::STEPS_PER_REVOLUTION)).round() as i32;
        #[allow(clippy::cast_possible_truncation)] // Ensured above to fit. Also no TryFrom...
        let right_motor_steps = (right * f64::from(Self::STEPS_PER_REVOLUTION)).round() as i32;
        Some(Self {
            left_motor_steps,
            right_motor_steps,
        })
    }

    /// Compares two `SteeringMotorRevolutions` values.
    /// Returns true if both motor values of the caller are greater/equal than
    /// both motor values of the `other`.
    /// Otherwise returns false.
    pub fn both_greater_equal(&self, other: &SteeringMotorRevolutions) -> bool {
        self.left_motor_steps >= other.left_motor_steps
            && self.right_motor_steps >= other.right_motor_steps
    }
}
