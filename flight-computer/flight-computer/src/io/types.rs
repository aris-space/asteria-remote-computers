//! Defines all types used in the data structures.

pub use hermes_can::messages::sensor_data::OrientationData as Orientation;

/// Position in WGS84 inertial frame.
/// Mirroring [`hermes_can::messages::sensor_data::PositionData`].
#[derive(Debug, Default, Clone, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct Location {
    /// Location Latitude WGS84 in `°`
    pub latitude: f64,
    /// Location Longitude WGS84 in `°`
    pub longitude: f64,
    /// Location Altitude above sealevel in `m`
    pub altitude: f32,

    /// Horizontal accuracy estimate in meters (1-sigma).
    pub horizontal_accuracy: f32,
    /// Vertical accuracy estimate in meters (1-sigma).
    pub vertical_accuracy: f32,
}

/// Velocity (m/s) in both inertial (native) and box fixed frame.
/// Mirroring [`hermes_can::messages::sensor_data::VelocityData`].
#[derive(Debug, Default, Clone, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct Velocity {
    /// Linear velocity north (inertial frame) in `m/s`
    pub north: f32,
    /// Linear velocity east (inertial frame) in `m/s`
    pub east: f32,
    /// Linear velocity down (inertial frame) in `m/s`
    pub down: f32,

    /// Velocity along body X-axis (forward) in `m/s`
    pub x: f32,
    /// Velocity along body Y-axis (right) in `m/s`
    pub y: f32,
    /// Velocity along body Z-axis (down) in `m/s`
    pub z: f32,
}

/// Acceleration (m/s^2) in both inertial and body fixed (native) frame.
/// Subset of [`hermes_can::messages::sensor_data::ImuData`].
#[derive(Debug, Default, Clone, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct Acceleration {
    /// Linear acceleration north (inertial frame) in `m/s^2`
    pub north: f32,
    /// Linear acceleration east (inertial frame) in `m/s^2`
    pub east: f32,
    /// Linear acceleration down (inertial frame) in `m/s^2`
    pub down: f32,

    /// Acceleration along body X-axis (forward) in `m/s²`
    pub x: f32,
    /// Acceleration along body Y-axis (right) in `m/s²`
    pub y: f32,
    /// Acceleration along body Z-axis (down) in `m/s²`
    pub z: f32,
}

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

/// Describes the target state of the nosecone separation, and thus drogue deployment.
#[derive(Default, Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub enum SeparationState {
    #[default]
    NotYetSeparated,
    Separated,
}

/// Tells whether we should deploy the chute.
#[derive(Default, Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub enum MainDeploymentState {
    #[default]
    NotYetDeployed,
    Deployed,
}

/// The feedback we get from the ECU about the firing sequence.
#[derive(Default, Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub enum ECUFiringState {
    /// In default state, no firing initated yet.
    #[default]
    NotYetInitiated,
    /// A firing was initated, and the ECU acknowledged that.
    Initiated,
    /// The igniter has ignited.
    IgnitionDetected,
    /// The main chamber has combustion.
    CombustionDetected,
    /// The ECU sent a completed message last.
    Completed,
    /// The ECU last sent an aborted message.
    Aborted,
}

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

/// Tells whether we should send the ignition signal.
/// Whenever this is set to `Ignited`, the signal is sent.
#[derive(Default, Debug, Clone, Copy, serde::Serialize)]
pub enum IgnitionState {
    #[default]
    NotYetIgnited,
    Ignited,
}

/// Tells whether we should send the abort ignition signal.
#[derive(Default, Debug, Clone, Copy, serde::Serialize)]
pub enum AbortIgnitionState {
    #[default]
    NotYetAborted,
    Aborted,
}

/// Tells whether we should send the fuel pressurization signal.
#[derive(Default, Debug, Clone, Copy, serde::Serialize)]
pub enum FuelPressurizationState {
    #[default]
    NotYetPressurized,
    Pressurized(f32),
}

/// Tells whether we should send the oxidizer pressurization signal.
#[derive(Default, Debug, Clone, Copy, serde::Serialize)]
pub enum OxidizerPressurizationState {
    #[default]
    NotYetPressurized,
    Pressurized(f32),
}

/// Tells whether we should send the fuel pressurization abort signal.
#[derive(Default, Debug, Clone, Copy, serde::Serialize)]
pub enum AbortFuelPressurizationState {
    #[default]
    NotYetAborted,
    Aborted,
}

/// Tells whether we should send the oxidizer pressurization abort signal.
#[derive(Default, Debug, Clone, Copy, serde::Serialize)]
pub enum AbortOxidizerPressurizationState {
    #[default]
    NotYetAborted,
    Aborted,
}

/// The feedback we get from the ECU about the fuel pressurization sequence.
#[derive(Default, Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub enum FuelPressurizationCompletedState {
    #[default]
    NotYetCompleted,
    Completed,
}

/// The feedback we get from the ECU about the oxidizer pressurization sequence.
#[derive(Default, Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub enum OxidizerPressurizationCompletedState {
    #[default]
    NotYetCompleted,
    Completed,
}

/// The tank that should be pressurized at the pressurization sequence
#[derive(Debug, Clone, Copy, serde::Deserialize, serde::Serialize)]
pub enum Tank {
    Fuel,
    Oxidizer,
}
