use crate::util::timer::Timer;
use gnc_interface::MotorDeflections;
use lib_mission::reactive::{IntoWatched, Repeatedly, Watched};
use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
use lib_mission::transition::ManualTransitions;
use messages::datapoints::dp_sensor_carrier as sensor_types;
use nalgebra::{Quaternion, UnitQuaternion};
use std::sync::Mutex;
use std::time::Duration;

use crate::io::types;

#[derive(Debug)]
pub struct ControlLoop<Gnc> {
    /// The GNC implementation, initialized during startup.
    /// Needs to be in a Mutex since they require mutability. This struct and thus the mutex is never shared though.
    pub(crate) gnc_controller: Mutex<Gnc>,
    /// How often the control loop is called in the period between invocations..
    pub(crate) loop_period: Duration,
    /// The configured manual transitions.
    pub(crate) manual_transitions: ManualTransitions,
    /// Touchdown detection isn't implemented, instead:
    /// When the time since the transition into this state is larger
    /// than `time_limit`, we transition to `on_touchdown` which serves to conserve power.
    pub(crate) time_limit: Duration,
    /// The state to advance to once we detected touchdown.
    pub(crate) on_touchdown: StateName,
    /// The state to advance to when GNC return an error.
    pub(crate) on_failure: StateName,
    /// The number of full revolutions pulling the left line for the 0 and 1 GNC output in this order.
    /// Can be negative to signify relaxation beyond the intial state.
    /// See [`types::SteeringMotorRevolutions`].
    pub(crate) revolution_interval_mapping_left: (f64, f64),
    /// The number of full revolutions pulling the right line for the 0 and 1 GNC output in this order.
    /// Can be negative to signify relaxation beyond the intial state.
    /// See [`types::SteeringMotorRevolutions`].
    pub(crate) revolution_interval_mapping_right: (f64, f64),
}

#[async_trait::async_trait]
impl<Gnc> StateLogic for ControlLoop<Gnc>
where
    Gnc: gnc_interface::GuidanceController,
{
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::ControlLoop::run", skip_all, fields(state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        let mut looper = Repeatedly::new(input.time.watched(), self.loop_period);
        let mut timer = Timer::new(input.time.watched(), self.time_limit);
        loop {
            let gnc_out = tokio::select! {
                next = self.manual_transitions.next(input, param) => {
                    return next;
                },
                _ = looper.next() => {
                    let mut gnc = self
                        .gnc_controller
                        .lock()
                        .expect("GNC lock was poisoned: can only happen if critical section panics");
                    tracing::debug!(gnc_state = ?gnc, "calling GNC");
                    // FIXME: might need to add angular velocity.
                    gnc.step(
                        input.time.get().into(),
                        conv_loc(input.location.get()),
                        conv_vel(input.velocity.get()),
                        conv_accl(input.acceleration.get()),
                        quat_to_angles(&input.orientation.get()),
                    )
                },
                _ = timer.wait() => {
                    tracing::info!(cause = "time trigger", "transitioning");
                    return self.on_touchdown.clone();
                },
            };
            // To aid the IDE, we fall through the select only when GNC was called.

            let motor = match gnc_out {
                Ok(m) => m,
                Err(e) => {
                    tracing::error!("GNC returned an error: {e}");
                    return self.on_failure.clone();
                }
            };

            match self.convert_gnc_output(&motor) {
                Ok(steps) => {
                    let _old = output.steering_motor_target_deflections.update(Some(steps));
                }
                Err(e) => {
                    tracing::error!(error = %e, "failure converting the GNC output");
                }
            }
        }
    }
}

#[derive(Debug, thiserror::Error)]
enum Error {
    #[error("GNC returned a {0} deflection outside the unit interval: {1}")]
    GNCIntervalError(&'static str, f64),
    #[error("The deflections {0:?} lead to non-finite or out of range revolutions.")]
    NonFiniteRevolutions((f64, f64)),
}

impl<Gnc> ControlLoop<Gnc> {
    fn convert_gnc_output(
        &self,
        motor: &MotorDeflections,
    ) -> Result<types::SteeringMotorRevolutions, Error> {
        // Convert the GNC output first to revolutions by linear interpolation and then to steps.
        let (l, r) = (motor.motor_left, motor.motor_right);
        let left_revs = if (0.0..=1.0).contains(&l) {
            // Linearly interpolate
            self.revolution_interval_mapping_left.0
                + (self.revolution_interval_mapping_left.1
                    - self.revolution_interval_mapping_left.0)
                    * l
        } else {
            return Err(Error::GNCIntervalError("left", l));
        };
        let right_revs = if (0.0..=1.0).contains(&r) {
            // Linearly interpolate
            self.revolution_interval_mapping_right.0
                + (self.revolution_interval_mapping_right.1
                    - self.revolution_interval_mapping_right.0)
                    * r
        } else {
            return Err(Error::GNCIntervalError("right", r));
        };

        let Some(steps) =
            types::SteeringMotorRevolutions::try_from_revolutions(left_revs, right_revs)
        else {
            return Err(Error::NonFiniteRevolutions((l, r)));
        };
        Ok(steps)
    }
}

#[allow(clippy::similar_names)]
fn quat_to_angles(orient: &sensor_types::OrientationData) -> gnc_interface::Orientation {
    let quat = Quaternion::new(
        f64::from(orient.orientation_w),
        f64::from(orient.orientation_x),
        f64::from(orient.orientation_y),
        f64::from(orient.orientation_z),
    );
    let unitq = UnitQuaternion::from_quaternion(quat);
    let (roll, pitch, yaw) = unitq.euler_angles();

    tracing::trace!(
        quat_w = orient.orientation_w,
        quat_x = orient.orientation_x,
        quat_y = orient.orientation_y,
        quat_z = orient.orientation_z,
        roll,
        pitch,
        yaw,
        "quat_to_angles"
    );

    gnc_interface::Orientation { roll, pitch, yaw }
}

/// Conversion from internal type to GNC type.
fn conv_loc(loc: sensor_types::PositionData) -> gnc_interface::Position {
    gnc_interface::Position {
        latitude: loc.location_latitude,
        longitude: loc.location_longitude,
        altitude: loc.location_hamsl.into(),
    }
}

/// Conversion from internal type to GNC type.
fn conv_vel(vel: sensor_types::VelocityData) -> gnc_interface::Velocity {
    gnc_interface::Velocity {
        north: vel.velocity_north.into(),
        east: vel.velocity_east.into(),
        down: vel.velocity_down.into(),
    }
}

/// Conversion from internal type to GNC type.
fn conv_accl(accl: sensor_types::ImuData) -> gnc_interface::Acceleration {
    gnc_interface::Acceleration {
        north: accl.acceleration_north.into(),
        east: accl.acceleration_east.into(),
        down: accl.acceleration_down.into(),
    }
}

/// A [`gnc_interface::GuidanceController`] implementor that moves the motors in sine waves. Used for testing.
#[derive(Debug)]
pub struct MockGncController;

impl gnc_interface::GuidanceController for MockGncController {
    type Config = ();

    fn new((): Self::Config) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(MockGncController)
    }

    fn step(
        &mut self,
        time_unix: std::time::Duration, // Current time as a duration since 1970.
        _position: gnc_interface::Position,
        _velocity: gnc_interface::Velocity,
        _acceleration: gnc_interface::Acceleration,
        _orientation: gnc_interface::Orientation,
    ) -> Result<gnc_interface::MotorDeflections, gnc_interface::GuidanceControllerError> {
        Ok(gnc_interface::MotorDeflections {
            // convert -1, 1 to 0, 1. Period is 2pi * 3.
            motor_left: (time_unix.as_secs_f64() / 3.).sin() / 2. + 0.5,
            motor_right: (time_unix.as_secs_f64() / 3.).cos() / 2. + 0.5,
        })
    }
}

/// A [`gnc_interface::GuidanceController`] implementor that moves the motors in known patterns. Used for testing.
/// The pattern is:
/// - Move left motor to 0 for 5 sec
/// - Move left motor to 1 for 5 sec
/// - Move to 0.5 for 5 sec
/// - Move right motor to 0 for 5 sec
/// - Move right motor to 1 for 5 sec
/// - Move to 0.5 for 5 sec
/// - repeat after another 5 sec
#[derive(Debug)]
pub struct SelfTestGncController(Option<std::time::Duration>);

impl gnc_interface::GuidanceController for SelfTestGncController {
    type Config = ();

    fn new((): Self::Config) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(SelfTestGncController(None))
    }

    fn step(
        &mut self,
        time_unix: std::time::Duration,
        _position: gnc_interface::Position,
        _velocity: gnc_interface::Velocity,
        _acceleration: gnc_interface::Acceleration,
        _orientation: gnc_interface::Orientation,
    ) -> Result<gnc_interface::MotorDeflections, gnc_interface::GuidanceControllerError> {
        let Some(start_time) = self.0 else {
            self.0 = Some(time_unix);
            return Ok(gnc_interface::MotorDeflections {
                motor_left: 0.5,
                motor_right: 0.5,
            });
        };
        let time_since = time_unix.checked_sub(start_time).unwrap_or_default();
        let sec_in_period = time_since.as_secs_f32() % 35.;
        let (l, r) = match sec_in_period {
            0.0..5.0 => (0., 0.5),
            5.0..10.0 => (1., 0.5),
            15.0..20.0 => (0.5, 0.),
            20.0..25.0 => (0.5, 1.),
            _ => (0.5, 0.5),
        };
        Ok(gnc_interface::MotorDeflections {
            motor_left: l,
            motor_right: r,
        })
    }
}

/// This controller can be any of multiple choices and delegates to the existing variant.
#[derive(Debug)]
pub enum AnyGncController {
    Mock(MockGncController),
    SelfTest(SelfTestGncController),
    #[cfg(feature = "gnc")]
    ImprovedOApproach(Box<hermes_gnc::guidance_controller::improved_o_approach_pconttroler::ImprovedOApproachPController>),
    #[cfg(feature = "gnc")]
    SysId(Box<hermes_gnc::guidance_controller::sys_id_sys_id_controller::SysIDSysIDController>),
}

/// The config for any of the implemented GNC controllers.
/// The variant is given by the `controller_type` field and other fields are flattened.
#[derive(Debug, serde::Deserialize, serde::Serialize)]
#[serde(tag = "controller_type")]
pub enum AnyGncControllerConfig {
    Mock,
    SelfTest,
    #[cfg(feature = "gnc")]
    ImprovedOApproach(<hermes_gnc::guidance_controller::improved_o_approach_pconttroler::ImprovedOApproachPController as gnc_interface::GuidanceController>::Config),
    #[cfg(feature = "gnc")]
    SysId(<hermes_gnc::guidance_controller::sys_id_sys_id_controller::SysIDSysIDController as gnc_interface::GuidanceController>::Config),
}

impl gnc_interface::GuidanceController for AnyGncController {
    type Config = AnyGncControllerConfig;

    fn new(config: Self::Config) -> Result<Self, Box<dyn std::error::Error>> {
        match config {
            AnyGncControllerConfig::Mock => Ok(Self::Mock(MockGncController)),
            AnyGncControllerConfig::SelfTest => Ok(Self::SelfTest(SelfTestGncController(None))),
            #[cfg(feature = "gnc")]
            AnyGncControllerConfig::ImprovedOApproach(c) => Ok(Self::ImprovedOApproach(Box::new(
                hermes_gnc::guidance_controller::improved_o_approach_pconttroler::ImprovedOApproachPController::new(c)?,
            ))),
            #[cfg(feature = "gnc")]
            AnyGncControllerConfig::SysId(c) => Ok(Self::SysId(Box::new(
                <hermes_gnc::guidance_controller::sys_id_sys_id_controller::SysIDSysIDController as gnc_interface::GuidanceController>::new(c)?,
            ))),
        }
    }

    fn step(
        &mut self,
        time_unix: std::time::Duration,
        position: gnc_interface::Position,
        velocity: gnc_interface::Velocity,
        acceleration: gnc_interface::Acceleration,
        orientation: gnc_interface::Orientation,
    ) -> Result<MotorDeflections, gnc_interface::GuidanceControllerError> {
        match self {
            AnyGncController::Mock(gnc) => {
                gnc.step(time_unix, position, velocity, acceleration, orientation)
            }
            AnyGncController::SelfTest(gnc) => {
                gnc.step(time_unix, position, velocity, acceleration, orientation)
            }
            #[cfg(feature = "gnc")]
            AnyGncController::ImprovedOApproach(gnc) => {
                gnc.step(time_unix, position, velocity, acceleration, orientation)
            }
            #[cfg(feature = "gnc")]
            AnyGncController::SysId(gnc) => {
                gnc.step(time_unix, position, velocity, acceleration, orientation)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_control_loop(
        left_map: (f64, f64),
        right_map: (f64, f64),
    ) -> ControlLoop<MockGncController> {
        ControlLoop {
            gnc_controller: Mutex::new(MockGncController),
            loop_period: Duration::from_millis(10),
            manual_transitions: ManualTransitions::default(),
            time_limit: Duration::from_secs(120),
            on_touchdown: StateName::new_unchecked("touchdown").expect("is short enough"),
            on_failure: StateName::new_unchecked("failure").expect("is short enough"),
            revolution_interval_mapping_left: left_map,
            revolution_interval_mapping_right: right_map,
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    #[test]
    fn test_convert_gnc_output_good() {
        // Test with different mapping ranges
        let control_loop = create_control_loop((-3.0, 3.0), (2.0, 5.0));

        // Left motor: -3 to 3 revs
        let min_left = control_loop
            .convert_gnc_output(&MotorDeflections {
                motor_left: 0.0,
                motor_right: 0.0,
            })
            .expect("is in bounds");
        assert_eq!(min_left.left_motor_steps, (-3.0 * 4096.0) as i32);
        let min_left = control_loop
            .convert_gnc_output(&MotorDeflections {
                motor_left: 0.5,
                motor_right: 0.0,
            })
            .expect("is in bounds");
        assert_eq!(min_left.left_motor_steps, (0.0 * 4096.0) as i32);
        let max_left = control_loop
            .convert_gnc_output(&MotorDeflections {
                motor_left: 1.0,
                motor_right: 0.0,
            })
            .expect("is in bounds");
        assert_eq!(max_left.left_motor_steps, (3.0 * 4096.0) as i32);

        // Right motor: 2 to 5 revs
        let min_right = control_loop
            .convert_gnc_output(&MotorDeflections {
                motor_left: 0.0,
                motor_right: 0.0,
            })
            .expect("is in bounds");
        assert_eq!(min_right.right_motor_steps, (2.0 * 4096.0) as i32);
        let max_right = control_loop
            .convert_gnc_output(&MotorDeflections {
                motor_left: 0.0,
                motor_right: 0.5,
            })
            .expect("is in bounds");
        assert_eq!(max_right.right_motor_steps, (3.5 * 4096.0) as i32);
        let max_right = control_loop
            .convert_gnc_output(&MotorDeflections {
                motor_left: 0.0,
                motor_right: 1.0,
            })
            .expect("is in bounds");
        assert_eq!(max_right.right_motor_steps, (5.0 * 4096.0) as i32);
    }

    #[test]
    fn test_convert_gnc_output_invalid_input() {
        let control_loop = create_control_loop((0.0, 7.0), (0.0, 7.0));

        // Test out of range values
        assert!(matches!(
            control_loop.convert_gnc_output(&MotorDeflections {
                motor_left: -0.1,
                motor_right: 0.5
            }),
            Err(Error::GNCIntervalError("left", -0.1))
        ));
        assert!(matches!(
            control_loop.convert_gnc_output(&MotorDeflections {
                motor_left: 1.1,
                motor_right: 0.5
            }),
            Err(Error::GNCIntervalError("left", 1.1))
        ));
        assert!(matches!(
            control_loop.convert_gnc_output(&MotorDeflections {
                motor_left: 0.5,
                motor_right: -0.1
            }),
            Err(Error::GNCIntervalError("right", -0.1))
        ));
        assert!(matches!(
            control_loop.convert_gnc_output(&MotorDeflections {
                motor_left: 0.5,
                motor_right: 1.1
            }),
            Err(Error::GNCIntervalError("right", 1.1))
        ));
    }

    #[test]
    fn test_convert_gnc_output_non_finite() {
        // Test GNC NaN
        let control_loop = create_control_loop((-1.0, 3.0), (-1.0, 3.0));
        assert!(control_loop
            .convert_gnc_output(&MotorDeflections {
                motor_left: f64::NAN,
                motor_right: 0.5
            })
            .is_err());
        // Test Mapping NaN infinity
        let control_loop = create_control_loop((f64::NAN, 3.0), (-1.0, 3.0));
        assert!(matches!(
            control_loop.convert_gnc_output(&MotorDeflections {
                motor_left: 0.5,
                motor_right: 0.5
            }),
            Err(Error::NonFiniteRevolutions(_))
        ));
        // Test values that would result in out-of-range steps
        let control_loop = create_control_loop((0.0, 8.0), (0.0, 8.0)); // Exceeds 7 rev limit
        assert!(matches!(
            control_loop.convert_gnc_output(&MotorDeflections {
                motor_left: 1.0,
                motor_right: 1.0
            }),
            Err(Error::NonFiniteRevolutions(_))
        ));
    }
}
