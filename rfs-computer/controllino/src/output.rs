use std::collections::BTreeMap;

use datatypes::actuator::Settable;
use datatypes::actuator::{DigitalOut, InvertedDigitalOut, NormallyClosedValve};

use crate::driver::CoilSettable;

/// The target states of the actuators connected to the u-remote.
///
/// From `Instrumentation Naming` Excel sheet
#[derive(Clone)]
#[allow(clippy::struct_field_names)]
pub struct Actuators {
    /// The target state of the ROCKET_POWER.
    pub rocket_power: CoilSettable<InvertedDigitalOut>,
    /// The target state of the ROCKET_HEATING.
    pub rocket_heating: CoilSettable<DigitalOut>,
    /// The target state of the LOX decoupler extend valve.
    pub lox_decoupler_extend_valve: CoilSettable<NormallyClosedValve>,
    /// The target state of the LOX decoupler retract valve.
    pub lox_decoupler_retract_valve: CoilSettable<NormallyClosedValve>,
    /// The target state of the N2 decoupler extend valve.
    pub n2_decoupler_extend_valve: CoilSettable<NormallyClosedValve>,
    /// The target state of the N2 decoupler retract valve.
    pub n2_decoupler_retract_valve: CoilSettable<NormallyClosedValve>,
    /// The target state of the N2 shower valves.
    pub n2_shower_valve: CoilSettable<NormallyClosedValve>,
    /// The target state of the N2 engine purge valve.
    pub n2_engine_purge_valve: CoilSettable<NormallyClosedValve>,
}

/// See the matching types in [`Actuators`].
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Deserialize, serde::Serialize,
)]
#[allow(non_camel_case_types)]
pub enum ActuatorName {
    /// The name for [`Actuators::rocket_power`].
    rocket_power,
    /// The name for [`Actuators::rocket_heating`].
    rocket_heating,
    /// The name for [`Actuators::lox_decoupler_extend_valve`].
    lox_dec_ext,
    /// The name for [`Actuators::lox_decoupler_retract_valve`].
    lox_dec_ret,
    /// The name for [`Actuators::n2_decoupler_extend_valve`].
    n2_dec_ext,
    /// The name for [`Actuators::n2_decoupler_retract_valve`].
    n2_dec_ret,
    /// The name for [`Actuators::n2_shower_valve`].
    n2_shw_vlv,
    /// The name for [`Actuators::n2_engine_purge_valve`].
    eng_prg_vlv,
}

impl std::str::FromStr for ActuatorName {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        use serde::Deserialize as _;
        Self::deserialize(<&str as serde::de::IntoDeserializer>::into_deserializer(s))
            .map_err(|e| e.to_string())
    }
}

impl Actuators {
    pub fn as_settables(&self) -> BTreeMap<ActuatorName, Box<dyn Settable + Send>> {
        macro_rules! boxx {
            ($e:expr) => {
                Box::new($e.clone()) as Box<dyn Settable + Send>
            };
        }
        BTreeMap::from([
            (ActuatorName::rocket_power, boxx!(self.rocket_power)),
            (ActuatorName::rocket_heating, boxx!(self.rocket_heating)),
            (
                ActuatorName::lox_dec_ext,
                boxx!(self.lox_decoupler_extend_valve),
            ),
            (
                ActuatorName::lox_dec_ret,
                boxx!(self.lox_decoupler_retract_valve),
            ),
            (
                ActuatorName::n2_dec_ext,
                boxx!(self.n2_decoupler_extend_valve),
            ),
            (
                ActuatorName::n2_dec_ret,
                boxx!(self.n2_decoupler_retract_valve),
            ),
            (ActuatorName::n2_shw_vlv, boxx!(self.n2_shower_valve)),
            (ActuatorName::eng_prg_vlv, boxx!(self.n2_engine_purge_valve)),
        ])
    }
}
