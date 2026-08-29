use std::collections::BTreeMap;

use datatypes::actuator::Settable;
use datatypes::actuator::{
    DigitalOut, Energizable, InvertedDigitalOut, NormallyClosedValve, OutOfRange,
};
use watchable::Watchable;

use crate::driver::CoilSettable;

/// A [`CoilSettable`] that also retains the value it was last set to.
///
/// [`CoilSettable`] only writes to the Modbus coil and keeps nothing, so there is
/// nothing to observe. Wrapping it in a [`Watchable`] makes the commanded value
/// watchable, which is what the target state logging needs.
#[derive(Clone)]
pub struct Tracked<T> {
    /// The value this actuator was last commanded to.
    pub target: Watchable<T>,
    coil: CoilSettable<T>,
}

impl<T: Default> Tracked<T> {
    pub(crate) fn new(coil: CoilSettable<T>) -> Self {
        Self {
            target: Watchable::default(),
            coil,
        }
    }
}

impl<T: Energizable + Clone> Tracked<T> {
    /// Records `value` as the new target and writes it to the coil.
    ///
    /// Mirrors [`CoilSettable::set`], and must be used in its place so that the
    /// commanded value stays observable through [`Self::target`].
    pub fn set(&self, value: T) {
        self.target.replace(value.clone());
        self.coil.set(value);
    }
}

impl<T: Default + Settable + Energizable + Clone> Settable for Tracked<T> {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        let mut typed = T::default();
        typed.set_value(value)?;
        self.set(typed);
        Ok(())
    }
}

/// The target states of the actuators connected to the controllino.
///
/// From `Instrumentation Naming` Excel sheet
#[derive(Clone)]
#[allow(clippy::struct_field_names)]
pub struct Actuators {
    /// The target state of the ROCKET_POWER.
    pub rocket_power: Tracked<InvertedDigitalOut>,
    /// The target state of the ROCKET_HEATING.
    pub rocket_heating: Tracked<DigitalOut>,
    /// The target state of the LOX decoupler extend valve.
    pub lox_decoupler_extend_valve: Tracked<NormallyClosedValve>,
    /// The target state of the LOX decoupler retract valve.
    pub lox_decoupler_retract_valve: Tracked<NormallyClosedValve>,
    /// The target state of the N2 decoupler extend valve.
    pub n2_decoupler_extend_valve: Tracked<NormallyClosedValve>,
    /// The target state of the N2 decoupler retract valve.
    pub n2_decoupler_retract_valve: Tracked<NormallyClosedValve>,
    /// The target state of the N2 shower valves.
    pub n2_shower_valve: Tracked<NormallyClosedValve>,
    /// The target state of the N2 engine purge valve.
    pub n2_engine_purge_valve: Tracked<NormallyClosedValve>,
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
