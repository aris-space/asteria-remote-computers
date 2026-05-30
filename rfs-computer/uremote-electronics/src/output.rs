use std::collections::BTreeMap;

use datatypes::actuator::Settable;
use datatypes::actuator::{DigitalOut, InvertedDigitalOut, NormallyClosedValve, NormallyOpenValve};
use watchable::Watchable;

/// The target states of the actuators connected to the u-remote.
///
/// From `Instrumentation Naming` Excel sheet
#[derive(Debug, Default, Clone)]
#[allow(clippy::struct_field_names)]
pub struct Actuators {
    /// The target state of the ROCKET_POWER.
    pub rocket_power: Watchable<InvertedDigitalOut>,
    /// The target state of the MAXON_BOARD_POWER.
    pub maxon_board_power: Watchable<InvertedDigitalOut>,
    /// The target state of the FSS_HEATING.
    pub fss_heating: Watchable<DigitalOut>,
    /// The target state of the PRESSURE_SENSOR_HEATING.
    pub pressure_sensor_heating: Watchable<DigitalOut>,
    /// The target state of the ETH_PUMP_POWER.
    pub eth_pump_power: Watchable<DigitalOut>,
    /// The target state of the CUSTOM1.
    pub custom1: Watchable<DigitalOut>,
    /// The target state of the CUSTOM2.
    pub custom2: Watchable<DigitalOut>,
    /// The target state of the CUSTOM3.
    pub custom3: Watchable<DigitalOut>,

    /// The target state of the GOX purge valve.
    pub gox_purge_valve: Watchable<NormallyClosedValve>,
    /// The target state of the H2 purge valve.
    pub h2_purge_valve: Watchable<NormallyClosedValve>,
    /// The target state of the N2 bleed valve.
    pub n2_bleed_valve: Watchable<NormallyOpenValve>,
    /// The target state of the LOX purge valve.
    pub lox_purge_valve: Watchable<NormallyClosedValve>,
    /// The target state of the GOX bleed valve.
    pub gox_bleed_valve: Watchable<NormallyOpenValve>,
    /// The target state of the N2 purge valve.
    pub n2_purge_valve: Watchable<NormallyClosedValve>,
    /// The target state of the N2 fill valve.
    pub n2_fill_valve: Watchable<NormallyClosedValve>,
    /// The target state of the H2 bleed valve.
    pub h2_bleed_valve: Watchable<NormallyOpenValve>,
    /// The target state of the LOX bleed valve.
    pub lox_bleed_valve: Watchable<NormallyClosedValve>,
    /// The target state of the Booster valve.
    pub booster_valve: Watchable<NormallyClosedValve>,
    /// The target state of the LOX decoupler extend valve.
    pub lox_decoupler_extend_valve: Watchable<NormallyClosedValve>,
    /// The target state of the LOX decoupler retract valve.
    pub lox_decoupler_retract_valve: Watchable<NormallyClosedValve>,
    /// The target state of the N2 decoupler extend valve.
    pub n2_decoupler_extend_valve: Watchable<NormallyClosedValve>,
    /// The target state of the N2 decoupler retract valve.
    pub n2_decoupler_retract_valve: Watchable<NormallyClosedValve>,
    /// The target state of the N2 shower valves.
    pub n2_shower_valve: Watchable<NormallyClosedValve>,
    /// The target state of the N2 engine purge valve.
    pub n2_engine_purge_valve: Watchable<NormallyClosedValve>,
    /// The target state of the H2 igniter valve.
    pub h2_igniter_valve: Watchable<NormallyClosedValve>,
    /// The target state of the GOX igniter valve.
    pub gox_igniter_valve: Watchable<NormallyClosedValve>,
    /// The target state of the Igniter spark.
    pub igniter_spark: Watchable<DigitalOut>,
}

/// See the matching types in [`Actuators`].
///
/// ```
/// use uremote_electronics::output::ActuatorName;
/// assert_eq!(
///     serde_json::to_string(&ActuatorName::gox_prg_vlv).unwrap(),
///     "\"gox_prg_vlv\""    
/// );
/// assert_eq!("gox_prg_vlv".parse::<ActuatorName>().unwrap(), ActuatorName::gox_prg_vlv)
/// ```
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Deserialize, serde::Serialize,
)]
#[allow(non_camel_case_types)]
pub enum ActuatorName {
    /// The name for [`Actuators::rocket_power`].
    rocket_power,
    /// The name for [`Actuators::maxon_board_power`].
    maxon_board_power,
    /// The name for [`Actuators::fss_heating`].
    fss_heating,
    /// The name for [`Actuators::pressure_sensor_heating`].
    pres_sensor_heat,
    /// The name for [`Actuators::eth_pump_power`].
    eth_pump_power,
    /// The name for [`Actuators::custom1`].
    custom1,
    /// The name for [`Actuators::custom2`].
    custom2,
    /// The name for [`Actuators::custom3`].
    custom3,
    /// The name for [`Actuators::gox_purge_valve`].
    gox_prg_vlv,
    /// The name for [`Actuators::h2_purge_valve`].
    h2_prg_vlv,
    /// The name for [`Actuators::n2_bleed_valve`].
    n2_bld_vlv,
    /// The name for [`Actuators::lox_purge_valve`].
    lox_prg_vlv,
    /// The name for [`Actuators::gox_bleed_valve`].
    gox_bld_vlv,
    /// The name for [`Actuators::n2_purge_valve`].
    n2_prg_vlv,
    /// The name for [`Actuators::n2_fill_valve`].
    n2_fll_vlv,
    /// The name for [`Actuators::h2_bleed_valve`].
    h2_bld_vlv,
    /// The name for [`Actuators::lox_bleed_valve`].
    lox_bld_vlv,
    /// The name for [`Actuators::booster_valve`].
    n2_bst_vlv,
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
    /// The name for [`Actuators::h2_igniter_valve`].
    h2_ign_vlv,
    /// The name for [`Actuators::gox_igniter_valve`].
    gox_ign_vlv,
    /// The name for [`Actuators::igniter_spark`].
    ign_spk_plg,
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
            (
                ActuatorName::maxon_board_power,
                boxx!(self.maxon_board_power),
            ),
            (ActuatorName::fss_heating, boxx!(self.fss_heating)),
            (
                ActuatorName::pres_sensor_heat,
                boxx!(self.pressure_sensor_heating),
            ),
            (ActuatorName::eth_pump_power, boxx!(self.eth_pump_power)),
            (ActuatorName::custom1, boxx!(self.custom1)),
            (ActuatorName::custom2, boxx!(self.custom2)),
            (ActuatorName::custom3, boxx!(self.custom3)),
            (ActuatorName::gox_prg_vlv, boxx!(self.gox_purge_valve)),
            (ActuatorName::h2_prg_vlv, boxx!(self.h2_purge_valve)),
            (ActuatorName::n2_bld_vlv, boxx!(self.n2_bleed_valve)),
            (ActuatorName::lox_prg_vlv, boxx!(self.lox_purge_valve)),
            (ActuatorName::gox_bld_vlv, boxx!(self.gox_bleed_valve)),
            (ActuatorName::n2_prg_vlv, boxx!(self.n2_purge_valve)),
            (ActuatorName::n2_fll_vlv, boxx!(self.n2_fill_valve)),
            (ActuatorName::h2_bld_vlv, boxx!(self.h2_bleed_valve)),
            (ActuatorName::lox_bld_vlv, boxx!(self.lox_bleed_valve)),
            (ActuatorName::n2_bst_vlv, boxx!(self.booster_valve)),
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
            (ActuatorName::h2_ign_vlv, boxx!(self.h2_igniter_valve)),
            (ActuatorName::gox_ign_vlv, boxx!(self.gox_igniter_valve)),
            (ActuatorName::ign_spk_plg, boxx!(self.igniter_spark)),
        ])
    }
}
