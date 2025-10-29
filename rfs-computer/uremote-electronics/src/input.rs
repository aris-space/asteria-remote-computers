use lib_mission::actuator_control::valves::{NormallyClosedValve, NormallyOpenValve};
use lib_mission::util::types::Celsius;
use watchable::Watchable;

use crate::types::{DigitalOut, InvertedDigitalOut, SparkPlug};

#[derive(Debug, Default, Clone)]
pub struct Sensors {
    /// The last received temperature of the Lox line, from the `R-T-L-1` sensor.
    pub lox_temperature: Watchable<Celsius>,
    /// The last received temperature of the igniter, from the `I-T-I-1` sensor.
    pub igniter_temperature: Watchable<Celsius>,
    /// The last received temperature of the cabinet thermocouple.
    pub cabinet_temperature: Watchable<Celsius>,
    /// The last received temperature of the ambient thermocouple.
    pub ambient_temperature: Watchable<Celsius>,
    /// `true` if proximity of the lox decoupler to the retracted state is detected.
    pub lox_decoupler_proximity: Watchable<bool>, // FIXME: newtype would be nicer
    /// `true` if proximity of the n2 decoupler to the retracted state is detected.
    pub n2_decoupler_proximity: Watchable<bool>,
    /// `true` if the valves receive power.
    pub valves_armed: Watchable<bool>, // FIXME: newtype would be nicer
    /// `true` if the spark plug receives power.
    pub igniter_armed: Watchable<bool>,
}

/// The states of the actuators connected to the u-remote.
///
/// For most valves this is based on whether they are powered or not.
/// H2 Bleed and Gox Bleed have additional feedback.
#[derive(Debug, Default, Clone)]
#[allow(clippy::struct_field_names)]
pub struct Actuators {
    /// The state of the ROCKET_POWER (`rocket_power`).
    pub rocket_power: Watchable<InvertedDigitalOut>,
    /// The state of the MAXON_BOARD_POWER (`maxon_board_power`).
    pub maxon_board_power: Watchable<InvertedDigitalOut>,
    /// The state of the FSS_HEATING (`fss_heating`).
    pub fss_heating: Watchable<DigitalOut>,
    /// The state of the PRESSURE_SENSOR_HEATING (`pres_sensor_heat`).
    pub pressure_sensor_heating: Watchable<DigitalOut>,
    /// The state of the ETH_PUMP_POWER (`eth_pump_power`).
    pub eth_pump_power: Watchable<DigitalOut>,
    /// The state of the CUSTOM1 (`custom1`).
    pub custom1: Watchable<DigitalOut>,
    /// The state of the CUSTOM2 (`custom2`).
    pub custom2: Watchable<DigitalOut>,
    /// The state of the CUSTOM3 (`custom3`).
    pub custom3: Watchable<DigitalOut>,

    /// The state of the GOX purge valve (`gox_prg_vlv`).
    pub gox_purge_valve: Watchable<NormallyClosedValve>,
    /// The state of the H2 purge valve (`h2_prg_vlv`).
    pub h2_purge_valve: Watchable<NormallyClosedValve>,
    /// The state of the N2 bleed valve (`n2_bld_vlv`).
    pub n2_bleed_valve: Watchable<NormallyOpenValve>,
    /// The state of the LOX purge valve (`lox_prg_vlv`).
    pub lox_purge_valve: Watchable<NormallyClosedValve>,
    /// The state of the GOX bleed valve (`gox_bld_vlv`).
    pub gox_bleed_valve: Watchable<NormallyOpenValve>,
    /// The state of the N2 purge valve (`n2_prg_vlv`).
    pub n2_purge_valve: Watchable<NormallyClosedValve>,
    /// The state of the N2 fill valve (`n2_fll_vlv`).
    pub n2_fill_valve: Watchable<NormallyClosedValve>,
    /// The state of the H2 bleed valve (`h2_bld_vlv`).
    pub h2_bleed_valve: Watchable<NormallyOpenValve>,
    /// The state of the LOX bleed valve (`lox_bld_vlv`).
    pub lox_bleed_valve: Watchable<NormallyClosedValve>,
    /// The state of the Booster valve (`pnm_bstr_vlv`).
    pub booster_valve: Watchable<NormallyClosedValve>,
    /// The state of the LOX decoupler extend valve (`pnm_loxdec_ext`).
    pub lox_decoupler_extend_valve: Watchable<NormallyClosedValve>,
    /// The state of the LOX decoupler retract valve (`pnm_loxdec_ret`).
    pub lox_decoupler_retract_valve: Watchable<NormallyClosedValve>,
    /// The state of the N2 decoupler extend valve (`pnm_n2dec_ext`).
    pub n2_decoupler_extend_valve: Watchable<NormallyClosedValve>,
    /// The state of the N2 decoupler retract valve (`pnm_n2dec_ret`).
    pub n2_decoupler_retract_valve: Watchable<NormallyClosedValve>,
    /// The state of the N2 release valve (`pnm_n2rls_vlv`).
    pub n2_release_valve: Watchable<NormallyClosedValve>,
    /// The state of the H2 igniter valve (`h2_ign_vlv`).
    pub h2_igniter_valve: Watchable<NormallyClosedValve>,
    /// The state of the GOX igniter valve (`gox_ign_vlv`).
    pub gox_igniter_valve: Watchable<NormallyClosedValve>,
    /// The state of the Igniter spark (`ign_spk_plg`).
    pub igniter_spark: Watchable<SparkPlug>,
}
