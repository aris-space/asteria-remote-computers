//! This module serves to communicate with the Modbus server on the Controllino.
//!
//! Helper functions can poll data and write it into input trees.
//! Output data is directly written

use std::sync::Arc;
use std::time::Duration;

use datatypes::actuator::Energizable;
use datatypes::units::{BarG, Celsius};
use tokio::sync::Mutex as AsyncMutex;
use tokio_modbus::Address;
use tokio_modbus::client::{Context, Reader, Writer as _};

use crate::input;

/// The Modbus addresses as defined in the `Instrumentation Naming` excel sheet.
mod addresses {
    use tokio_modbus::{Address, Quantity};

    pub const COIL_BASE: Address = ROCKET_POWER;
    pub const ROCKET_POWER: Address = 0x100;
    pub const LOX_DECOUPLER_EXTEND_VALVE: Address = 0x101;
    pub const LOX_DECOUPLER_RETRACT_VALVE: Address = 0x102;
    pub const N2_DECOUPLER_EXTEND_VALVE: Address = 0x103;
    pub const N2_DECOUPLER_RETRACT_VALVE: Address = 0x104;
    pub const N2_SHOWER_VALVE: Address = 0x105;
    pub const N2_ENGINE_PURGE_VALVE: Address = 0x106;
    pub const HEATING_PADS_POWER: Address = 0x107;
    pub const COIL_QUANTITY: Quantity = HEATING_PADS_POWER - COIL_BASE + 1;

    pub const DISCRETE_IN_BASE: Address = VALVE_POWER_INTERLOCK;
    pub const VALVE_POWER_INTERLOCK: Address = 0x200;
    pub const ROCKET_CONNECTION_DETECT: Address = 0x201;
    pub const LOX_DECOUPLER_PROXIMITY: Address = 0x202;
    pub const N2_DECOUPLER_PROXIMITY: Address = 0x203;
    pub const DISCRETE_IN_QUANTITY: Quantity = N2_DECOUPLER_PROXIMITY - DISCRETE_IN_BASE + 1;

    // High words of the floats. Low word is + 1
    pub const HOLDING_REGISTER_FLOAT_HI_OFFSET: Address = 0;
    pub const HOLDING_REGISTER_FLOAT_LO_OFFSET: Address = 1;
    pub const HOLDING_REGISTER_FLOAT_LEN: Address = 2;

    pub const IGNITER_PRESSURE_BASE: Address = 0x300;

    pub const IGNITER_TEMPERATURE_BASE: Address = 0x302;
    pub const SMART_BOX_INTERNAL_TEMPERATURE_BASE: Address = 0x304;
    pub const TEMPERATURE_QUANTITY: Quantity = 2;
}

#[derive(Debug, Clone)]
pub struct Client {
    /// Timeout for the Modbus requests using `poll_*` functions.
    /// Does not include locking time, thus if `context` stays locked, the functions will wait.
    timeout: Duration,
    /// Underlying connection to the Controllino
    context: Arc<AsyncMutex<Context>>,
}

impl Client {
    /// Reads all actuator states from the modbus server at once and writes them to `actuators`
    pub async fn poll_actuators(&self, actuators: &input::Actuators) {
        let mut ctx = self.context.lock().await;
        match tokio::time::timeout(
            self.timeout,
            ctx.read_coils(addresses::COIL_BASE, addresses::COIL_QUANTITY),
        )
        .await
        {
            Ok(Ok(Ok(vals))) => {
                #[allow(clippy::wildcard_imports)] // scoped finely enough here
                use addresses::*;
                if vals.len() != COIL_QUANTITY as usize {
                    tracing::warn!(
                        "modbus server didn't return all coils: {count}",
                        count = vals.len()
                    );
                    return;
                }
                let _ = actuators.rocket_power.update(Energizable::from_active(
                    vals[(ROCKET_POWER - COIL_BASE) as usize],
                ));
                let _ = actuators
                    .lox_decoupler_extend_valve
                    .update(Energizable::from_active(
                        vals[(LOX_DECOUPLER_EXTEND_VALVE - COIL_BASE) as usize],
                    ));
                let _ = actuators
                    .lox_decoupler_retract_valve
                    .update(Energizable::from_active(
                        vals[(LOX_DECOUPLER_RETRACT_VALVE - COIL_BASE) as usize],
                    ));
                let _ = actuators
                    .n2_decoupler_extend_valve
                    .update(Energizable::from_active(
                        vals[(N2_DECOUPLER_EXTEND_VALVE - COIL_BASE) as usize],
                    ));
                let _ = actuators
                    .n2_decoupler_retract_valve
                    .update(Energizable::from_active(
                        vals[(N2_DECOUPLER_RETRACT_VALVE - COIL_BASE) as usize],
                    ));
                let _ = actuators.n2_shower_valve.update(Energizable::from_active(
                    vals[(N2_SHOWER_VALVE - COIL_BASE) as usize],
                ));
                let _ = actuators
                    .n2_engine_purge_valve
                    .update(Energizable::from_active(
                        vals[(N2_ENGINE_PURGE_VALVE - COIL_BASE) as usize],
                    ));
                let _ = actuators.fss_heating.update(Energizable::from_active(
                    vals[(HEATING_PADS_POWER - COIL_BASE) as usize],
                ));
            }
            Ok(Ok(Err(code))) => {
                tracing::warn!("modbus server rejected actuator coil read: {code}");
            }
            Ok(Err(err)) => {
                tracing::warn!("Cannot reach modbus server: {err}");
            }
            Err(_elapsed) => {
                tracing::warn!("Timed out modbus server");
            }
        }
    }

    /// Reads all digital inputs from the modbus server at once and writes them to `sensors`
    pub async fn poll_discrete_inputs(&self, sensors: &input::Sensors) {
        let mut ctx = self.context.lock().await;
        match tokio::time::timeout(
            self.timeout,
            ctx.read_discrete_inputs(addresses::DISCRETE_IN_BASE, addresses::DISCRETE_IN_QUANTITY),
        )
        .await
        {
            Ok(Ok(Ok(vals))) => {
                #[allow(clippy::wildcard_imports)] // scoped finely enough here
                use addresses::*;
                if vals.len() != DISCRETE_IN_QUANTITY as usize {
                    tracing::warn!(
                        "modbus server didn't return all discrete inputs: {count}",
                        count = vals.len()
                    );
                    return;
                }
                let _ = sensors
                    .valve_power_interlock
                    .update(Energizable::from_active(
                        vals[(VALVE_POWER_INTERLOCK - DISCRETE_IN_BASE) as usize],
                    ));
                let _ = sensors
                    .rocket_connection_detect
                    .update(Energizable::from_active(
                        vals[(ROCKET_CONNECTION_DETECT - DISCRETE_IN_BASE) as usize],
                    ));
                let _ = sensors
                    .lox_decoupler_proximity
                    .update(Energizable::from_active(
                        vals[(LOX_DECOUPLER_PROXIMITY - DISCRETE_IN_BASE) as usize],
                    ));
                let _ = sensors
                    .n2_decoupler_proximity
                    .update(Energizable::from_active(
                        vals[(N2_DECOUPLER_PROXIMITY - DISCRETE_IN_BASE) as usize],
                    ));
            }
            Ok(Ok(Err(code))) => {
                tracing::warn!("modbus server rejected discrete input read: {code}");
            }
            Ok(Err(err)) => {
                tracing::warn!("Cannot reach modbus server: {err}");
            }
            Err(_elapsed) => {
                tracing::warn!("Timed out modbus server");
            }
        }
    }

    /// Reads the analog input for the pressure sensor from the modbus server and write it to `sensors`
    ///
    /// This is separate, as it needs to be called more frequently.
    pub async fn poll_pressure_sensor(&self, sensors: &input::Sensors) {
        let mut ctx = self.context.lock().await;
        match tokio::time::timeout(
            self.timeout,
            ctx.read_holding_registers(
                addresses::IGNITER_PRESSURE_BASE,
                addresses::HOLDING_REGISTER_FLOAT_LEN,
            ),
        )
        .await
        {
            Ok(Ok(Ok(vals))) => {
                #[allow(clippy::wildcard_imports)] // scoped finely enough here
                use addresses::*;
                if vals.len() != HOLDING_REGISTER_FLOAT_LEN as usize {
                    tracing::warn!(
                        "modbus server didn't return all words of the pressure sensor: {count}",
                        count = vals.len()
                    );
                    return;
                }
                let _ = sensors
                    .igniter_pressure
                    .update(BarG(decode_f32_from_holding_registers(
                        vals[(HOLDING_REGISTER_FLOAT_HI_OFFSET) as usize],
                        vals[(HOLDING_REGISTER_FLOAT_LO_OFFSET) as usize],
                    )));
            }
            Ok(Ok(Err(code))) => {
                tracing::warn!("modbus server rejected holding register read: {code}");
            }
            Ok(Err(err)) => {
                tracing::warn!("Cannot reach modbus server: {err}");
            }
            Err(_elapsed) => {
                tracing::warn!("Timed out modbus server");
            }
        }
    }

    /// Reads all analog inputs for the temperature sensors from the modbus server at once and writes them to `sensors`
    pub async fn poll_temperature_sensors(&self, sensors: &input::Sensors) {
        let mut ctx = self.context.lock().await;
        match tokio::time::timeout(
            self.timeout,
            ctx.read_holding_registers(
                addresses::IGNITER_TEMPERATURE_BASE,
                addresses::TEMPERATURE_QUANTITY * addresses::HOLDING_REGISTER_FLOAT_LEN,
            ),
        )
        .await
        {
            Ok(Ok(Ok(vals))) => {
                #[allow(clippy::wildcard_imports)] // scoped finely enough here
                use addresses::*;
                if vals.len() != (TEMPERATURE_QUANTITY * HOLDING_REGISTER_FLOAT_LEN) as usize {
                    tracing::warn!(
                        "modbus server didn't return all words of the pressure sensor: {count}",
                        count = vals.len()
                    );
                    return;
                }

                let _ =
                    sensors
                        .igniter_temperature
                        .update(Celsius(decode_f32_from_holding_registers(
                            vals[(HOLDING_REGISTER_FLOAT_HI_OFFSET) as usize],
                            vals[(HOLDING_REGISTER_FLOAT_LO_OFFSET) as usize],
                        )));
                let _ = sensors.smart_box_temperature.update(Celsius(
                    decode_f32_from_holding_registers(
                        vals[((SMART_BOX_INTERNAL_TEMPERATURE_BASE - IGNITER_TEMPERATURE_BASE)
                            + HOLDING_REGISTER_FLOAT_HI_OFFSET)
                            as usize],
                        vals[((SMART_BOX_INTERNAL_TEMPERATURE_BASE - IGNITER_TEMPERATURE_BASE)
                            + HOLDING_REGISTER_FLOAT_LO_OFFSET)
                            as usize],
                    ),
                ));
            }
            Ok(Ok(Err(code))) => {
                tracing::warn!("modbus server rejected holding register read: {code}");
            }
            Ok(Err(err)) => {
                tracing::warn!("Cannot reach modbus server: {err}");
            }
            Err(_elapsed) => {
                tracing::warn!("Timed out modbus server");
            }
        }
    }

    /// Writes a single coil at given address to state `high`.
    pub async fn write_coil(&self, coil: Address, high: bool) {
        let mut ctx = self.context.lock().await;
        match tokio::time::timeout(self.timeout, ctx.write_single_coil(coil, high)).await {
            Ok(Ok(Ok(()))) => tracing::debug!("Set coil {coil:x} to {high}"),
            Ok(Ok(Err(code))) => {
                tracing::warn!("modbus server denied coil write to {coil:x}: {code}");
            }
            Ok(Err(err)) => {
                tracing::warn!("Cannot reach modbus server: {err}");
            }
            Err(_elapsed) => {
                tracing::warn!("Timed out modbus server");
            }
        }
    }
}

fn decode_f32_from_holding_registers(data_hi: u16, data_lo: u16) -> f32 {
    f32::from_bits(u32::from(data_lo) | (u32::from(data_hi) << 16))
}
