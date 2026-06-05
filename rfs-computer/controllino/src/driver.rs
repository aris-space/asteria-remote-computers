use std::sync::Arc;
use std::time::Duration;

use datatypes::actuator::Energizable;
use tokio::sync::Mutex as AsyncMutex;
use tokio_modbus::client::{Context, Reader};

use crate::input;

pub mod addresses {
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
}

pub struct Client {
    context: Arc<AsyncMutex<Context>>,
}

impl Client {
    /// Reads all actuator states from the modbus server at once and writes them to `actuators`
    pub async fn poll_actuators(&self, actuators: &input::Actuators) {
        match tokio::time::timeout(
            Duration::from_millis(100), // TODO: constant
            self.context
                .lock()
                .await
                .read_coils(addresses::COIL_BASE, addresses::COIL_QUANTITY),
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
        match tokio::time::timeout(
            Duration::from_millis(100), // TODO: constant
            self.context
                .lock()
                .await
                .read_discrete_inputs(addresses::DISCRETE_IN_BASE, addresses::DISCRETE_IN_QUANTITY),
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
}
