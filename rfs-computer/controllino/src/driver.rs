use std::{sync::Arc, time::Duration};

use datatypes::actuator::Energizable;
use tokio::sync::Mutex as AsyncMutex;
use tokio_modbus::client::{Context, Reader};

use crate::input;

pub mod addresses {
    use tokio_modbus::{Address, Quantity};

    pub const BASE: Address = ROCKET_POWER;
    pub const ROCKET_POWER: Address = 0x100;
    pub const LOX_DECOUPLER_EXTEND_VALVE: Address = 0x101;
    pub const LOX_DECOUPLER_RETRACT_VALVE: Address = 0x102;
    pub const N2_DECOUPLER_EXTEND_VALVE: Address = 0x103;
    pub const N2_DECOUPLER_RETRACT_VALVE: Address = 0x104;
    pub const N2_SHOWER_VALVE: Address = 0x105;
    pub const N2_ENGINE_PURGE_VALVE: Address = 0x106;
    pub const HEATING_PADS_POWER: Address = 0x107;
    pub const QUANTITY: Quantity = HEATING_PADS_POWER - BASE + 1;
}

pub struct Client {
    context: Arc<AsyncMutex<Context>>,
}

impl Client {
    pub async fn poll_actuators(&self, actuators: &input::Actuators) {
        match tokio::time::timeout(
            Duration::from_millis(100), // TODO: constant
            self.context
                .lock()
                .await
                .read_coils(addresses::BASE, addresses::QUANTITY),
        )
        .await
        {
            Ok(Ok(Ok(vals))) => {
                #[allow(clippy::wildcard_imports)] // scoped finely enough here
                use addresses::*;
                if vals.len() != QUANTITY as usize {
                    tracing::warn!(
                        "modbus server didn't return all coils: {count}",
                        count = vals.len()
                    );
                    return;
                }
                let _ = actuators.rocket_power.update(Energizable::from_active(
                    vals[(ROCKET_POWER - BASE) as usize],
                ));
                let _ = actuators
                    .lox_decoupler_extend_valve
                    .update(Energizable::from_active(
                        vals[(LOX_DECOUPLER_EXTEND_VALVE - BASE) as usize],
                    ));
                let _ = actuators
                    .lox_decoupler_retract_valve
                    .update(Energizable::from_active(
                        vals[(LOX_DECOUPLER_RETRACT_VALVE - BASE) as usize],
                    ));
                let _ = actuators
                    .n2_decoupler_extend_valve
                    .update(Energizable::from_active(
                        vals[(N2_DECOUPLER_EXTEND_VALVE - BASE) as usize],
                    ));
                let _ = actuators
                    .n2_decoupler_retract_valve
                    .update(Energizable::from_active(
                        vals[(N2_DECOUPLER_RETRACT_VALVE - BASE) as usize],
                    ));
                let _ = actuators.n2_shower_valve.update(Energizable::from_active(
                    vals[(N2_SHOWER_VALVE - BASE) as usize],
                ));
                let _ = actuators
                    .n2_engine_purge_valve
                    .update(Energizable::from_active(
                        vals[(N2_ENGINE_PURGE_VALVE - BASE) as usize],
                    ));
                let _ = actuators.fss_heating.update(Energizable::from_active(
                    vals[(HEATING_PADS_POWER - BASE) as usize],
                ));
            }
            Ok(Ok(Err(code))) => {
                tracing::warn!("modbus server read actuator coils to: {code}");
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
