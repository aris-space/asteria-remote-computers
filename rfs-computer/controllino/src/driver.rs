//! This module serves to communicate with the Modbus server on the Controllino.
//!
//! Helper functions can poll data and write it into input trees.
//! Output data is directly written

use std::marker::PhantomData;
use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use datatypes::actuator::{Energizable, OutOfRange, Settable};
use datatypes::status::SensorStatus;
use datatypes::units::{BarG, Celsius};
use messages::datapoints::dp_rfs_computer::RfsComputerStatus;
use tokio::sync::Mutex as AsyncMutex;
use tokio_modbus::Address;
use tokio_modbus::client::{Context, Reader, Writer as _};
use watchable::Watchable;

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

#[derive(Clone)]
pub struct Client {
    /// Timeout for the Modbus requests using `poll_*` functions.
    /// Does not include locking time, thus if `context` stays locked, the functions will wait.
    timeout: Duration,
    /// Underlying connection to the Controllino
    context: Arc<AsyncMutex<Context>>,
    /// The IP address of the Controllino, usually `192.168.0.227:502`.
    socket_addr: SocketAddr,
    /// The shared variable where the controllino health should be stored in.
    health: Watchable<RfsComputerStatus>,
}

#[derive(Clone)]
pub struct CoilSettable<T> {
    coil: tokio_modbus::Address,
    driver: Client,
    _phantom: PhantomData<T>,
}

impl<T> CoilSettable<T> {
    pub fn new(driver: Client, coil: tokio_modbus::Address) -> Self {
        Self {
            coil,
            driver,
            _phantom: PhantomData,
        }
    }
}

impl<T: Energizable> CoilSettable<T> {
    #[allow(clippy::needless_pass_by_value)] // typically u8
    pub fn set(&self, value: T) {
        let high = value.is_active();

        // send to coil, need async, so spawn a task
        let context = self.driver.clone();
        let coil = self.coil;
        tokio::spawn(async move { context.write_coil(coil, high).await });
    }
}

impl<T: Default + Settable + Energizable> Settable for CoilSettable<T> {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        let mut typed = T::default();
        typed.set_value(value)?;
        self.set(typed);
        Ok(())
    }
}

impl Client {
    // / Creates a lazy client.
    // / Does not connect yet.
    // pub fn new(
    //     socket_addr: &str,
    //     timeout: Duration,
    //     health_callback: Watchable<RfsComputerStatus>,
    // ) -> std::io::Result<Self> {
    //     let socket_addr = singleton(socket_addr.to_socket_addrs()?).ok_or_else(|| {
    //         std::io::Error::other("Expected socket address to parse to exactly one element")
    //     })?;
    //     Ok(Self {
    //         timeout,
    //         context: Arc::default(),
    //         socket_addr,
    //         health: health_callback,
    //     })
    // }

    /// Connects to the modbus server.
    /// TODO: make it lazy and reconnectable.
    pub async fn connect(
        socket_addr: SocketAddr,
        timeout: Duration,
        health: Watchable<RfsComputerStatus>,
    ) -> std::io::Result<Self> {
        tracing::info!("Connecting to Controllino at {socket_addr}");
        let ctx =
            tokio::time::timeout(5 * timeout, tokio_modbus::client::tcp::connect(socket_addr))
                .await??;
        Ok(Self {
            timeout,
            context: Arc::new(AsyncMutex::new(ctx)),
            socket_addr,
            health,
        })
    }

    /// Starts tasks to periodically poll the sensors and actuator states
    pub fn start_polling(
        &self,
        pressure_polling_hz: f32,
        temperature_polling_hz: f32,
        digital_polling_hz: f32,
        actuator_polling_hz: f32,
        actuators: input::Actuators,
        sensors: &input::Sensors,
    ) {
        let driver = self.clone();
        tokio::spawn(async move {
            let period = Duration::from_secs_f32(1. / actuator_polling_hz);
            loop {
                tokio::time::sleep(period).await; // intentionally not `interval` to have natural random jitter.
                driver.poll_actuators(&actuators).await;
            }
        });

        let driver = self.clone();
        let sensor_tree = sensors.clone();
        tokio::spawn(async move {
            let period = Duration::from_secs_f32(1. / pressure_polling_hz);
            loop {
                tokio::time::sleep(period).await; // intentionally not `interval` to have natural random jitter.
                driver.poll_pressure_sensor(&sensor_tree).await;
            }
        });

        let driver = self.clone();
        let sensor_tree = sensors.clone();
        tokio::spawn(async move {
            let period = Duration::from_secs_f32(1. / temperature_polling_hz);
            loop {
                tokio::time::sleep(period).await; // intentionally not `interval` to have natural random jitter.
                driver.poll_temperature_sensors(&sensor_tree).await;
            }
        });

        let driver = self.clone();
        let sensor_tree = sensors.clone();
        tokio::spawn(async move {
            let period = Duration::from_secs_f32(1. / digital_polling_hz);
            loop {
                tokio::time::sleep(period).await; // intentionally not `interval` to have natural random jitter.
                driver.poll_discrete_inputs(&sensor_tree).await;
            }
        });
    }

    fn mark_error(&self) {
        let mut x = self.health.write();
        if x.controllino.online != SensorStatus::Offline {
            x.controllino.online = SensorStatus::Offline;
        }
    }

    fn mark_connected(&self) {
        let mut x = self.health.write();
        if x.controllino.online != SensorStatus::Online {
            x.controllino.online = SensorStatus::Online;
        }
    }

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
                let _ = actuators.rocket_heating.update(Energizable::from_active(
                    vals[(HEATING_PADS_POWER - COIL_BASE) as usize],
                ));
                self.mark_connected();
            }
            Ok(Ok(Err(code))) => {
                tracing::warn!("modbus server rejected actuator coil read: {code}");
                self.mark_connected(); // it responded, but weirdly
            }
            Ok(Err(err)) => {
                tracing::warn!("Cannot reach modbus server: {err}");
                self.mark_error();
            }
            Err(_elapsed) => {
                tracing::warn!("Timed out modbus server");
                self.mark_error();
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
                self.mark_connected();
            }
            Ok(Ok(Err(code))) => {
                tracing::warn!("modbus server rejected discrete input read: {code}");
                self.mark_connected(); // it responded, but weirdly
            }
            Ok(Err(err)) => {
                tracing::warn!("Cannot reach modbus server: {err}");
                self.mark_error();
            }
            Err(_elapsed) => {
                tracing::warn!("Timed out modbus server");
                self.mark_error();
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
                self.mark_connected();
            }
            Ok(Ok(Err(code))) => {
                tracing::warn!("modbus server rejected holding register read: {code}");
                self.mark_connected(); // it responded, but weirdly
            }
            Ok(Err(err)) => {
                tracing::warn!("Cannot reach modbus server: {err}");
                self.mark_error();
            }
            Err(_elapsed) => {
                tracing::warn!("Timed out modbus server");
                self.mark_error();
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
                self.mark_connected();
            }
            Ok(Ok(Err(code))) => {
                tracing::warn!("modbus server rejected holding register read: {code}");
                self.mark_connected(); // it responded, but weirdly
            }
            Ok(Err(err)) => {
                tracing::warn!("Cannot reach modbus server: {err}");
                self.mark_error();
            }
            Err(_elapsed) => {
                tracing::warn!("Timed out modbus server");
                self.mark_error();
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
                self.mark_error();
            }
            Err(_elapsed) => {
                tracing::warn!("Timed out modbus server");
                self.mark_error();
            }
        }
    }

    /// Make an output tree that directly talks to this driver
    pub fn into_output_tree(self) -> crate::output::Actuators {
        crate::output::Actuators {
            rocket_power: CoilSettable::new(self.clone(), addresses::ROCKET_POWER),
            rocket_heating: CoilSettable::new(self.clone(), addresses::LOX_DECOUPLER_EXTEND_VALVE),
            lox_decoupler_extend_valve: CoilSettable::new(
                self.clone(),
                addresses::LOX_DECOUPLER_RETRACT_VALVE,
            ),
            lox_decoupler_retract_valve: CoilSettable::new(
                self.clone(),
                addresses::N2_DECOUPLER_EXTEND_VALVE,
            ),
            n2_decoupler_extend_valve: CoilSettable::new(
                self.clone(),
                addresses::N2_DECOUPLER_RETRACT_VALVE,
            ),
            n2_decoupler_retract_valve: CoilSettable::new(self.clone(), addresses::N2_SHOWER_VALVE),
            n2_shower_valve: CoilSettable::new(self.clone(), addresses::N2_ENGINE_PURGE_VALVE),
            n2_engine_purge_valve: CoilSettable::new(self.clone(), addresses::HEATING_PADS_POWER),
        }
    }
}

fn decode_f32_from_holding_registers(data_hi: u16, data_lo: u16) -> f32 {
    f32::from_bits(u32::from(data_lo) | (u32::from(data_hi) << 16))
}
