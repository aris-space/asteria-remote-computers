//! This module serves to communicate with the Modbus server on the Controllino.
//!
//! Helper functions poll data and return messages, which callers dispatch into input trees
//! via the [`Collector`] trait. Output data is written directly to the Modbus server.

use std::marker::PhantomData;
use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use datatypes::actuator::{Energizable, OutOfRange, Settable};
use datatypes::status::SensorStatus;
use datatypes::units::{BarG, Celsius};
use io_utils::Collector;
use messages::Message;
use messages::datapoints::dp_rfs_computer::Message as RFSCMessage;
use messages::datapoints::dp_rfs_computer::RfsComputerStatus;
use tokio::sync::Mutex as AsyncMutex;
use tokio::sync::mpsc;
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
    /// Underlying connection to the Controllino. `None` when disconnected; re-established lazily.
    context: Arc<AsyncMutex<Option<Context>>>,
    /// The IP address of the Controllino, usually `192.168.0.227:502`.
    socket_addr: SocketAddr,
    /// The shared variable where the controllino health should be stored in.
    health: Watchable<RfsComputerStatus>,
}

#[derive(Clone)]
pub struct CoilSettable<T> {
    coil: tokio_modbus::Address,
    driver: Client,
    #[allow(clippy::type_complexity)] // okay enough: it is (Channel, Message Mapping)
    log: Option<(mpsc::Sender<Message>, fn(T) -> Message)>,
    _phantom: PhantomData<T>,
}

impl<T> CoilSettable<T> {
    fn new(driver: Client, coil: tokio_modbus::Address) -> Self {
        Self {
            coil,
            driver,
            log: None,
            _phantom: PhantomData,
        }
    }

    fn new_with_log(
        driver: Client,
        coil: tokio_modbus::Address,
        log_tx: mpsc::Sender<Message>,
        to_msg: fn(T) -> Message,
    ) -> Self {
        Self {
            coil,
            driver,
            log: Some((log_tx, to_msg)),
            _phantom: PhantomData,
        }
    }
}

impl<T: Energizable + Clone> CoilSettable<T> {
    #[allow(clippy::needless_pass_by_value)] // typically u8
    pub fn set(&self, value: T) {
        if let Some((tx, to_msg)) = &self.log {
            let _ = tx.try_send(to_msg(value.clone()));
        }
        let high = value.is_active();

        // send to coil, need async, so spawn a task
        let context = self.driver.clone();
        let coil = self.coil;
        tokio::spawn(async move { context.write_coil(coil, high).await });
    }
}

impl<T: Default + Settable + Energizable + Clone> Settable for CoilSettable<T> {
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        let mut typed = T::default();
        typed.set_value(value)?;
        self.set(typed);
        Ok(())
    }
}

impl Client {
    /// Creates a client without connecting. The TCP connection is established lazily on first use
    /// and re-established automatically after any IO error.
    pub fn new(
        socket_addr: SocketAddr,
        timeout: Duration,
        health: Watchable<RfsComputerStatus>,
    ) -> Self {
        Self {
            timeout,
            context: Arc::new(AsyncMutex::new(None)),
            socket_addr,
            health,
        }
    }

    /// Attempts a single TCP connection to the Controllino within `self.timeout`.
    async fn try_connect(&self) -> Option<Context> {
        match tokio::time::timeout(
            5 * self.timeout, // longer timeout than single request timeout
            tokio_modbus::client::tcp::connect(self.socket_addr),
        )
        .await
        {
            Ok(Ok(ctx)) => {
                tracing::info!("Connected to Controllino at {}", self.socket_addr);
                self.mark_connected();
                Some(ctx)
            }
            Ok(Err(err)) => {
                tracing::warn!("Failed to connect to Controllino: {err}");
                self.mark_error();
                None
            }
            Err(_elapsed) => {
                tracing::warn!("Timed out connecting to Controllino");
                self.mark_error();
                None
            }
        }
    }

    /// Acquires the context (reconnecting if needed), runs `f` under a timeout, and handles the
    /// two common error arms (IO error and timeout), resetting the connection on both.
    /// Returns `None` when the operation failed; `Some(inner)` is the modbus result for callers
    /// to interpret (either `Ok(T)` or `Err(exception code)`).
    async fn with_context<T, E>(
        &self,
        f: impl AsyncFnOnce(&mut Context) -> Result<Result<T, E>, tokio_modbus::Error>,
    ) -> Option<Result<T, E>> {
        let mut guard = self.context.lock().await;
        let ctx = match guard.as_mut() {
            Some(ctx) => ctx,
            None => match self.try_connect().await {
                Some(ctx) => {
                    *guard = Some(ctx);
                    guard.as_mut().expect("just set above")
                }
                None => return None,
            },
        };

        match tokio::time::timeout(self.timeout, f(ctx)).await {
            Ok(Ok(inner)) => Some(inner),
            Ok(Err(err)) => {
                tracing::warn!("Cannot reach modbus server: {err}");
                *guard = None;
                self.mark_error();
                None
            }
            Err(_elapsed) => {
                tracing::warn!("Timed out modbus server");
                *guard = None;
                self.mark_error();
                None
            }
        }
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

    /// Starts tasks to periodically poll the sensors and actuator states, dispatching results
    /// into the input trees and optionally forwarding each message to `log`.
    #[allow(clippy::too_many_arguments)] // its fine
    pub fn start_polling(
        &self,
        pressure_polling_hz: f32,
        temperature_polling_hz: f32,
        digital_polling_hz: f32,
        actuator_polling_hz: f32,
        actuators: input::Actuators,
        sensors: &input::Sensors,
        log: Option<mpsc::Sender<Message>>,
    ) {
        /// Sends all messages to both the input tree and the log
        fn dispatch(
            msgs: Vec<Message>,
            tree: &mut impl Collector<Message>,
            log: Option<&mpsc::Sender<Message>>,
        ) {
            for msg in msgs {
                if let Some(tx) = log {
                    let res = tx.try_send(msg.clone());
                    if let Err(e) = res {
                        tracing::warn!("dropping log: {e}");
                    }
                }
                let _ = tree.update_from(msg);
            }
        }

        let driver = self.clone();
        let log_a = log.clone();
        tokio::spawn(async move {
            let mut actuators = actuators;
            let period = Duration::from_secs_f32(1. / actuator_polling_hz);
            loop {
                tokio::time::sleep(period).await; // intentionally not `interval` to have natural random jitter.
                let msgs = driver.poll_actuators().await;
                dispatch(msgs, &mut actuators, log_a.as_ref());
            }
        });

        let driver = self.clone();
        let mut pressure_sensors = sensors.clone();
        let log_p = log.clone();
        tokio::spawn(async move {
            let period = Duration::from_secs_f32(1. / pressure_polling_hz);
            loop {
                tokio::time::sleep(period).await; // intentionally not `interval` to have natural random jitter.
                let msgs = driver.poll_pressure_sensor().await;
                dispatch(msgs, &mut pressure_sensors, log_p.as_ref());
            }
        });

        let driver = self.clone();
        let mut temp_sensors = sensors.clone();
        let log_t = log.clone();
        tokio::spawn(async move {
            let period = Duration::from_secs_f32(1. / temperature_polling_hz);
            loop {
                tokio::time::sleep(period).await; // intentionally not `interval` to have natural random jitter.
                let msgs = driver.poll_temperature_sensors().await;
                dispatch(msgs, &mut temp_sensors, log_t.as_ref());
            }
        });

        let driver = self.clone();
        let mut digital_sensors = sensors.clone();
        let log_d = log;
        tokio::spawn(async move {
            let period = Duration::from_secs_f32(1. / digital_polling_hz);
            loop {
                tokio::time::sleep(period).await; // intentionally not `interval` to have natural random jitter.
                let msgs = driver.poll_discrete_inputs().await;
                dispatch(msgs, &mut digital_sensors, log_d.as_ref());
            }
        });
    }

    /// Reads all actuator states from the modbus server at once and returns them as messages.
    pub async fn poll_actuators(&self) -> Vec<Message> {
        let Some(result) = self
            .with_context(async |ctx| {
                ctx.read_coils(addresses::COIL_BASE, addresses::COIL_QUANTITY)
                    .await
            })
            .await
        else {
            return vec![];
        };
        match result {
            Err(code) => {
                tracing::warn!("modbus server rejected actuator coil read: {code}");
                self.mark_connected(); // it responded, but weirdly
                vec![]
            }
            Ok(vals) => {
                #[allow(clippy::wildcard_imports)] // scoped finely enough here
                use addresses::*;
                if vals.len() != COIL_QUANTITY as usize {
                    tracing::warn!(
                        "modbus server didn't return all coils: {count}",
                        count = vals.len()
                    );
                    return vec![];
                }
                self.mark_connected();
                let mut msgs = Vec::with_capacity(COIL_QUANTITY as usize);
                msgs.push(Message::RfsComputer(RFSCMessage::RocketPowerState(
                    Energizable::from_active(vals[(ROCKET_POWER - COIL_BASE) as usize]),
                )));
                msgs.push(Message::RfsComputer(
                    RFSCMessage::LoxDecouplerExtendValveState(Energizable::from_active(
                        vals[(LOX_DECOUPLER_EXTEND_VALVE - COIL_BASE) as usize],
                    )),
                ));
                msgs.push(Message::RfsComputer(
                    RFSCMessage::LoxDecouplerRetractValveState(Energizable::from_active(
                        vals[(LOX_DECOUPLER_RETRACT_VALVE - COIL_BASE) as usize],
                    )),
                ));
                msgs.push(Message::RfsComputer(
                    RFSCMessage::N2DecouplerExtendValveState(Energizable::from_active(
                        vals[(N2_DECOUPLER_EXTEND_VALVE - COIL_BASE) as usize],
                    )),
                ));
                msgs.push(Message::RfsComputer(
                    RFSCMessage::N2DecouplerRetractValveState(Energizable::from_active(
                        vals[(N2_DECOUPLER_RETRACT_VALVE - COIL_BASE) as usize],
                    )),
                ));
                msgs.push(Message::RfsComputer(RFSCMessage::N2ShowerValvesState(
                    Energizable::from_active(vals[(N2_SHOWER_VALVE - COIL_BASE) as usize]),
                )));
                msgs.push(Message::RfsComputer(RFSCMessage::N2RocketPurgeValveState(
                    Energizable::from_active(vals[(N2_ENGINE_PURGE_VALVE - COIL_BASE) as usize]),
                )));
                msgs.push(Message::RfsComputer(RFSCMessage::RocketHeatingState(
                    Energizable::from_active(vals[(HEATING_PADS_POWER - COIL_BASE) as usize]),
                )));
                msgs
            }
        }
    }

    /// Reads all digital inputs from the modbus server at once and returns them as messages.
    pub async fn poll_discrete_inputs(&self) -> Vec<Message> {
        let Some(result) = self
            .with_context(async |ctx| {
                ctx.read_discrete_inputs(
                    addresses::DISCRETE_IN_BASE,
                    addresses::DISCRETE_IN_QUANTITY,
                )
                .await
            })
            .await
        else {
            return vec![];
        };
        match result {
            Err(code) => {
                tracing::warn!("modbus server rejected discrete input read: {code}");
                self.mark_connected(); // it responded, but weirdly
                vec![]
            }
            Ok(vals) => {
                #[allow(clippy::wildcard_imports)] // scoped finely enough here
                use addresses::*;
                if vals.len() != DISCRETE_IN_QUANTITY as usize {
                    tracing::warn!(
                        "modbus server didn't return all discrete inputs: {count}",
                        count = vals.len()
                    );
                    return vec![];
                }
                self.mark_connected();
                vec![
                    Message::RfsComputer(RFSCMessage::ValvesArmedSmartBox(
                        Energizable::from_active(
                            vals[(VALVE_POWER_INTERLOCK - DISCRETE_IN_BASE) as usize],
                        ),
                    )),
                    Message::RfsComputer(RFSCMessage::RocketConnected(Energizable::from_active(
                        vals[(ROCKET_CONNECTION_DETECT - DISCRETE_IN_BASE) as usize],
                    ))),
                    Message::RfsComputer(RFSCMessage::LoxDecouplerProximity(
                        Energizable::from_active(
                            vals[(LOX_DECOUPLER_PROXIMITY - DISCRETE_IN_BASE) as usize],
                        ),
                    )),
                    Message::RfsComputer(RFSCMessage::N2DecouplerProximity(
                        Energizable::from_active(
                            vals[(N2_DECOUPLER_PROXIMITY - DISCRETE_IN_BASE) as usize],
                        ),
                    )),
                ]
            }
        }
    }

    /// Reads the analog input for the pressure sensor from the modbus server.
    ///
    /// This is separate, as it needs to be called more frequently.
    pub async fn poll_pressure_sensor(&self) -> Vec<Message> {
        let Some(result) = self
            .with_context(async |ctx| {
                ctx.read_holding_registers(
                    addresses::IGNITER_PRESSURE_BASE,
                    addresses::HOLDING_REGISTER_FLOAT_LEN,
                )
                .await
            })
            .await
        else {
            return vec![];
        };
        match result {
            Err(code) => {
                tracing::warn!("modbus server rejected holding register read: {code}");
                self.mark_connected(); // it responded, but weirdly
                vec![]
            }
            Ok(vals) => {
                #[allow(clippy::wildcard_imports)] // scoped finely enough here
                use addresses::*;
                if vals.len() != HOLDING_REGISTER_FLOAT_LEN as usize {
                    tracing::warn!(
                        "modbus server didn't return all words of the pressure sensor: {count}",
                        count = vals.len()
                    );
                    return vec![];
                }
                self.mark_connected();
                vec![Message::RfsComputer(RFSCMessage::IgniterPressure(BarG(
                    decode_f32_from_holding_registers(
                        vals[HOLDING_REGISTER_FLOAT_HI_OFFSET as usize],
                        vals[HOLDING_REGISTER_FLOAT_LO_OFFSET as usize],
                    ),
                )))]
            }
        }
    }

    /// Reads all analog inputs for the temperature sensors from the modbus server at once.
    pub async fn poll_temperature_sensors(&self) -> Vec<Message> {
        let Some(result) = self
            .with_context(async |ctx| {
                ctx.read_holding_registers(
                    addresses::IGNITER_TEMPERATURE_BASE,
                    addresses::TEMPERATURE_QUANTITY * addresses::HOLDING_REGISTER_FLOAT_LEN,
                )
                .await
            })
            .await
        else {
            return vec![];
        };
        match result {
            Err(code) => {
                tracing::warn!("modbus server rejected holding register read: {code}");
                self.mark_connected(); // it responded, but weirdly
                vec![]
            }
            Ok(vals) => {
                #[allow(clippy::wildcard_imports)] // scoped finely enough here
                use addresses::*;
                if vals.len() != (TEMPERATURE_QUANTITY * HOLDING_REGISTER_FLOAT_LEN) as usize {
                    tracing::warn!(
                        "modbus server didn't return all words of the temperature sensors: {count}",
                        count = vals.len()
                    );
                    return vec![];
                }
                self.mark_connected();
                vec![
                    Message::RfsComputer(RFSCMessage::IgniterTemperature(Celsius(
                        decode_f32_from_holding_registers(
                            vals[HOLDING_REGISTER_FLOAT_HI_OFFSET as usize],
                            vals[HOLDING_REGISTER_FLOAT_LO_OFFSET as usize],
                        ),
                    ))),
                    Message::RfsComputer(RFSCMessage::SmartBoxTemperature(Celsius(
                        decode_f32_from_holding_registers(
                            vals[((SMART_BOX_INTERNAL_TEMPERATURE_BASE - IGNITER_TEMPERATURE_BASE)
                                + HOLDING_REGISTER_FLOAT_HI_OFFSET)
                                as usize],
                            vals[((SMART_BOX_INTERNAL_TEMPERATURE_BASE - IGNITER_TEMPERATURE_BASE)
                                + HOLDING_REGISTER_FLOAT_LO_OFFSET)
                                as usize],
                        ),
                    ))),
                ]
            }
        }
    }

    /// Writes a single coil at given address to state `high`.
    pub async fn write_coil(&self, coil: Address, high: bool) {
        let Some(result) = self
            .with_context(async |ctx| ctx.write_single_coil(coil, high).await)
            .await
        else {
            return;
        };
        match result {
            Ok(()) => tracing::debug!("Set coil {coil:x} to {high}"),
            Err(code) => tracing::warn!("modbus server denied coil write to {coil:x}: {code}"),
        }
    }

    /// Make an output tree that directly talks to this driver.
    /// Pass `log` to record every actuator command as a message.
    pub fn into_output_tree(self, log: &Option<mpsc::Sender<Message>>) -> crate::output::Actuators {
        macro_rules! coil {
            ($addr:expr, $variant:path) => {
                match log.as_ref() {
                    Some(tx) => CoilSettable::new_with_log(
                        self.clone(),
                        $addr,
                        tx.clone(),
                        (|v| Message::RfsComputer($variant(v))) as fn(_) -> Message,
                    ),
                    None => CoilSettable::new(self.clone(), $addr),
                }
            };
        }
        crate::output::Actuators {
            rocket_power: coil!(addresses::ROCKET_POWER, RFSCMessage::RocketPowerState),
            rocket_heating: coil!(
                addresses::HEATING_PADS_POWER,
                RFSCMessage::RocketHeatingState
            ),
            lox_decoupler_extend_valve: coil!(
                addresses::LOX_DECOUPLER_EXTEND_VALVE,
                RFSCMessage::LoxDecouplerExtendValveState
            ),
            lox_decoupler_retract_valve: coil!(
                addresses::LOX_DECOUPLER_RETRACT_VALVE,
                RFSCMessage::LoxDecouplerRetractValveState
            ),
            n2_decoupler_extend_valve: coil!(
                addresses::N2_DECOUPLER_EXTEND_VALVE,
                RFSCMessage::N2DecouplerExtendValveState
            ),
            n2_decoupler_retract_valve: coil!(
                addresses::N2_DECOUPLER_RETRACT_VALVE,
                RFSCMessage::N2DecouplerRetractValveState
            ),
            n2_shower_valve: coil!(addresses::N2_SHOWER_VALVE, RFSCMessage::N2ShowerValvesState),
            n2_engine_purge_valve: coil!(
                addresses::N2_ENGINE_PURGE_VALVE,
                RFSCMessage::N2RocketPurgeValveState
            ),
        }
    }
}

fn decode_f32_from_holding_registers(data_hi: u16, data_lo: u16) -> f32 {
    f32::from_bits(u32::from(data_lo) | (u32::from(data_hi) << 16))
}
