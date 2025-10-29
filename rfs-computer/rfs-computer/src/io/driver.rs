use std::sync::Arc;
use std::time::Duration;
use std::vec;

use keller23sx::Keller23SX;
use lib_mission::actuator_control::Settable as _;
use lib_mission::reactive::{IntoWatched as _, Watched as _};
use lib_mission::util::types::BarG;
use lox_fill_valve::LoxValve;
use pss_drivers::types::ThrottleableValve;
use tokio::sync::Mutex;
use tokio::task::AbortHandle;
use tokio_modbus::Slave;
use tokio_modbus::client::{Context, rtu};
use tokio_modbus_shared::ModbusDevice;
use tokio_serial::SerialStream;
use tokio_serial_pacing::SerialPacing;
use tracing::{Instrument, instrument};
use uremote_electronics::driver::Driver;
use watchable::Watchable;

use crate::config::SerialConfig;
use crate::config::URemoteConfig;
use crate::io::input::{PressureSensors, SensorValues};

use super::input::InputTree;
use super::output::OutputTree;

/// The slave addresses of various devices on the shared serial modbus.
mod modbus_addresses {
    pub const N2_PRE_BOOSTER_SENSOR_ADDR: u8 = 2;
    pub const H2_PURGE_SENSOR_ADDR: u8 = 3;
    pub const LOX_SENSOR_ADDR: u8 = 4;
    pub const GOX_SENSOR_ADDR: u8 = 5;
    pub const PURGE_SENSOR_ADDR: u8 = 6;
    pub const PNEUMATIC_SENSOR_ADDR: u8 = 7;
    pub const N2_POST_BOOSTER_SENSOR_ADDR: u8 = 8;
    pub const H2_SENSOR_ADDR: u8 = 9;
    pub const IGNITER_SENSOR_ADDR: u8 = 10;

    pub const LOX_FILL_VALVE_ADDR: u8 = 101;
}

/// Sets up the drivers and spawns their tasks.
pub async fn init_drivers(
    ur_config: &URemoteConfig,
    serial_config: &SerialConfig,
    input_tree: &InputTree,
    output_tree: &OutputTree,
) -> Result<Vec<AbortHandle>, Box<dyn std::error::Error>> {
    let uremote_handle = init_uremote_driver(ur_config, input_tree, output_tree).await?;
    let serial_handles = init_serial_drivers(serial_config, input_tree, output_tree)?;

    let mut handles = serial_handles;
    handles.push(uremote_handle);
    Ok(handles)
}

/// Spawns a task to communicate with the u-remote
async fn init_uremote_driver(
    ur_config: &URemoteConfig,
    input_tree: &InputTree,
    output_tree: &OutputTree,
) -> Result<AbortHandle, Box<dyn std::error::Error + 'static>> {
    let mut uremote_driver = Driver::new(
        &ur_config.address,
        Duration::from_secs_f32(1. / ur_config.min_read_frequency_hz),
    )
    .await?;

    let input = input_tree.uremote_sensors.clone();
    let actuators_input = input_tree.uremote_actuators.clone();
    let output = output_tree.rfs_actuators.clone();
    let uremote_task = tokio::spawn(async move {
        uremote_driver.run(&input, &actuators_input, &output).await;
    });

    Ok(uremote_task.abort_handle())
}

/// Spawns tasks to communicate with devices on the serial bus
fn init_serial_drivers(
    serial_config: &SerialConfig,
    input_tree: &InputTree,
    output_tree: &OutputTree,
) -> Result<Vec<AbortHandle>, Box<dyn std::error::Error>> {
    // Set up the serial port
    let builder = tokio_serial::new(
        serial_config.serial_port.clone(),
        u32::from(serial_config.baud_rate),
    )
    .timeout(Duration::from_millis(serial_config.timeout_ms));

    let serial = match SerialStream::open(&builder) {
        Ok(serial) => serial,
        Err(e) => {
            tracing::error!("Error opening serial port: {e}");
            tracing::error!("Please check the device path and ensure it is not in use.");
            return Err(e.into());
        }
    };
    let wait_time = tokio_serial_pacing::wait_time(&serial);
    let mut port = tokio_serial_pacing::SerialWritePacing::from(serial);
    port.set_delay(wait_time);

    let modbus = rtu::attach(port);
    let shared_modbus = Arc::new(Mutex::new(modbus));

    // Spawn lox fill valve driver
    let handle = spawn_lox_fill_valve_driver(
        serial_config,
        &output_tree.lox_fill_valve,
        &input_tree.lox_fill_valve,
        &shared_modbus,
    );

    // Spawn pressure sensors tasks
    let mut handles = init_keller23sx_drivers(
        serial_config,
        &shared_modbus,
        input_tree.serial_pressure_sensors.clone(),
    )?;

    handles.extend(handle);
    Ok(handles)
}

/// Spawns tasks to talk with the lox fill valve board
fn spawn_lox_fill_valve_driver(
    serial_config: &SerialConfig,
    lox_fill_valve_target: &Watchable<ThrottleableValve>,
    lox_fill_valve_observed: &Watchable<ThrottleableValve>,
    shared_modbus: &Arc<Mutex<Context>>,
) -> [tokio::task::AbortHandle; 2] {
    let timeout_dur = Duration::from_millis(serial_config.timeout_ms);
    // Having two instances is fine with the abstraction
    let mut valve_writer = LoxValve::new(ModbusDevice::new(
        shared_modbus.clone(),
        Slave(modbus_addresses::LOX_FILL_VALVE_ADDR),
        timeout_dur,
    ));
    let mut valve_reader = LoxValve::new(ModbusDevice::new(
        shared_modbus.clone(),
        Slave(modbus_addresses::LOX_FILL_VALVE_ADDR),
        timeout_dur,
    ));

    // If device is disconnected, then it still at most consumes 1/11th of the bandwidth;
    let init_backoff = 10 * timeout_dur;
    let max_backoff = 100 * timeout_dur; // probably 10 sec, 1% bandwidth

    // Spawning output driver
    let mut lox_fill_valve_target = lox_fill_valve_target.watched();
    let handle_w = tokio::spawn(
        async move {
            let mut backoff = init_backoff;

            loop {
                // Await next value, but use the current value later while retrying.
                lox_fill_valve_target.next().await;

                while let Err(e) = valve_writer
                    .set_target_position(
                        f32::from(lox_fill_valve_target.current().openness_percent) / 100.,
                    )
                    .await
                {
                    tracing::warn!(error = %e, "failed to set openness");
                    tokio::time::sleep(backoff).await;
                    backoff = (2 * backoff).min(max_backoff);
                }
                backoff = init_backoff;
            }
        }
        .instrument(tracing::debug_span!("drivers::LoxFillValveWrite")),
    )
    .abort_handle();

    // Spawn input driver
    let lox_fill_valve_current = lox_fill_valve_observed.clone();
    let polling_period = Duration::from_secs_f32(1. / serial_config.valve_state_polling_hz);
    let handle_r = tokio::spawn(
        async move {
            let mut interval = tokio::time::interval(polling_period);
            let mut backoff = init_backoff;
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            loop {
                interval.tick().await;

                match valve_reader.read_current_position().await {
                    Ok(openness) => {
                        // Setting value is also 0-1
                        if lox_fill_valve_current.write().set_value(openness).is_err() {
                            tracing::warn!(openness, "lox valve read returned bad value");
                        }
                        backoff = init_backoff;
                    }
                    Err(e) => {
                        tracing::warn!(error = %e, "failed to read openness");

                        // stupid value to indicate the problem. FIXME I guess
                        let _ = lox_fill_valve_current.update(ThrottleableValve {
                            openness_percent: 255,
                        });

                        // Exponential backoff on erros, maybe due to device not connected
                        tokio::time::sleep(backoff).await;
                        backoff = (2 * backoff).min(max_backoff);
                    }
                }
            }
        }
        .instrument(tracing::debug_span!("drivers::LoxFillValveRead")),
    )
    .abort_handle();

    [handle_w, handle_r]
}

/// Initializes tasks to talk with the Keller 23SX pressure sensors.
pub fn init_keller23sx_drivers(
    serial_config: &SerialConfig,
    shared_modbus: &Arc<Mutex<Context>>,
    serial_input_values: PressureSensors,
) -> Result<Vec<AbortHandle>, Box<dyn std::error::Error>> {
    let polling_period = Duration::from_secs_f32(1. / serial_config.pressure_polling_hz);
    let timeout = Duration::from_millis(serial_config.timeout_ms);

    let PressureSensors {
        n2_pre_booster,
        h2_purge,
        n2_post_booster,
        lox,
        purge,
        pneumatic,
        gox,
        h2,
        igniter,
    } = serial_input_values;
    Ok(vec![
        spawn_individual_keller23sx_driver(
            shared_modbus.clone(),
            Slave(modbus_addresses::N2_PRE_BOOSTER_SENSOR_ADDR),
            n2_pre_booster,
            "n2_pre_booster",
            polling_period,
            timeout,
        )?,
        spawn_individual_keller23sx_driver(
            shared_modbus.clone(),
            Slave(modbus_addresses::H2_PURGE_SENSOR_ADDR),
            h2_purge,
            "h2_purge",
            polling_period,
            timeout,
        )?,
        spawn_individual_keller23sx_driver(
            shared_modbus.clone(),
            Slave(modbus_addresses::N2_POST_BOOSTER_SENSOR_ADDR),
            n2_post_booster,
            "n2_post_booster",
            polling_period,
            timeout,
        )?,
        spawn_individual_keller23sx_driver(
            shared_modbus.clone(),
            Slave(modbus_addresses::LOX_SENSOR_ADDR),
            lox,
            "lox",
            polling_period,
            timeout,
        )?,
        spawn_individual_keller23sx_driver(
            shared_modbus.clone(),
            Slave(modbus_addresses::PURGE_SENSOR_ADDR),
            purge,
            "purge",
            polling_period,
            timeout,
        )?,
        spawn_individual_keller23sx_driver(
            shared_modbus.clone(),
            Slave(modbus_addresses::PNEUMATIC_SENSOR_ADDR),
            pneumatic,
            "pneumatic",
            polling_period,
            timeout,
        )?,
        spawn_individual_keller23sx_driver(
            shared_modbus.clone(),
            Slave(modbus_addresses::GOX_SENSOR_ADDR),
            gox,
            "gox",
            polling_period,
            timeout,
        )?,
        spawn_individual_keller23sx_driver(
            shared_modbus.clone(),
            Slave(modbus_addresses::H2_SENSOR_ADDR),
            h2,
            "h2",
            polling_period,
            timeout,
        )?,
        spawn_individual_keller23sx_driver(
            shared_modbus.clone(),
            Slave(modbus_addresses::IGNITER_SENSOR_ADDR),
            igniter,
            "igniter",
            polling_period / 2, // Igniter sensor is more critical, it gets double to bandwidth by giving it half the period
            timeout,
        )?,
    ])
}

#[instrument(level = "INFO", name = "drivers::KellerPressureDriver", skip_all, fields(?address, sensor = _device_name))]
#[allow(clippy::used_underscore_binding)]
fn spawn_individual_keller23sx_driver(
    bus: Arc<Mutex<Context>>,
    address: Slave,
    input_values: SensorValues,
    _device_name: &str,
    polling_period: Duration,
    timeout: Duration,
) -> Result<AbortHandle, Box<dyn std::error::Error>> {
    let mut device = Keller23SX::new(ModbusDevice::new(bus, address, timeout));
    let device_task = tokio::spawn(
        async move {
            let device_serial_number = device.read_serial_number().await;
            match device_serial_number {
                Ok(device_serial_number) => {
                    tracing::info!(device_serial_number, "the serial number of the device");
                }
                e => {
                    tracing::warn!(errror = ?e, "Failed to read the serial number");
                }
            }

            let mut interval = tokio::time::interval(polling_period);
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            // If device is disconnected, then it still at most consumes 1/11th of the bandwidth;
            let init_backoff = 10 * timeout;
            let max_backoff = 100 * timeout; // probably 10 sec, 1% bandwidth
            let mut backoff = init_backoff;

            loop {
                interval.tick().await;

                match device.read_pressure_sensor1().await {
                    Ok(pressure) => {
                        let _ = input_values.pressure_value_1.update(BarG(pressure));
                        backoff = init_backoff;
                    }
                    Err(e) => {
                        tracing::warn!(
                            error = %e,
                            "Failed to read from pressure sensor"
                        );
                        let _ = input_values.pressure_value_1.update(BarG(f32::NAN));

                        // Exponential backoff on erros, maybe due to device not connected
                        tokio::time::sleep(backoff).await;
                        backoff = (2 * backoff).min(max_backoff);
                    }
                }
            }
        }
        .instrument(tracing::Span::current()),
    );

    Ok(device_task.abort_handle())
}
