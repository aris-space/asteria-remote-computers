#![allow(clippy::uninlined_format_args)]

use keller23sx::Keller23SX;
use std::time::Duration;
use tokio::time::sleep_until;
use tokio_modbus::Slave;
use tokio_modbus::client::rtu;
use tokio_serial::SerialStream;
use tokio_serial_pacing::SerialPacing;

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // On the RFS, the Keller23SX sensors are connected to the serial port /dev/ttyS0.
    // By default, the sensors are configured with a baud rate of 9600, though this can and has
    // been changed to 115200 on the RFS.
    let builder = tokio_serial::new("/dev/ttyS0", 115_200).timeout(Duration::from_millis(500));
    let serial = SerialStream::open(&builder)?;

    // This is important, because modbus requires a 3.5 character delay to separate frames.
    // tokio-modbus DOES NOT handle this automatically, so we use the `tokio_serial_pacing`
    // to implement the slowdown for us.
    let wait_time = tokio_serial_pacing::wait_time(&serial);
    let mut port = tokio_serial_pacing::SerialWritePacing::from(serial);
    port.set_delay(wait_time);

    // Attach the Modbus RTU client to the serial port with a specific slave address. Useful
    // when you have a single sensor, that exclusively uses the Modbus interface.
    // See `multiple.rs` for an example of how to use a shared Modbus interface instead.
    let modbus = rtu::attach_slave(port, Slave(0x02));
    let mut sensor = Keller23SX::new(modbus);

    println!("====== Device Info ======");

    // Sensor might have been in use right before we (re)started the program, so
    // we also need to insert a small delay here.

    if let Ok(serial) = sensor.read_serial_number().await {
        println!("Serial: {}", serial);
    }

    if let Ok(addr) = sensor.read_device_address().await {
        println!("Address: {}", addr);
    }

    if let Ok(fw) = sensor.read_firmware_class_group().await {
        println!("Firmware: {}", fw);
    }

    if let Ok(date) = sensor.read_firmware_date().await {
        println!("Build Date: {}", date);
    }

    if let Ok(pressure_flags) = sensor.read_pressure_channel_flags().await {
        println!("Pressure Flags: {:?}", pressure_flags);
    }

    if let Ok(temp_flags) = sensor.read_temperature_channel_flags().await {
        println!("Temperature Flags: {:?}", temp_flags);
    }

    if let Ok(uart) = sensor.read_uart_settings().await {
        println!(
            "UART: baud={:?}, parity={:?}, stop={:?}",
            uart.baud_rate, uart.parity, uart.stop_bits
        );
    }

    if let Ok(status) = sensor.read_status().await {
        println!("Status: {:?}", status);
    }

    println!("\n====== Readout Loop ======");

    loop {
        let next_tick = tokio::time::Instant::now() + Duration::from_millis(100);

        if let Ok(pressure) = sensor.read_pressure_sensor1().await {
            println!("Pressure: {:.3} bar", pressure);
        }

        if let Ok(temp) = sensor.read_temperature_outside_sensor_1().await {
            println!("Temperature: {:.2} °C", temp);
        }

        if let Ok((pressure, temperature)) = sensor.read_pressure_temperature_sensor_1().await {
            println!(
                "Multi Read Pressure: {:.3} bar, Temperature: {:.2} °C",
                pressure, temperature
            );
        }

        // wait until next measurement is due
        sleep_until(next_tick).await;
    }
}
