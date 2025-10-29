#![allow(clippy::uninlined_format_args)]
#![allow(clippy::similar_names)]

use keller23sx::Keller23SX;
use std::io::{self, Write};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use tokio::time::{Instant, sleep, sleep_until, timeout};
use tokio_modbus::Slave;
use tokio_modbus::client::rtu;
use tokio_modbus_shared::ModbusDevice;
use tokio_serial::SerialStream;
use tokio_serial_pacing::SerialPacing;

#[tokio::main(flavor = "current_thread")]
#[allow(clippy::too_many_lines)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// We are working with 23SX with are class.group 5.24, giving a T1 of max 100, plus some headroom for transmission.
    const TIMEOUT: Duration = Duration::from_millis(105);

    // On the RFS, the Keller23SX sensors are connected to the serial port /dev/ttyAMA2.
    // By default, the sensors are configured with a baud rate of 9600, though this can and has
    // been changed to 115200 on the RFS.
    let builder = tokio_serial::new("/dev/ttyAMA2", 115_200).timeout(Duration::from_millis(100));
    let serial = SerialStream::open(&builder)?;

    // This is important, because modbus requires a 3.5 character delay to separate frames.
    // tokio-modbus DOES NOT handle this automatically, so we use the `tokio_serial_pacing`
    // to implement the slowdown for us.
    let wait_time = tokio_serial_pacing::wait_time(&serial);
    let mut port = tokio_serial_pacing::SerialWritePacing::from(serial);
    port.set_delay(wait_time);

    // Attach the Modbus RTU client to the serial port. If written like this, this will use the
    // default slave address of 0x01.
    // We don't specify a slave address here however, because we will use the modbus through the
    // shared ModbusDevice abstraction, which manages the slave address internally.
    let modbus = rtu::attach(port);

    // We can also invoke, which sets the slave address. This is only
    // necessary if we don't use the shared ModbusDevice abstraction.
    // ```
    // let modbus = rtu::attach_slave(port, Slave(0x2));
    // ```

    // Sensor might have been in use right before we (re)started the program, so
    // we also need to insert a small delay here.
    sleep(Duration::from_millis(5)).await;

    // Create a shared ModbusDevice instance using an Arc and Mutex. This will allow use to share
    // the bus across multiple Keller23SX instances.
    let shared_modbus = Arc::new(Mutex::new(modbus));

    // These two devices now share the same Modbus, but have different slave addresses, that
    // are automatically managed.
    let new_for_id =
        |id: u8| Keller23SX::new(ModbusDevice::new(shared_modbus.clone(), Slave(id), TIMEOUT));
    let rpn1 = new_for_id(0x2);
    let rpn2 = new_for_id(0x3);
    let rpl1 = new_for_id(0x4);
    let ipg1 = new_for_id(0x5);
    let rpp1 = new_for_id(0x6);
    let rpp2 = new_for_id(0x7);
    let rpn3 = new_for_id(0x8);
    let iph1 = new_for_id(0x9);

    let mut sensors = vec![
        ("RPN1", rpn1, 0x2),
        ("RPN2", rpn2, 0x3),
        ("RPL1", rpl1, 0x4),
        ("IPG1", ipg1, 0x5),
        ("RPP1", rpp1, 0x6),
        ("RPP2", rpp2, 0x7),
        ("RPN3", rpn3, 0x8),
        ("IPH1", iph1, 0x9),
    ];

    let mut tick_counter = 0;

    loop {
        let next_tick = Instant::now() + Duration::from_millis(100);

        let mut readings = Vec::new();

        // FIXME: timeouts are now handled internally.
        for (name, sensor, slave_id) in &mut sensors {
            match timeout(
                Duration::from_millis(100),
                sensor.read_pressure_temperature_sensor_1(),
            )
            .await
            {
                Ok(Ok((pressure, temperature))) => {
                    readings.push((*slave_id, name, Some((pressure, temperature))));
                }
                Ok(Err(_)) | Err(_) => {
                    // Error or timeout occurred
                    readings.push((*slave_id, name, None));
                }
            }
        }

        // Clear the console and print the header
        print!("\x1B[2J\x1B[H");
        io::stdout().flush().expect("Failed to flush stdout");

        let indicators = ["○", "◔", "◑", "◕", "●", "◕", "◑", "◔"];
        let indicator = indicators[tick_counter % indicators.len()];

        println!("╔═════════════════════════════════════════════╗");
        println!(
            "║               SENSOR READINGS {}             ║",
            indicator
        );
        println!("╠═════════════════════════════════════════════╣");

        for (slave_id, name, reading) in &readings {
            if let Some((pressure, temperature)) = reading {
                println!(
                    "║ [{:02}] {:<8} | {:>8.3} barg | {:>8.2} °C ║",
                    slave_id, name, pressure, temperature
                );
            } else {
                println!(
                    "║ [{:02}] {:<8} | {:^27} ║",
                    slave_id, name, "ERROR - No reading"
                );
            }
        }

        println!("╚═════════════════════════════════════════════╝");
        println!("\nPress Ctrl+C to exit...");

        tick_counter += 1;
        sleep_until(next_tick).await;
    }
}
