#![allow(clippy::uninlined_format_args)]

use lox_fill_valve::LoxValve;
use std::time::Duration;
use tokio::time::{sleep, timeout};
use tokio_modbus::Slave;
use tokio_modbus::client::rtu;
use tokio_serial::SerialStream;
use tokio_serial_pacing::SerialPacing;

const LOX_VALVE_SLAVE_ID: u8 = 101;
const SERIAL_PORT: &str = "/dev/ttyS0";
const BAUD_RATE: u32 = 115_200;

const OP_TIMEOUT: Duration = Duration::from_secs(2); // per-try timeout
const RETRY_DELAY: Duration = Duration::from_millis(200); // back-off

const POSITIONS: &[f32] = &[0.0, 0.25, 0.50, 0.75, 1.0, 0.33, 1.0, 0.1, 0.7]; // sequence to run
const MOVE_CHECKS: usize = 2; // how many polls/step
const CHECK_INTERVAL: Duration = Duration::from_millis(500); // between polls

macro_rules! retry_op {
    ($expr:expr) => {{
        loop {
            match timeout(OP_TIMEOUT, async { $expr }).await {
                Ok(Ok(v)) => break v,
                Ok(Err(e)) => eprintln!("    Error: {e}; retrying…"),
                Err(_) => eprintln!("    Timeout; retrying…"),
            }
            sleep(RETRY_DELAY).await;
        }
    }};
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let builder = tokio_serial::new(SERIAL_PORT, BAUD_RATE).timeout(Duration::from_millis(500));
    let port = match SerialStream::open(&builder) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Error opening {}: {e}", SERIAL_PORT);
            return Ok(());
        }
    };

    let mut port: tokio_serial_pacing::SerialWritePacing<_> = port.into();
    port.set_delay(Duration::from_millis(3));

    let modbus = rtu::attach_slave(port, Slave(LOX_VALVE_SLAVE_ID));
    let mut valve = LoxValve::new(modbus);

    println!("====== LoxValve Extended Demo ======");

    for &target in POSITIONS {
        println!("\n--> Commanding valve to {:>5.1}% …", target * 100.0);

        retry_op!(valve.set_target_position(target).await);
        println!("    Command accepted; polling position…");

        for n in 1..=MOVE_CHECKS {
            let cur_pos = retry_op!(valve.read_current_position().await);
            let tgt_pos = retry_op!(valve.read_target_position().await);
            println!(
                "      ✔ Check {}/{} — target {:>5.1}%, current {:>5.1}%",
                n,
                MOVE_CHECKS,
                tgt_pos * 100.0,
                cur_pos * 100.0
            );
            sleep(CHECK_INTERVAL).await;
        }
    }

    println!("\n--> Returning valve to 0 % (closed) …");
    retry_op!(valve.set_target_position(0.0).await);
    println!("Done. 👍");

    Ok(())
}
