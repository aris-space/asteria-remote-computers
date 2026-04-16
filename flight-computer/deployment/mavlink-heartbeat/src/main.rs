//! This script attempts to connect to a localhost mavlink router,
//! and controls the green LED based on this.

use std::net::TcpStream;
use std::thread::sleep;
use std::time::Duration;

use mavio::dialects::minimal as dialect;
use mavio::dialects::minimal::enums::{MavAutopilot, MavModeFlag, MavState, MavType};
use mavio::io::StdIoWriter;
use mavio::{Dialect, Endpoint, MavLinkId, Sender};
use rppal::gpio::Gpio;

const GPIO_PIN: u8 = 21;
const MAVLINK_ADDR: &str = "127.0.0.1:5766";
const SYSTEM_ID: u8 = 1;
const COMPONENT_ID: u8 = 2;

fn blink(pin: &mut rppal::gpio::OutputPin, pattern: &[u64]) {
    let mut high = true;
    for &ms in pattern {
        if high {
            pin.set_high()
        } else {
            pin.set_low()
        }
        sleep(Duration::from_millis(ms));
        high = !high;
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut pin = Gpio::new()?.get(GPIO_PIN)?.into_output();
    let endpoint = Endpoint::v2(MavLinkId::new(SYSTEM_ID, COMPONENT_ID));
    let msg = dialect::messages::Heartbeat {
        type_: MavType::OnboardController,
        autopilot: MavAutopilot::Invalid,
        base_mode: MavModeFlag::default(),
        custom_mode: 0,
        system_status: MavState::Active,
        mavlink_version: dialect::Minimal::version().unwrap(),
    };

    loop {
        match TcpStream::connect(MAVLINK_ADDR) {
            Ok(stream) => {
                let mut sender = Sender::new(StdIoWriter::new(stream));
                loop {
                    match endpoint.next_frame(&msg) {
                        Ok(frame) if sender.send(&frame).is_ok() => blink(&mut pin, &[900, 100]),
                        _ => break,
                    }
                }
            }
            Err(_) => blink(&mut pin, &[250, 250]),
        }
    }
}
