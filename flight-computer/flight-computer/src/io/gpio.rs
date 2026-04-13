use watchable::Watchable;

use crate::io::types::{MainDeploymentState, SeparationState};

/// Reads the CATS pin at 5Hz and writes it to `cats_separation` and `cats_deployment`.
#[cfg(target_os = "linux")]
pub fn start_cats_driver(
    cats_separation: Watchable<SeparationState>,
    cats_deployment: Watchable<MainDeploymentState>,
) -> std::thread::JoinHandle<()> {
    const SEP_PIN: u8 = 2; // GPIO2 connected to CATS IO 1
    const DEPL_PIN: u8 = 3; // GPIO3 connected to CATS IO 2

    std::thread::spawn(move || {
        // Attempt to set up the GPIO pin, and handle errors appropriately
        let gpio = match rppal::gpio::Gpio::new() {
            Ok(g) => g,
            Err(e) => {
                tracing::error!("Failed to initialize GPIO: {e:?}");
                return;
            }
        };

        // Try to get the pins and set it as an input
        let sep_pin = match gpio.get(SEP_PIN) {
            Ok(pin) => pin.into_input_pullup(),
            Err(e) => {
                tracing::error!("Failed to access GPIO pin {SEP_PIN}: {e:?}");
                return;
            }
        };
        let depl_pin = match gpio.get(DEPL_PIN) {
            Ok(pin) => pin.into_input_pullup(),
            Err(e) => {
                tracing::error!("Failed to access GPIO pin {DEPL_PIN}: {e:?}");
                return;
            }
        };

        loop {
            // High by default, low once action occured
            let _ = cats_separation.update(if sep_pin.is_high() {
                SeparationState::NotYetSeparated
            } else {
                SeparationState::Separated
            });
            let _ = cats_deployment.update(if depl_pin.is_high() {
                MainDeploymentState::NotYetDeployed
            } else {
                MainDeploymentState::Deployed
            });

            // Sleep before the next check
            std::thread::sleep(std::time::Duration::from_millis(200));
        }
    })
}

/// Dummy on non-linux to read the CATS pin at 5Hz and writes it to `cats_separation` and `cats_deployment`.
/// Doesnt do anything
#[cfg(not(target_os = "linux"))]
pub fn start_cats_driver(
    _cats_separation: Watchable<SeparationState>,
    _cats_deployment: Watchable<MainDeploymentState>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(|| ())
}

/// Spawns a thread that blinks the status LED on GPIO 20.
/// Blinks at 1Hz if the build is release and clean, 2Hz otherwise.
#[cfg(target_os = "linux")]
pub fn start_status_led(is_release: bool, git_dirty: bool) -> std::thread::JoinHandle<()> {
    const LED_PIN: u8 = 20;

    std::thread::spawn(move || {
        let gpio = match rppal::gpio::Gpio::new() {
            Ok(g) => g,
            Err(e) => {
                tracing::error!("Failed to initialize GPIO for status LED: {e:?}");
                return;
            }
        };
        let mut pin = match gpio.get(LED_PIN) {
            Ok(p) => p.into_output(),
            Err(e) => {
                tracing::error!("Failed to access GPIO pin {LED_PIN}: {e:?}");
                return;
            }
        };

        // 1Hz (500ms per half-period) for clean release, 2Hz (250ms) otherwise
        let half_period = if is_release && !git_dirty {
            std::time::Duration::from_millis(500)
        } else {
            std::time::Duration::from_millis(250)
        };

        loop {
            pin.set_high();
            std::thread::sleep(half_period);
            pin.set_low();
            std::thread::sleep(half_period);
        }
    })
}

/// Dummy status LED blink on non-linux. Does nothing.
#[cfg(not(target_os = "linux"))]
pub fn start_status_led(_is_release: bool, _git_dirty: bool) -> std::thread::JoinHandle<()> {
    std::thread::spawn(|| ())
}
