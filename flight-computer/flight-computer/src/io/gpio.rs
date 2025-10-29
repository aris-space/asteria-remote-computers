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
