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

        // Clean release: 1Hz with 90/10 duty cycle (1800ms on, 200ms off)
        // Otherwise: 2Hz with 50/50 duty cycle (125ms per half-period)
        let (on_duration, off_duration) = if is_release && !git_dirty {
            (
                std::time::Duration::from_millis(1800),
                std::time::Duration::from_millis(200),
            )
        } else {
            (
                std::time::Duration::from_millis(125),
                std::time::Duration::from_millis(125),
            )
        };

        loop {
            pin.set_high();
            std::thread::sleep(on_duration);
            pin.set_low();
            std::thread::sleep(off_duration);
        }
    })
}

/// Dummy status LED blink on non-linux. Does nothing.
#[cfg(not(target_os = "linux"))]
pub fn start_status_led(_is_release: bool, _git_dirty: bool) -> std::thread::JoinHandle<()> {
    std::thread::spawn(|| ())
}
