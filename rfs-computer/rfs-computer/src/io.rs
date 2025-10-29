pub mod can;
pub mod csv_log;
pub mod driver;
pub mod input;
pub mod output;
pub mod telemetry;

/// Spawns a driver to set the clock based on the system clock at `1 / tick_period.as_secs()` Hz.
pub fn spawn_system_time_driver(
    clock: watchable::Watchable<lib_mission::util::time::MissionTime>,
    tick_period: std::time::Duration,
) -> tokio::task::AbortHandle {
    // make sure that the time is updated to a sensible value immediately.
    clock.replace(lib_mission::util::time::MissionTime::from(
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("time moved backwards"),
    ));

    tokio::spawn(async move {
        loop {
            tokio::time::sleep(tick_period).await;
            clock.replace(lib_mission::util::time::MissionTime::from(
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .expect("time moved backwards"),
            ));
        }
    })
    .abort_handle()
}
