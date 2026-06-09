use std::path::Path;

use lib_mission::reactive::{IntoWatched as _, WatchedExt as _};
use lib_mission::util::log::{make_timeseries_writer, spawn_watchable_logger};
use lib_mission::util::time::MissionTime;
use tokio::task::AbortHandle;
use watchable::Watchable;

use super::input::InputTree;
use super::output::OutputTree;

impl InputTree {
    /// Starts logging input tree timeseries to csv files in `folder`.
    /// Used for both database import and simulation replays.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    #[allow(clippy::too_many_lines)] // This is ugly, but acceptable here.
    pub async fn start_logging(
        &self,
        folder: &Path,
    ) -> Result<Vec<AbortHandle>, Box<dyn std::error::Error>> {
        // This destructure is used so that we don't forget later additions.
        let InputTree {
            time,
            transition_command,
            actuator_command,
            rocket_health,
            flight_computer_status,
            uremote_sensors,
            uremote_actuators,
            controllino_valves: _,  // TODO (or find a better way)
            controllino_sensors: _, // TODO (or find a better way)
            n2_pre_booster_pressure,
            h2_purge_pressure,
            n2_post_booster_pressure,
            lox_pressure,
            purge_pressure,
            pneumatic_pressure,
            gox_pressure,
            h2_pressure,
            lox_fill_valve,
            pss_valves,
            pss_sensors,
        } = self;

        /// A macro which captures `time` and `folder` to spawn a logger for the board health.
        /// Probably could also be a closure.
        macro_rules! spawn_board_health_logger {
            ($name:literal, $e:expr, $header:expr,) => {
                lib_mission::util::log::spawn_watchable_logger(
                    lib_mission::util::log::make_timeseries_writer(folder, $name, &[$header])
                        .await?,
                    $e.watched().map(|h| h.map(|t| t.as_ms_timestamp())),
                    time.watched(),
                )
            };
        }

        let mut handles = vec![
            // Log real time to "timeseries-realtime.csv" with headers "timestamp [unix ms]", "real time [unix ms]".
            // NOTE: Disabled for now, since we use realtime to set the mission time.
            // spawn_watchable_logger(
            //     make_timeseries_writer(folder, "realtime", &["real time [unix ms]"]).await?,
            //     // trigger based on updated mission time
            //     time.watched().map(|_t| {
            //         std::time::SystemTime::now()
            //             .duration_since(std::time::SystemTime::UNIX_EPOCH)
            //             .expect("travelled to past 1970")
            //             .as_millis()
            //     }),
            //     time.watched(),
            // ),

            // Log transition commands to two columns (or leave them empty)
            spawn_watchable_logger(
                make_timeseries_writer(
                    folder,
                    "transition_command",
                    &["index [int?]", "state [string?]"],
                )
                .await?,
                transition_command.watched().map(|r| {
                    (
                        r.transition_index,
                        Some(
                            String::from_utf8_lossy(&r.state)
                                .trim_end_matches('\0')
                                .to_string(),
                        ),
                    )
                }),
                time.watched(),
            ),
            // Log actuator commands to three columns (or leave them empty)
            spawn_watchable_logger(
                make_timeseries_writer(
                    folder,
                    "actuator_command",
                    &["index [int?]", "actuator [string?], value [float?]"],
                )
                .await?,
                actuator_command.watched().map(|r| {
                    // Propagate the Noneness to each field
                    r.map_or((None, None, None), |c| {
                        (Some(c.in_state_index), Some(c.actuator), Some(c.value))
                    })
                }),
                time.watched(),
            ),
            // Log board last healths to a single column csv files
            spawn_board_health_logger!(
                "rocket_health_flight_computer",
                rocket_health.flight_computer,
                "fuel control board last healthy [unix ms?]",
            ),
            spawn_board_health_logger!(
                "rocket_health_fuel_control_board",
                rocket_health.fuel_control_board,
                "fuel control board last healthy [unix ms?]",
            ),
            spawn_board_health_logger!(
                "rocket_health_oxidizer_control_board",
                rocket_health.oxidizer_control_board,
                "oxidizer control board last healthy [unix ms?]",
            ),
            spawn_board_health_logger!(
                "rocket_health_engine_control_board",
                rocket_health.engine_control_board,
                "engine control board last healthy [unix ms?]",
            ),
            // Log flight computer status (as the CAN message is defined)
            spawn_watchable_logger(
                make_timeseries_writer(
                    folder,
                    "flight_computer_status",
                    &[
                        "errors [#]",
                        "micros_since_restart [us]",
                        "has_pss_control [bool]",
                        "is_ignition_armed [bool]",
                    ],
                )
                .await?,
                flight_computer_status.watched(),
                time.watched(),
            ),
            // Log observed lox fill valve state
            spawn_watchable_logger(
                make_timeseries_writer(
                    folder,
                    "observed_state_LOX_FLL_VLV",
                    &["observed state [openness %]"],
                )
                .await?,
                lox_fill_valve.watched(),
                time.watched(),
            ),
        ];

        handles.extend(uremote_actuators.start_logging(folder, time).await?);
        handles.extend(uremote_sensors.start_logging(folder, time).await?);
        handles.extend(
            start_pressure_sensors_logging(
                folder,
                time,
                n2_pre_booster_pressure,
                h2_purge_pressure,
                n2_post_booster_pressure,
                lox_pressure,
                purge_pressure,
                pneumatic_pressure,
                gox_pressure,
                h2_pressure,
            )
            .await?,
        );

        handles.extend(pss_valves.start_logging(folder, time).await?);
        handles.extend(pss_sensors.start_logging(folder, time).await?);

        Ok(handles)
    }
}

impl OutputTree {
    /// Starts logging output tree timeseries to csv files in `folder`.
    /// Used for data analysis.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    pub async fn start_logging(
        &self,
        folder: &Path,
        time: Watchable<MissionTime>,
    ) -> Result<Vec<AbortHandle>, Box<dyn std::error::Error>> {
        // This destructure is used so that we don't forget later additions.
        let OutputTree {
            rfs_actuators,
            controllino_actuators: _, // TODO kind of
            pss_valves: _,
            lox_fill_valve,
        } = self;

        let mut handles = vec![lib_mission::util::log::spawn_watchable_logger(
            lib_mission::util::log::make_timeseries_writer(
                folder,
                "target_state_LOX_FLL_VLV",
                &["target state [openness %]"],
            )
            .await?,
            lox_fill_valve.watched(),
            time.watched(),
        )];
        handles.extend(rfs_actuators.start_logging(folder, &time).await?);

        Ok(handles)
    }
}

#[allow(clippy::too_many_arguments)]
async fn start_pressure_sensors_logging(
    folder: &std::path::Path,
    time: &Watchable<lib_mission::util::time::MissionTime>,
    n2_pre_booster_pressure: &Watchable<datatypes::units::BarG>,
    h2_purge_pressure: &Watchable<datatypes::units::BarG>,
    n2_post_booster_pressure: &Watchable<datatypes::units::BarG>,
    lox_pressure: &Watchable<datatypes::units::BarG>,
    purge_pressure: &Watchable<datatypes::units::BarG>,
    pneumatic_pressure: &Watchable<datatypes::units::BarG>,
    gox_pressure: &Watchable<datatypes::units::BarG>,
    h2_pressure: &Watchable<datatypes::units::BarG>,
) -> Result<Vec<AbortHandle>, Box<dyn std::error::Error>> {
    use lib_mission::reactive::IntoWatched as _;
    macro_rules! spawn_pressure_logger {
        ($name:literal, $e:expr) => {
            lib_mission::util::log::spawn_watchable_logger(
                lib_mission::util::log::make_timeseries_writer(
                    folder,
                    concat!($name, "_pressure_1"),
                    &["pressure_1 [barg]"],
                )
                .await?,
                $e.watched(),
                time.watched(),
            )
        };
    }

    // One log per sensor.
    let handles = vec![
        spawn_pressure_logger!("sensor_n2_pre_booster_p", n2_pre_booster_pressure),
        spawn_pressure_logger!("sensor_h2_purge_p", h2_purge_pressure),
        spawn_pressure_logger!("sensor_lox_p", lox_pressure),
        spawn_pressure_logger!("sensor_gox_p", gox_pressure),
        spawn_pressure_logger!("sensor_purge_p", purge_pressure),
        spawn_pressure_logger!("sensor_pneumatic_p", pneumatic_pressure),
        spawn_pressure_logger!("sensor_n2_post_booster_p", n2_post_booster_pressure),
        spawn_pressure_logger!("sensor_h2_p", h2_pressure),
    ];
    Ok(handles)
}
