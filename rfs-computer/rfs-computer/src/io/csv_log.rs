use std::path::Path;

use lib_mission::reactive::{IntoWatched as _, WatchedExt as _};
use lib_mission::util::log::{make_timeseries_writer, spawn_watchable_logger};
use lib_mission::util::time::MissionTime;
use tokio::task::AbortHandle;
use watchable::Watchable;

use crate::io::input::PressureSensors;

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
            telemetry_health,
            is_can_connected,
            rocket_health,
            flight_computer_status,
            uremote_sensors,
            uremote_actuators,
            serial_pressure_sensors,
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

            // Log the timestamp of the last received telemetry message
            spawn_watchable_logger(
                make_timeseries_writer(
                    folder,
                    "telemetry_health",
                    &[
                        "time_of_last_msg [unix ms]",
                        "num_msg_received",
                        "time_of_last_processed_msg [unix ms]",
                        "num_msg_processed",
                    ],
                )
                .await?,
                telemetry_health.watched().map(|h| {
                    (
                        h.time_of_last_msg.as_ms_timestamp(),
                        h.num_msg_received,
                        h.time_of_last_processed_msg.as_ms_timestamp(),
                        h.num_msg_processed,
                    )
                }),
                time.watched(),
            ),
            // Log transition commands to two columns (or leave them empty)
            spawn_watchable_logger(
                make_timeseries_writer(
                    folder,
                    "transition_command",
                    &["index [int?]", "state [string?]"],
                )
                .await?,
                transition_command.watched().map(|r| {
                    // Propagate the Noneness to each field
                    r.map_or((None, None), |(i, s)| (i, Some(s.to_string())))
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
            spawn_watchable_logger(
                make_timeseries_writer(folder, "is_can_connected", &["connected [bool]"]).await?,
                is_can_connected.watched(),
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
        handles.extend(serial_pressure_sensors.start_logging(folder, time).await?);

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
            pss_valves,
            lox_fill_valve,
            is_software_armed,
        } = self;

        let mut handles = vec![
            lib_mission::util::log::spawn_watchable_logger(
                lib_mission::util::log::make_timeseries_writer(
                    folder,
                    "target_state_LOX_FLL_VLV",
                    &["target state [openness %]"],
                )
                .await?,
                lox_fill_valve.watched(),
                time.watched(),
            ),
            lib_mission::util::log::spawn_watchable_logger(
                lib_mission::util::log::make_timeseries_writer(
                    folder,
                    "is_software_armed",
                    &["is armed [bool]"],
                )
                .await?,
                is_software_armed.watched(),
                time.watched(),
            ),
        ];
        handles.extend(rfs_actuators.start_logging(folder, &time).await?);
        handles.extend(pss_valves.start_logging(folder, time.clone()).await?);

        Ok(handles)
    }
}

impl PressureSensors {
    async fn start_logging(
        &self,
        folder: &std::path::Path,
        time: &Watchable<lib_mission::util::time::MissionTime>,
    ) -> Result<Vec<AbortHandle>, Box<dyn std::error::Error>> {
        use lib_mission::reactive::IntoWatched as _;
        macro_rules! spawn_pressure_logger {
            ($name:literal, $e:expr) => {
                [
                    lib_mission::util::log::spawn_watchable_logger(
                        lib_mission::util::log::make_timeseries_writer(
                            folder,
                            concat!($name, "_pressure_1"),
                            &["pressure_1 [barg]"],
                        )
                        .await?,
                        $e.pressure_value_1.watched(),
                        time.watched(),
                    ),
                    // Those sensor channels are disabled, so no need to log them:

                    // lib_mission::util::log::spawn_watchable_logger(
                    //     lib_mission::util::log::make_timeseries_writer(
                    //         folder,
                    //         concat!($name, "_pressure_2"),
                    //         &["pressure_2 [barg]"],
                    //     )
                    //     .await?,
                    //     $e.pressure_value_2.watched(),
                    //     time.watched(),
                    // ),
                    // lib_mission::util::log::spawn_watchable_logger(
                    //     lib_mission::util::log::make_timeseries_writer(
                    //         folder,
                    //         concat!($name, "_temperature_1"),
                    //         &["temperature_1 [°C]"],
                    //     )
                    //     .await?,
                    //     $e.temperature_value_1.watched(),
                    //     time.watched(),
                    // ),
                    // lib_mission::util::log::spawn_watchable_logger(
                    //     lib_mission::util::log::make_timeseries_writer(
                    //         folder,
                    //         concat!($name, "_temperature_2"),
                    //         &["temperature_2 [°C]"],
                    //     )
                    //     .await?,
                    //     $e.temperature_value_2.watched(),
                    //     time.watched(),
                    // ),
                ]
            };
        }

        let PressureSensors {
            n2_pre_booster,
            h2_purge,
            lox,
            gox,
            purge,
            pneumatic,
            n2_post_booster,
            h2,
            igniter,
        } = self;

        // Log serial devices values
        let mut handles = vec![];
        handles.extend(spawn_pressure_logger!(
            "sensor_n2_pre_booster_p",
            n2_pre_booster
        ));
        handles.extend(spawn_pressure_logger!("sensor_h2_purge_p", h2_purge));
        handles.extend(spawn_pressure_logger!("sensor_lox_p", lox));
        handles.extend(spawn_pressure_logger!("sensor_gox_p", gox));
        handles.extend(spawn_pressure_logger!("sensor_purge_p", purge));
        handles.extend(spawn_pressure_logger!("sensor_pneumatic_p", pneumatic));
        handles.extend(spawn_pressure_logger!(
            "sensor_n2_post_booster_p",
            n2_post_booster
        ));
        handles.extend(spawn_pressure_logger!("sensor_h2_p", h2));
        handles.extend(spawn_pressure_logger!("sensor_igniter_p", igniter));
        Ok(handles)
    }
}
