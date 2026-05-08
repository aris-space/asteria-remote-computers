use std::path::Path;

use lib_mission::reactive::{IntoWatched as _, WatchedExt as _};
use lib_mission::util::log::{make_timeseries_writer, spawn_watchable_logger};
use lib_mission::util::time::MissionTime;
use watchable::Watchable;

use crate::io::input::CurrentStateWrapper;

use super::input::{InputTree, RawUnprocessedData};
use super::output::OutputTree;

impl InputTree {
    /// Starts logging input tree timeseries to csv files in `folder`.
    /// Used for both database import and simulation replays.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    #[allow(clippy::too_many_lines)] // This is ugly, but acceptable here.
    pub async fn start_logging(&self, folder: &Path) -> Result<(), Box<dyn std::error::Error>> {
        use lib_mission::reactive::IntoWatched as _;

        // This destructure is used so that we don't forget later additions.
        let InputTree {
            state: CurrentStateWrapper { current_state },
            time,
            transition_command,
            actuator_command,
            telemetry_health,
            downlink_telemetry_stats: _, // TODO: log
            boards_status,
            recovery_armed,
            ecu_armed,
            motors_position,
            cats_separation,
            cats_deployment,
            location,
            velocity,
            imu,
            orientation,
            raw_unprocessed_data,
            pss_valves,
            pss_sensors,
        } = self;
        let RawUnprocessedData {
            environmental_data,
            rail_status_24v,
            rail_status_5v,
        } = raw_unprocessed_data;

        // Log real time to "timeseries-realtime.csv" with headers "timestamp [unix ms]", "real time [unix ms]".
        // Disabled for now, since we use realtime to set the mission time.

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
        // );

        spawn_watchable_logger(
            make_timeseries_writer(folder, "current_state", &["index [int]", "state [string]"])
                .await?,
            current_state
                .watched()
                .map(|(index, state)| (index, Some(state.to_string()))),
            time.watched(),
        );

        //////////////////////// DATA STANDARDS ///////////////////////
        // Log location data to "timeseries-location.csv" with headers "timestamp [unix ms]", "location latitude [°]", "location longitude [°]", "location altitude [m above sea level]".
        // And additional columns for accuracy.
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "location",
                &[
                    "location latitude [°]",
                    "location longitude [°]",
                    "location altitude [m above sea level]",
                    "location horizontal accuracy [1-sigma]",
                    "location vertical accuracy [1-sigma]",
                ],
            )
            .await?,
            location.watched().map(|l| {
                (
                    l.location_latitude,
                    l.location_longitude,
                    l.location_hamsl,
                    l.horizontal_accuracy,
                    l.vertical_accuracy,
                )
            }),
            time.watched(),
        );
        // Log acceleration data to "timeseries-acceleration.csv" with headers "timestamp [unix ms]", "acceleration north [m/s^2]",  "acceleration east [m/s^2]",  "acceleration down [m/s^2]".
        // And additional columns for body-fixed frame.
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "acceleration",
                &[
                    "acceleration north [m/s^2]",
                    "acceleration east [m/s^2]",
                    "acceleration down [m/s^2]",
                    "acceleration x-axis [m/s^2]",
                    "acceleration y-axis [m/s^2]",
                    "acceleration z-axis [m/s^2]",
                ],
            )
            .await?,
            imu.watched().map(|a| {
                (
                    a.acceleration_north,
                    a.acceleration_east,
                    a.acceleration_down,
                    a.acceleration_x,
                    a.acceleration_y,
                    a.acceleration_z,
                )
            }),
            time.watched(),
        );
        // Log velocity data to "timeseries-velocity.csv" with headers "timestamp [unix ms]", "velocity north [m/s]", "velocity east [m/s]", "velocity down [m/s]".
        // And additional columns for body-fixed frame.
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "velocity",
                &[
                    "velocity north [m/s]",
                    "velocity east [m/s]",
                    "velocity down [m/s]",
                    "velocity x-axis [m/s]",
                    "velocity y-axis [m/s]",
                    "velocity z-axis [m/s]",
                ],
            )
            .await?,
            velocity.watched().map(|v| {
                (
                    v.velocity_north,
                    v.velocity_east,
                    v.velocity_down,
                    v.velocity_x,
                    v.velocity_y,
                    v.velocity_z,
                )
            }),
            time.watched(),
        );
        // Log orientation data as NED quaternion to "timeseries-orientation.csv" with headers "timestamp [unix ms]", "orientation W", "orientation X", "orientation Y", "orientation Z".
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "orientation",
                &[
                    "orientation W",
                    "orientation X",
                    "orientation Y",
                    "orientation Z",
                ],
            )
            .await?,
            orientation.watched().map(|o| {
                (
                    o.orientation_w,
                    o.orientation_x,
                    o.orientation_y,
                    o.orientation_z,
                )
            }),
            time.watched(),
        );

        ///////////////////// INTERNALS (for replaying) ///////////////////////
        // Log the timestamp of the last received telemetry message
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "telemetry_health",
                &[
                    "time_of_last_msg [unix us]",
                    "num_msg_received",
                    "num_msg_lost",
                    "time_of_last_processed_msg [unix us]",
                    "num_msg_processed",
                ],
            )
            .await?,
            telemetry_health.watched().map(|h| {
                (
                    h.time_of_last_msg.0,
                    h.num_msg_received,
                    h.num_msg_lost,
                    h.time_of_last_processed_msg.0,
                    h.num_msg_processed,
                )
            }),
            time.watched(),
        );
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
        );
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
        );
        // TODO: Log board last healths
        // Log RFS computer status
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "rfs_computer_status",
                &[
                    "errors [#]",
                    "micros_since_restart [us]",
                    "has_pss_control [bool]",
                    "is_ignition_armed [bool]",
                ],
            )
            .await?,
            boards_status.rfs_computer.watched().map(|rep| {
                (
                    rep.current_status.common.errors,
                    rep.current_status.common.micros_since_restart,
                    rep.current_status.has_pss_control,
                    rep.current_status.is_ignition_armed == datatypes::status::ArmingState::Armed,
                )
            }),
            time.watched(),
        );
        // Log arming states
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "recovery_hardware_arming",
                &["recovery arming state [Safe|Armed]"],
            )
            .await?,
            recovery_armed.watched(),
            time.watched(),
        );
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "ecu_hardware_arming",
                &["ecu arming state [Safe|Armed]"],
            )
            .await?,
            ecu_armed.watched(),
            time.watched(),
        );

        // Log actual steering motor positions
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "steering_motor_actual_positions",
                &["actual left [steps]", "actual right [steps]"],
            )
            .await?,
            motors_position.watched().map(|r| (r.left_pos, r.right_pos)),
            time.watched(),
        );

        // Log CATS drogue separation status
        spawn_watchable_logger(
            make_timeseries_writer(folder, "cats_separation", &["cats separation"]).await?,
            cats_separation.watched(),
            time.watched(),
        );
        // Log CATS main deployment status
        spawn_watchable_logger(
            make_timeseries_writer(folder, "cats_deployment", &["cats deployment"]).await?,
            cats_deployment.watched(),
            time.watched(),
        );

        // Log power data
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "rail_status_5v",
                &["voltage [V]", "current [A]", "power [W]"],
            )
            .await?,
            rail_status_5v
                .watched()
                .map(|v| (v.voltage, v.current, v.power)),
            time.watched(),
        );
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "rail_status_24v",
                &["voltage [V]", "current [A]", "power [W]"],
            )
            .await?,
            rail_status_24v
                .watched()
                .map(|v| (v.voltage, v.current, v.power)),
            time.watched(),
        );
        // Log environmental data
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "environment",
                &["temperature [°C]", "humidity [%]", "pressure [hPa]"],
            )
            .await?,
            environmental_data
                .watched()
                .map(|v| (v.temperature, v.humidity, v.pressure)),
            time.watched(),
        );
        pss_valves.start_logging(folder, time).await?;
        pss_sensors.start_logging(folder, time).await?;

        Ok(())
    }
}

impl OutputTree {
    /// Starts logging output tree timeseries to csv files in `folder`.
    /// Used for data analysis.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    #[allow(clippy::too_many_lines)] // agreed, but oh well
    pub async fn start_logging(
        &self,
        folder: &Path,
        time: Watchable<MissionTime>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // This destructure is used so that we don't forget later additions.
        let OutputTree {
            separate_drogue,
            deploy_main_chute,
            steering_motor_target_deflections,
            recovery_actuators_armed,
            camera_powered_on,
            valves: _,
            hash_config: _,
            fc_status: _,
        } = self;

        // Log target steering motor positions
        spawn_watchable_logger(
            make_timeseries_writer(
                folder,
                "steering_motor_target_positions",
                &["target left [steps]", "target right [steps]"],
            )
            .await?,
            steering_motor_target_deflections.watched().map(|r| {
                (
                    r.as_ref().map(|r| r.left_motor_steps),
                    r.as_ref().map(|r| r.right_motor_steps),
                )
            }),
            time.watched(),
        );
        // Log drogue separation status
        spawn_watchable_logger(
            make_timeseries_writer(folder, "drogue_separation", &["target drogue separation"])
                .await?,
            separate_drogue.watched(),
            time.watched(),
        );
        // Log deployment status
        spawn_watchable_logger(
            make_timeseries_writer(folder, "deployment", &["target deployment"]).await?,
            deploy_main_chute.watched(),
            time.watched(),
        );
        // Log recovery board arming status
        spawn_watchable_logger(
            make_timeseries_writer(folder, "recovery_arming", &["Recovery Armed State"]).await?,
            recovery_actuators_armed.watched(),
            time.watched(),
        );
        // Log camera power status
        spawn_watchable_logger(
            make_timeseries_writer(folder, "cameras", &["target camera power"]).await?,
            camera_powered_on.watched(),
            time.watched(),
        );

        Ok(())
    }
}
