use watchable::Watchable;

use crate::input;

impl input::Sensors {
    /// Starts logging sensors timeseries to csv files in `folder`.
    /// Used for data analysis.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    pub async fn start_logging(
        &self,
        folder: &std::path::Path,
        time: &Watchable<lib_mission::util::time::MissionTime>,
    ) -> Result<[tokio::task::AbortHandle; 7], Box<dyn std::error::Error>> {
        use lib_mission::reactive::IntoWatched as _;
        macro_rules! spawn_serializable_logger {
            ($name:literal, $e:expr, $headers:expr,) => {
                lib_mission::util::log::spawn_watchable_logger(
                    lib_mission::util::log::make_timeseries_writer(folder, $name, &$headers)
                        .await?,
                    $e.watched(),
                    time.watched(),
                )
            };
        }

        let Self {
            valve_power_interlock,
            rocket_connection_detect,
            lox_decoupler_proximity,
            n2_decoupler_proximity,
            igniter_pressure,
            igniter_temperature,
            smart_box_temperature,
        } = self;

        Ok([
            spawn_serializable_logger!(
                "arming_smart_valve_power",
                valve_power_interlock,
                ["are armed [bool]"],
            ),
            spawn_serializable_logger!(
                "sensor_rocket_con_detect",
                rocket_connection_detect,
                ["is connected [bool]"],
            ),
            spawn_serializable_logger!(
                "sensor_lox_dec_prox",
                lox_decoupler_proximity,
                ["is retracted [bool]"],
            ),
            spawn_serializable_logger!(
                "sensor_n2_dec_prox",
                n2_decoupler_proximity,
                ["is retracted [bool]"],
            ),
            spawn_serializable_logger!("sensor_ign_cc_p", igniter_pressure, ["ign_cc_p [barg]"],),
            spawn_serializable_logger!("sensor_ign_cc_t", igniter_temperature, ["ign_cc_t [°C]"],),
            spawn_serializable_logger!(
                "sensor_smart_cabinent_t",
                smart_box_temperature,
                ["smart_cabinent_t [°C]"],
            ),
        ])
    }
}
