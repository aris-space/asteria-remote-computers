use std::time::Duration;

use lib_mission::reactive::{IntoWatched, Watched};
use lib_mission::telemetry::{hermes_mavlink, spawn_periodic_telemetry_streamer};
use lib_mission::util::time::MissionTime;

use crate::input::{PSSSensors, PSSValves};

impl PSSValves {
    /// Spawns a tokio task which send mavlink messages for actuator feedback periodically to the channel.
    pub fn start_telemetry_streaming(
        &self,
        conn: &std::sync::mpsc::Sender<hermes_mavlink::MavMessage>,
        period: Duration,
        clock: impl Watched<MissionTime> + Send + 'static,
    ) -> tokio::task::AbortHandle {
        let PSSValves {
            pressurization_vent_valve,
            fuel_pressurization_valve,
            fuel_vent_valve,
            oxidizer_pressurization_valve,
            oxidizer_vent_valve,
            fuel_main_valve,
            oxidizer_main_valve,
        } = self.clone();

        // Actuator Feedback:
        // For each actuator, it reads the openness state and turns that the appropriate type for mavlink.
        spawn_periodic_telemetry_streamer(
            move || {
                let fss_dpr = fuel_pressurization_valve.get();
                let oss_dpr = oxidizer_pressurization_valve.get();
                hermes_mavlink::MavMessage::PSS_ACTUATOR_STATES(
                    hermes_mavlink::PSS_ACTUATOR_STATES_DATA {
                        time_unix_ms: clock.current().as_ms_timestamp(),
                        prz_vnt_vlv: pressurization_vent_valve.get().is_open().into(),
                        fss_prz_vlv: fss_dpr.into(),
                        fss_vnt_vlv: fuel_vent_valve.get().is_open().into(),
                        oss_prz_vlv: oss_dpr.into(),
                        oss_vnt_vlv: oxidizer_vent_valve.get().is_open().into(),
                        fss_mnl_vlv: fuel_main_valve.get().is_open().into(),
                        oss_mnl_vlv: oxidizer_main_valve.get().is_open().into(),
                    },
                )
            },
            period,
            conn.clone(),
        )
    }
}

impl PSSSensors {
    /// Spawns tokio tasks which send mavlink messages for sensor data periodically to the channel.
    pub fn start_telemetry_streaming<C>(
        &self,
        conn: &std::sync::mpsc::Sender<hermes_mavlink::MavMessage>,
        pressure_period: Duration,
        temperature_period: Duration,
        level_period: Duration,
        clock: &C,
    ) -> [tokio::task::AbortHandle; 3]
    where
        C: IntoWatched<MissionTime> + Clone,
        C::Watched: Send + 'static,
    {
        let PSSSensors {
            pressurization_line_pressure,
            fuel_tank_pressure,
            fuel_tank_pressure_1,
            fuel_tank_pressure_2,
            oxidizer_tank_pressure,
            oxidizer_tank_pressure_1,
            oxidizer_tank_pressure_2,
            injector_fuel_pressure,
            injector_oxidizer_pressure,
            engine_chamber_pressure,
            oxidizer_fill_fraction,
            fuel_tank_temperature,
            oxidizer_tank_temperature,
            oxidizer_runline_temperature,
        } = self.clone();
        let clock1 = clock.clone().watched();
        let clock2 = clock.clone().watched();
        let clock3 = clock.clone().watched();

        [
            // Temperature
            spawn_periodic_telemetry_streamer(
                move || {
                    hermes_mavlink::MavMessage::PSS_TEMPERATURE(
                        hermes_mavlink::PSS_TEMPERATURE_DATA {
                            time_unix_ms: clock1.current().as_ms_timestamp(),
                            fss_tnk_t: fuel_tank_temperature.get().as_f32(),
                            oss_tnk_t: oxidizer_tank_temperature.get().as_f32(),
                            oss_rnl_t: oxidizer_runline_temperature.get().as_f32(),
                        },
                    )
                },
                temperature_period,
                conn.clone(),
            ),
            // Pressure
            spawn_periodic_telemetry_streamer(
                move || {
                    hermes_mavlink::MavMessage::PSS_PRESSURE(hermes_mavlink::PSS_PRESSURE_DATA {
                        time_unix_ms: clock2.current().as_ms_timestamp(),
                        prz_mnl_p: pressurization_line_pressure.get().as_f32(),
                        fss_tnk_p: fuel_tank_pressure.get().as_f32(),
                        fss_tnk_p1: fuel_tank_pressure_1.get().as_f32(),
                        fss_tnk_p2: fuel_tank_pressure_2.get().as_f32(),
                        oss_tnk_p: oxidizer_tank_pressure.get().as_f32(),
                        oss_tnk_p1: oxidizer_tank_pressure_1.get().as_f32(),
                        oss_tnk_p2: oxidizer_tank_pressure_2.get().as_f32(),
                        fss_inj_p: injector_fuel_pressure.get().as_f32(),
                        oss_inj_p: injector_oxidizer_pressure.get().as_f32(),
                        eng_cc_p: engine_chamber_pressure.get().as_f32(),
                    })
                },
                pressure_period,
                conn.clone(),
            ),
            // Level
            spawn_periodic_telemetry_streamer(
                move || {
                    hermes_mavlink::MavMessage::PSS_LEVEL(hermes_mavlink::PSS_LEVEL_DATA {
                        time_unix_ms: clock3.current().as_ms_timestamp(),
                        oss_tnk_fraction: oxidizer_fill_fraction.get().as_u8(),
                    })
                },
                level_period,
                conn.clone(),
            ),
        ]
    }
}
