use std::collections::HashMap;
use std::time::Duration;

use datatypes::{actuator::Energizable, sensor::DigitalIn, units::Celsius};
use lib_mission::reactive::{IntoWatched, Watched, WatchedExt as _};
use ur20::ModuleType;
use ur20_modbus::Coupler;
use watchable::Watchable;

use crate::{input, output};

const MODULES: [ModuleType; 9] = [
    ModuleType::UR20_8DI_P_2W, // Slot 11, Module 0: 8 inputs (proximity sensors)
    ModuleType::UR20_4AI_TC_DIAG, // Slot 12, Module 1: 4 thermo couples
    ModuleType::UR20_4AI_UI_16, // Slot 13, Module 2: 4 analog inputs (?)
    ModuleType::UR20_8DO_P,    // Slot 14, Module 3: 8 outputs (valves)
    /* ModuleType::UR20_PF_O */ // Slot 15: power feed (passive)
    ModuleType::UR20_8DO_P,        // Slot 16, Module 4: 8 outputs (valves)
    ModuleType::UR20_8DO_P,        // Slot 17, Module 5: 8 outputs (valves)
    ModuleType::UR20_4DO_P_2A,     // Slot 18, Module 6: 4 2A outputs (valves)
    ModuleType::UR20_PF_O_1DI_SIL, // Slot 19, Module 7: Safe power feed
    ModuleType::UR20_4DO_P_2A,     // Slot 20, Module 8: 4 2A outputs (ignitors)
];

/// The channel addresses where the electronics are connected to the uremote
mod channels {
    pub const MAXON_BOARD_POWER: ur20::Address = ur20::Address {
        module: 3,
        channel: 1,
    };
    pub const CUSTOM1: ur20::Address = ur20::Address {
        module: 3,
        channel: 5,
    };
    pub const CUSTOM2: ur20::Address = ur20::Address {
        module: 3,
        channel: 6,
    };
    pub const CUSTOM3: ur20::Address = ur20::Address {
        module: 3,
        channel: 7,
    };

    // Valves
    pub const GOX_PURGE_VALVE: ur20::Address = ur20::Address {
        module: 4,
        channel: 0,
    };
    pub const H2_PURGE_VALVE: ur20::Address = ur20::Address {
        module: 4,
        channel: 1,
    };
    pub const N2_BLEED_VALVE: ur20::Address = ur20::Address {
        module: 4,
        channel: 2,
    };
    pub const LOX_PURGE_VALVE: ur20::Address = ur20::Address {
        module: 4,
        channel: 3,
    };
    pub const GOX_BLEED_VALVE: ur20::Address = ur20::Address {
        module: 4,
        channel: 4,
    };
    pub const N2_PURGE_VALVE: ur20::Address = ur20::Address {
        module: 4,
        channel: 5,
    };
    pub const N2_FILL_VALVE: ur20::Address = ur20::Address {
        module: 4,
        channel: 6,
    };
    pub const H2_BLEED_VALVE: ur20::Address = ur20::Address {
        module: 4,
        channel: 7,
    };
    pub const LOX_BLEED_VALVE: ur20::Address = ur20::Address {
        module: 6,
        channel: 3,
    };
    pub const BOOSTER_VALVE: ur20::Address = ur20::Address {
        module: 5,
        channel: 1,
    };
    pub const H2_IGNITER_VALVE: ur20::Address = ur20::Address {
        module: 6,
        channel: 0,
    };
    pub const GOX_IGNITER_VALVE: ur20::Address = ur20::Address {
        module: 6,
        channel: 1,
    };
    pub const IGNITER_SPARK: ur20::Address = ur20::Address {
        module: 8,
        channel: 0,
    };

    // Sensors
    pub const LOX_DECOUPLER_PROXIMITY: ur20::Address = ur20::Address {
        module: 0,
        channel: 0,
    };
    pub const N2_DECOUPLER_PROXIMITY: ur20::Address = ur20::Address {
        module: 0,
        channel: 1,
    };
    pub const VALVE_ARMING_CIRCUIT: ur20::Address = ur20::Address {
        module: 0,
        channel: 2,
    };
    /// Also wired to 0:3
    pub const IGNITER_ARMING_CIRCUIT: ur20::Address = ur20::Address {
        module: 7,
        channel: 0,
    };
    pub const LOX_TEMPERATURE: ur20::Address = ur20::Address {
        module: 1,
        channel: 0,
    };
    pub const CUSTOM_TEMPERATURE: ur20::Address = ur20::Address {
        module: 1,
        channel: 1,
    };
    pub const CABINET_TEMPERATURE: ur20::Address = ur20::Address {
        module: 1,
        channel: 2,
    };
    pub const AMBIENT_TEMPERATURE: ur20::Address = ur20::Address {
        module: 1,
        channel: 3,
    };
}

const NUM_WATCHED_OUTPUTS: usize = 17;

/// A abstraction of our Weidmüller u-remote driver.
///
/// Use [`Driver::new`] and [`Driver::run`] to use this.
pub struct Driver {
    /// The address of the uremote, usually `192.168.0.222:502`.
    socket_addr: String,
    coupler: Coupler,
    /// The maximal time between two reads. Reads can happen fast than this.
    /// This needs to be below 1 second such that the uremote watchdog doesn't time out.
    max_read_period: Duration,
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("Something is out of order with the setup: {0}")]
    SetupError(String),
    #[error(transparent)]
    UR20Error(#[from] ur20_modbus::Error),
}

/// A type which can't be constructed and as such a function which returns this cannot return.
pub enum NeverReturns {}

impl Driver {
    /// Connects to the Weidmüller UR20 coupler on address `socket_adr`, usually `192.168.0.222:502`
    #[tracing::instrument(level = "INFO", name = "weidmüller_driver_setup", skip_all)]
    pub async fn new(socket_adr: &str, max_read_period: Duration) -> Result<Self, Error> {
        let coupler = Self::connect(socket_adr, max_read_period).await?;

        Ok(Self {
            socket_addr: socket_adr.into(),
            coupler,
            max_read_period,
        })
    }

    /// Connects to the Weidmüller UR20 coupler on address `socket_adr`, usually `192.168.0.222:502`
    #[tracing::instrument(level = "INFO", skip_all)]
    async fn connect(socket_adr: &str, max_read_period: Duration) -> Result<Coupler, Error> {
        let addr = socket_adr
            .parse()
            .map_err(|e| Error::SetupError(format!("invalid socket address {e}")))?;
        let mut coupler = Coupler::connect(addr).await?;

        let id = coupler.id().await?;
        tracing::info!("Connected to {id}");

        if coupler.modules() != MODULES {
            tracing::error!(
                expected = ?MODULES,
                actual = ?coupler.modules(),
                "Coupler reports different modules than expected"
            );
            return Err(Error::SetupError(
                "Coupler reports different modules than expected".into(),
            ));
        }

        // Tick once to initialize state
        coupler.tick().await?;

        // Check inputs are enabled
        {
            let inputs = coupler.inputs();

            if !matches!(
                inputs.get(&channels::LOX_TEMPERATURE),
                Some(ur20::ChannelValue::Decimal32(_)),
            ) {
                tracing::error!("Cannot get LOX temperature value");
            }
            if !matches!(
                inputs.get(&channels::CUSTOM_TEMPERATURE),
                Some(ur20::ChannelValue::Decimal32(_)),
            ) {
                tracing::warn!("Cannot get Custom temperature value"); // potentially not connected
            }
            if !matches!(
                inputs.get(&channels::LOX_DECOUPLER_PROXIMITY),
                Some(ur20::ChannelValue::Bit(_))
            ) {
                tracing::error!("Cannot get lox decoupler proximity value",);
            }
            if !matches!(
                inputs.get(&channels::N2_DECOUPLER_PROXIMITY),
                Some(ur20::ChannelValue::Bit(_))
            ) {
                tracing::error!("Cannot get n2 decoupler proximity value",);
            }
        }

        // TODO: check all the expected outputs are in a default state.

        if max_read_period >= Duration::from_secs_f32(0.8) {
            return Err(Error::SetupError(
                "Max read period is cutting close to watchdog timer".into(),
            ));
        }

        Ok(coupler)
    }

    /// Runs the Weidmüller coupler driver until cancelled.
    #[tracing::instrument(level = "TRACE", name = "weidmüller_driver", skip_all)]
    pub async fn run(
        &mut self,
        sensors: &input::Sensors,
        input: &input::Actuators,
        output: &output::Actuators,
    ) -> NeverReturns {
        let mut watchers = output.as_watched_list();
        loop {
            self.loop_once(sensors, input, &mut watchers).await;
        }
    }

    /// Waits for any actuator target state to change and then sends this to the coupler and receives the newest sensor input values.
    /// If no actuator target changes before [`Self::max_read_period`] time passed, the read is also performed.
    async fn loop_once(
        &mut self,
        sensors: &input::Sensors,
        input: &input::Actuators,
        output: &mut [Box<dyn Watched<(ur20::Address, ur20::ChannelValue)> + Send>;
                 NUM_WATCHED_OUTPUTS],
    ) {
        // Either wait for a output to change, then update it, or else just do the read afterwards.
        tokio::select! {
            ((addr, value), _, _) = futures_util::future::select_all(output.iter_mut().map(|w| w.next())) => {
                if let Err(e) = self.coupler.set_output(&addr, value) {
                    tracing::warn!(error = %e, "Problem setting output");
                }
            },
            _ = tokio::time::sleep(self.max_read_period) => {}
        };

        self.reconnecting_tick().await;

        let inputs = self.coupler.inputs();
        let outputs = self.coupler.outputs();
        input.update_from_values(&inputs, &outputs);
        sensors.update_from_inputs(&inputs);
    }

    /// Does as [`Coupler::tick`] but until it succeeded
    async fn reconnecting_tick(&mut self) {
        let inital_backoff = self.max_read_period; // around 50ms;
        let max_backoff = 20 * self.max_read_period; // around 1 sec
        let mut backoff = inital_backoff;

        let mut fail_counter = 0;

        while let Err(e) = self.coupler.tick().await {
            fail_counter += 1;

            // For the first 3 failures, just try again. Afterwards, reconnect.
            if fail_counter <= 3 {
                tracing::warn!(error = %e, fail_counter, "Problem ticking, trying again");
            } else {
                tracing::warn!(error = %e, fail_counter, "Problem ticking, trying to reconnect");
                if let Err(disconn_e) = self.coupler.disconnect().await {
                    tracing::warn!(error = %disconn_e, "Problem disconnecting");
                }

                match Self::connect(&self.socket_addr, self.max_read_period).await {
                    Ok(new_coupler) => {
                        tracing::info!("reconnected to uremote");
                        self.coupler = new_coupler;
                        // Reset the backoff
                        fail_counter = 0;
                        backoff = inital_backoff;
                        continue;
                    }
                    Err(conn_e) => {
                        tracing::warn!(error = %conn_e, "Problem reconnecting, trying again");
                    }
                }
            }
            // Exponential backoff on errors, maybe due to device not connected
            tokio::time::sleep(backoff).await;
            backoff = (2 * backoff).min(max_backoff);
        }
    }
}

impl output::Actuators {
    /// Generate a list of `Watched`s which can be used to listen for any change in the output tree.
    fn as_watched_list(
        &self,
    ) -> [Box<dyn Watched<(ur20::Address, ur20::ChannelValue)> + Send>; NUM_WATCHED_OUTPUTS] {
        /// Boxes a watcher for `T` which gives active values for the provided ur20 address.
        fn box_actuator<T: Energizable + Clone + Send + 'static, W: IntoWatched<T>>(
            w: W,
            channel: ur20::Address,
        ) -> Box<impl Watched<(ur20::Address, ur20::ChannelValue)> + 'static>
        where
            W::Watched: 'static,
        {
            Box::new(
                w.watched()
                    .map(move |vlv| (channel, ur20::ChannelValue::Bit(vlv.is_active()))),
            )
        }
        // Matches targets to addresses
        [
            box_actuator(&self.maxon_board_power, channels::MAXON_BOARD_POWER),
            box_actuator(&self.custom1, channels::CUSTOM1),
            box_actuator(&self.custom2, channels::CUSTOM2),
            box_actuator(&self.custom3, channels::CUSTOM3),
            box_actuator(&self.gox_purge_valve, channels::GOX_PURGE_VALVE),
            box_actuator(&self.h2_purge_valve, channels::H2_PURGE_VALVE),
            box_actuator(&self.n2_bleed_valve, channels::N2_BLEED_VALVE),
            box_actuator(&self.lox_purge_valve, channels::LOX_PURGE_VALVE),
            box_actuator(&self.gox_bleed_valve, channels::GOX_BLEED_VALVE),
            box_actuator(&self.n2_purge_valve, channels::N2_PURGE_VALVE),
            box_actuator(&self.n2_fill_valve, channels::N2_FILL_VALVE),
            box_actuator(&self.h2_bleed_valve, channels::H2_BLEED_VALVE),
            box_actuator(&self.lox_bleed_valve, channels::LOX_BLEED_VALVE),
            box_actuator(&self.booster_valve, channels::BOOSTER_VALVE),
            box_actuator(&self.h2_igniter_valve, channels::H2_IGNITER_VALVE),
            box_actuator(&self.gox_igniter_valve, channels::GOX_IGNITER_VALVE),
            box_actuator(&self.igniter_spark, channels::IGNITER_SPARK),
        ]
    }
}

impl input::Actuators {
    /// Takes the input and output values the Weidmüller coupler provides,
    /// and update the input tree with the best estimate of actuator position.
    fn update_from_values(
        &self,
        _inputs: &HashMap<ur20::Address, ur20::ChannelValue>,
        outputs: &HashMap<ur20::Address, ur20::ChannelValue>,
    ) {
        /// updates a single output
        fn update_out<T: Energizable + PartialEq>(
            outputs: &HashMap<ur20::Address, ur20::ChannelValue>,
            w: &Watchable<T>,
            channel: ur20::Address,
        ) {
            if let Some(&ur20::ChannelValue::Bit(is_active)) = outputs.get(&channel) {
                let _ = w.update(T::from_active(is_active));
            } else {
                tracing::error!(?channel, "Actuator channel doesn't have output info");
            }
        }

        // TODO (later): Better data for valves with feedback
        update_out(
            outputs,
            &self.maxon_board_power,
            channels::MAXON_BOARD_POWER,
        );
        update_out(outputs, &self.custom1, channels::CUSTOM1);
        update_out(outputs, &self.custom2, channels::CUSTOM2);
        update_out(outputs, &self.custom3, channels::CUSTOM3);

        update_out(outputs, &self.gox_purge_valve, channels::GOX_PURGE_VALVE);
        update_out(outputs, &self.h2_purge_valve, channels::H2_PURGE_VALVE);
        update_out(outputs, &self.n2_bleed_valve, channels::N2_BLEED_VALVE);
        update_out(outputs, &self.lox_purge_valve, channels::LOX_PURGE_VALVE);
        update_out(outputs, &self.gox_bleed_valve, channels::GOX_BLEED_VALVE);
        update_out(outputs, &self.n2_purge_valve, channels::N2_PURGE_VALVE);
        update_out(outputs, &self.n2_fill_valve, channels::N2_FILL_VALVE);
        update_out(outputs, &self.h2_bleed_valve, channels::H2_BLEED_VALVE);
        update_out(outputs, &self.lox_bleed_valve, channels::LOX_BLEED_VALVE);
        update_out(outputs, &self.booster_valve, channels::BOOSTER_VALVE);
        update_out(outputs, &self.h2_igniter_valve, channels::H2_IGNITER_VALVE);
        update_out(
            outputs,
            &self.gox_igniter_valve,
            channels::GOX_IGNITER_VALVE,
        );
        update_out(outputs, &self.igniter_spark, channels::IGNITER_SPARK);
    }
}

impl input::Sensors {
    /// Consumes the input values the Weidmüller coupler provides, which includes all sensor data
    fn update_from_inputs(&self, values: &HashMap<ur20::Address, ur20::ChannelValue>) {
        // Helper function to read a single analog channel and write it to the target watchable.
        let read_temp = move |target: &Watchable<Celsius>, channel: ur20::Address| {
            let Some(ur20::ChannelValue::Decimal32(t_celsius)) = values.get(&channel) else {
                if channel != channels::CUSTOM_TEMPERATURE {
                    // not always installed
                    tracing::warn!(?channel, value = ?values.get(&channel), "Temperature reading failed, invalid channel type");
                }
                let _ = target.update(Celsius(f32::NAN));
                return;
            };
            let _ = target.update(Celsius(*t_celsius));
        };
        // Helper function to read a single digital channel and write it to the target watchable.
        let read_coil = move |target: &Watchable<DigitalIn>, channel: ur20::Address| {
            let Some(ur20::ChannelValue::Bit(val)) = values.get(&channel) else {
                tracing::warn!(?channel, value = ?values.get(&channel), "Coil reading failed, invalid channel type");
                return;
            };
            let _ = target.update(if *val { DigitalIn::On } else { DigitalIn::Off });
        };

        let Self {
            lox_temperature,
            custom_temperature,
            cabinet_temperature,
            ambient_temperature,
            valves_armed,
            igniter_armed,
        } = self;

        read_temp(lox_temperature, channels::LOX_TEMPERATURE);
        read_temp(custom_temperature, channels::CUSTOM_TEMPERATURE);
        read_temp(cabinet_temperature, channels::CABINET_TEMPERATURE);
        read_temp(ambient_temperature, channels::AMBIENT_TEMPERATURE);
        read_coil(valves_armed, channels::VALVE_ARMING_CIRCUIT);

        let Some(ur20::ChannelValue::SilPFIn(sil)) = values.get(&channels::IGNITER_ARMING_CIRCUIT)
        else {
            tracing::warn!(value = ?values.get(&channels::IGNITER_ARMING_CIRCUIT), "SIL PF reading failed, invalid channel type");
            return;
        };
        let _ = igniter_armed.update(if sil.volt_24_safe_output {
            DigitalIn::On
        } else {
            DigitalIn::Off
        });
    }
}
