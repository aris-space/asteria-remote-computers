use std::collections::BTreeMap;

use datatypes::actuator::Settable;
use messages::datapoints::dp_oxidizer_control_board;
use std::sync::mpsc;

/// The valves in the propellant supply system.
#[derive(Debug, Clone)]
pub struct PSSValves {
    // tx for CAN messages.
    tx: std::sync::mpsc::Sender<messages::Message>,
}

impl PSSValves {
    // here a bunch of functions that just generates CAN messages and push them into the channel
    fn push_pressurization_vent_valve_state(&self, state: NormallyOpenValve) {
        let tx = self.tx.clone();
        tokio::spawn(async move {
            // The message is sent two times over 100 ms, to be sure if
            // CAN has some problem
            for _ in 0..2 {
                let msg = messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::PressurizationVentControlFC(state.into())
                );
                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel closed; dropping pressurization vent burst");
                    return;
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }   
        })
    }
    
    fn push_fuel_pressurization_valve_state() {
        let tx = self.tx.clone();
        tokio::spawn(async move {
            // The message is sent two times over 100 ms, to be sure if
            // CAN has some problem
            for _ in 0..2 {
                let msg = messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::FuelPressurization(target_pressure)
                );
                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel closed; dropping pressurization vent burst");
                    return;
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }   
        })
    }
    
    fn push_fuel_vent_valve_state() {
        let tx = self.tx.clone();
        tokio::spawn(async move {
            // The message is sent two times over 100 ms, to be sure if
            // CAN has some problem
            for _ in 0..2 {
                let msg = messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::FuelVentControlFC(state.into())
                );
                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel closed; dropping pressurization vent burst");
                    return;
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }   
        })
    }
    
    fn push_oxidizer_pressurization_valve_state() {
        let tx = self.tx.clone();
        tokio::spawn(async move {
            // The message is sent two times over 100 ms, to be sure if
            // CAN has some problem
            for _ in 0..2 {
                let msg = messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::PressurizationVentControlFC(target_pressure)
                );
                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel closed; dropping pressurization vent burst");
                    return;
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }   
        })
    }
    
    fn push_oxidizer_vent_valve_state() {
        let tx = self.tx.clone();
        tokio::spawn(async move {
            // The message is sent two times over 100 ms, to be sure if
            // CAN has some problem
            for _ in 0..2 {
                let msg = messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::OxidizerVentControlFC(state.into())
                );
                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel closed; dropping pressurization vent burst");
                    return;
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }   
        })
    }
    
    fn push_fuel_main_valve_state() {
        let tx = self.tx.clone();
        tokio::spawn(async move {
            // The message is sent two times over 100 ms, to be sure if
            // CAN has some problem
            for _ in 0..2 {
                let msg = messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::FuelMainControlFC(state.into())
                );
                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel closed; dropping pressurization vent burst");
                    return;
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }   
        })
    }
    
    
    
    fn push_oxidizer_main_valve_state() {
        let tx = self.tx.clone();
        tokio::spawn(async move {
            // The message is sent two times over 100 ms, to be sure if
            // CAN has some problem
            for _ in 0..2 {
                let msg = messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::OxidizerMainControlFC(state.into())
                );
                if tx.send(msg).is_err() {
                    tracing::warn!("CAN channel closed; dropping pressurization vent burst");
                    return;
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }   
        })
    }
}

/// See the matching types in [`PSSValves`].
///
/// ```
/// use pss_drivers::output::ValveName;
/// assert_eq!("oss_vnt_vlv".parse::<ValveName>().unwrap(), ValveName::oss_vnt_vlv)
/// ```
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Deserialize, serde::Serialize,
)]
#[allow(non_camel_case_types)]
pub enum ValveName {
    /// The name of [`PSSValves::pressurization_vent_valve`].
    prz_vnt_vlv,
    /// The name of [`PSSValves::fuel_pressurization_valve`].
    fss_prz_vlv,
    /// The name of [`PSSValves::fuel_vent_valve`].
    fss_vnt_vlv,
    /// The name of [`PSSValves::oxidizer_pressurization_valve`].
    oss_prz_vlv,
    /// The name of [`PSSValves::oxidizer_vent_valve`].
    oss_vnt_vlv,
    /// The name of [`PSSValves::fuel_main_valve`].
    fss_mnl_vlv,
    /// The name of [`PSSValves::oxidizer_main_valve`].
    oss_mnl_vlv,
}
impl std::str::FromStr for ValveName {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        use serde::Deserialize as _;
        Self::deserialize(<&str as serde::de::IntoDeserializer>::into_deserializer(s))
            .map_err(|e| e.to_string())
    }
}


/// Returns a map keyed by `ValveName`, with
/// each entry being a boxed `Settable` that builds and sends the right
/// CAN message when `set_value` is called.
pub fn as_settables(&self) -> BTreeMap<ValveName, Box<dyn Settable + Send>> {
        use ValveName::*;
        let mut map: BTreeMap<ValveName, Box<dyn Settable + Send>> = BTreeMap::new();

        map.insert(prz_vnt_vlv, make_setter(self.tx.clone(), |s: NormallyOpenValve| {
            messages::Message::FuelControlBoard(
                dp_fuel_control_board::Message::PressurizationVentControlFC(s.into()),
            )
        }));

        map.insert(fss_prz_vlv, make_setter(self.tx.clone(), |s: DPRValve| {
            messages::Message::FuelControlBoard(
                dp_fuel_control_board::Message::FuelDprConfigFC(s),
            )
        }));

        map.insert(fss_vnt_vlv, make_setter(self.tx.clone(), |s: NormallyOpenValve| {
            messages::Message::FuelControlBoard(
                dp_fuel_control_board::Message::FuelVentControlFC(s.into()),
            )
        }));

        map.insert(oss_prz_vlv, make_setter(self.tx.clone(), |s: DPRValve| {
            messages::Message::OxidizerControlBoard(
                dp_oxidizer_control_board::Message::OxidizerDprConfigFC(s),
            )
        }));

        map.insert(oss_vnt_vlv, make_setter(self.tx.clone(), |s: NormallyClosedValve| {
            messages::Message::OxidizerControlBoard(
                dp_oxidizer_control_board::Message::OxidizerVentControlFC(s.into()),
            )
        }));

        map.insert(fss_mnl_vlv, make_setter(self.tx.clone(), |s: NormallyClosedValve| {
            messages::Message::EngineControlBoard(
                dp_engine_control_board::Message::FuelMainControlFC(s.into()),
            )
        }));

        map.insert(oss_mnl_vlv, make_setter(self.tx.clone(), |s: NormallyClosedValve| {
            messages::Message::EngineControlBoard(
                dp_engine_control_board::Message::OxidizerMainControlFC(s.into()),
            )
        }));

        map
    }
}

impl PSSValves {
    /// Starts logging valve timeseries to csv files in `folder`.
    /// Used for data analysis.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    /// This directly writes on csv the message
    /// 
}
