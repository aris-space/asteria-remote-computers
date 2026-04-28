use std::collections::BTreeMap;

use datatypes::actuator::Settable;
use messages::datapoints::dp_oxidizer_control_board;
use std::sync::mpsc;

/// The valves in the propellant supply system.
#[derive(Debug, Clone)]
pub struct PSSValves {
    // Now we 
    tx: mpsc::Sender<messages::Message>,
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
                let _ = tx.send(msg);
                tokio::time::sleep(Duration::from_millis(100)).await;
            }   
        })
    }
    
    fn push_fuel_pressurization_valve_state() {
        tokio::spawn(async move {       
              
        })
    }
    
    fn push_fuel_vent_valve_state() {
        tokio::spawn(async move {   
            
        })
    }
    
    fn push_oxidizer_pressurization_valve_state() {
        tokio::spawn(async move {     
              
        })
    }
    
    fn push_oxidizer_vent_valve_state() {
        tokio::spawn(async move {  
            
        })
    }
    
    fn push_fuel_main_valve_state() {
        tokio::spawn(async move {
            
        })
    }
    
    
    
    fn push_oxidizer_main_valve_state() {
        
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


/// ???????? Here 
impl PSSValves {
    pub fn as_settables(&self) -> BTreeMap<ValveName, Box<dyn Settable + Send>> {
        todo!()
}

impl PSSValves {
    /// Starts logging valve timeseries to csv files in `folder`.
    /// Used for data analysis.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    /// This directly writes on csv the message
    /// 
}
