use datatypes::actuator::{DPRValve, NormallyClosedValve, NormallyOpenValve, OutOfRange, Settable};
use messages::datapoints::{
    dp_engine_control_board, dp_fuel_control_board, dp_oxidizer_control_board,
};
use std::sync::mpsc;
use std::{collections::BTreeMap, marker::PhantomData};
use tokio::time::Duration;

/// Whether the valve commands originate from the Flight Computer or the Remote Filling Station.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandSender {
    FC,
    RFS,
}

/// The valves in the propellant supply system.
#[derive(Debug, Clone)]
pub struct PSSValves {
    tx: std::sync::mpsc::Sender<messages::Message>,
    sender: CommandSender,
}

impl PSSValves {
    /// This method is needed for the OutputTree constructor.
    /// We cannot derive Default for PSSValves because channels do
    /// not have a sensible default value.
    pub fn new(tx: mpsc::Sender<messages::Message>, sender: CommandSender) -> Self {
        Self { tx, sender }
    }
}

/// Seven functions for the different PSSValves messages.
/// The appropriate CAN message for each valve is generated
/// and sent twice within an interval of 100ms.
impl PSSValves {
    /// Controls the pressurization vent valve (`ValveName::prz_copv_vnt`).
    pub fn set_pressurization_vent(&self, state: NormallyOpenValve) {
        send_can_message(
            self.tx.clone(),
            messages::Message::FuelControlBoard(match self.sender {
                CommandSender::FC => {
                    dp_fuel_control_board::Message::PressurizationVentValveControlFC(state)
                }
                CommandSender::RFS => {
                    dp_fuel_control_board::Message::PressurizationVentValveControlRFS(state)
                }
            }),
        );
    }

    /// Controls the fuel pressurization valve (`ValveName::prz_fss_dpr`).
    pub fn set_fuel_pressurization(&self, state: DPRValve) {
        send_can_message(
            self.tx.clone(),
            messages::Message::FuelControlBoard(match self.sender {
                CommandSender::FC => dp_fuel_control_board::Message::FuelDprValveControlFC(state),
                CommandSender::RFS => dp_fuel_control_board::Message::FuelDprValveControlRFS(state),
            }),
        );
    }

    /// Controls the fuel vent valve (`ValveName::fss_tnk_vnt`).
    pub fn set_fuel_vent(&self, state: NormallyOpenValve) {
        send_can_message(
            self.tx.clone(),
            messages::Message::FuelControlBoard(match self.sender {
                CommandSender::FC => dp_fuel_control_board::Message::FuelVentValveControlFC(state),
                CommandSender::RFS => {
                    dp_fuel_control_board::Message::FuelVentValveControlRFS(state)
                }
            }),
        );
    }

    /// Controls the oxidizer pressurization valve (`ValveName::prz_oss_dpr`).
    pub fn set_oxidizer_pressurization(&self, state: DPRValve) {
        send_can_message(
            self.tx.clone(),
            messages::Message::OxidizerControlBoard(match self.sender {
                CommandSender::FC => {
                    dp_oxidizer_control_board::Message::OxidizerDprValveControlFC(state)
                }
                CommandSender::RFS => {
                    dp_oxidizer_control_board::Message::OxidizerDprValveControlRFS(state)
                }
            }),
        );
    }

    /// Controls the oxidizer vent valve (`ValveName::oss_tnk_vnt`).
    pub fn set_oxidizer_vent(&self, state: NormallyOpenValve) {
        send_can_message(
            self.tx.clone(),
            messages::Message::OxidizerControlBoard(match self.sender {
                CommandSender::FC => {
                    dp_oxidizer_control_board::Message::OxidizerVentValveControlFC(state)
                }
                CommandSender::RFS => {
                    dp_oxidizer_control_board::Message::OxidizerVentValveControlRFS(state)
                }
            }),
        );
    }

    /// Controls the fuel main valve (`ValveName::fss_ml_vlv`).
    pub fn set_fuel_main(&self, state: NormallyClosedValve) {
        send_can_message(
            self.tx.clone(),
            messages::Message::EngineControlBoard(match self.sender {
                CommandSender::FC => {
                    dp_engine_control_board::Message::FuelMainValveControlFC(state)
                }
                CommandSender::RFS => {
                    dp_engine_control_board::Message::FuelMainValveControlRFS(state)
                }
            }),
        );
    }

    /// Controls the oxidizer main valve (`ValveName::oss_ml_vlv`).
    pub fn set_oxidizer_main(&self, state: NormallyClosedValve) {
        send_can_message(
            self.tx.clone(),
            messages::Message::EngineControlBoard(match self.sender {
                CommandSender::FC => {
                    dp_engine_control_board::Message::OxidizerMainValveControlFC(state)
                }
                CommandSender::RFS => {
                    dp_engine_control_board::Message::OxidizerMainValveControlRFS(state)
                }
            }),
        );
    }
}

/// Helper function for sending CAN messages twice with interval of 100ms
fn send_can_message(tx: mpsc::Sender<messages::Message>, can_msg: messages::Message) {
    if tx.send(can_msg.clone()).is_err() {
        tracing::warn!("CAN channel closed; dropping PSS message");
        return;
    }
    tokio::spawn(async move {
        tokio::time::sleep(Duration::from_millis(100)).await;
        if tx.send(can_msg).is_err() {
            tracing::warn!("CAN channel closed; dropping PSS burst");
        }
    });
}

/// See the matching types in [`PSSValves`].
///
/// ```
/// use pss_drivers::output::ValveName;
/// assert_eq!("oss_tnk_vnt".parse::<ValveName>().unwrap(), ValveName::oss_tnk_vnt)
/// ```
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Deserialize, serde::Serialize,
)]
#[allow(non_camel_case_types)]
pub enum ValveName {
    /// The name of [`PSSValves::set_pressurization_vent`].
    prz_copv_vnt,
    /// The name of [`PSSValves::set_fuel_pressurization`].
    prz_fss_dpr,
    /// The name of [`PSSValves::set_fuel_vent`].
    fss_tnk_vnt,
    /// The name of [`PSSValves::set_oxidizer_pressurization`].
    prz_oss_dpr,
    /// The name of [`PSSValves::set_oxidizer_vent`].
    oss_tnk_vnt,
    /// The name of [`PSSValves::set_fuel_main`].
    fss_ml_vlv,
    /// The name of [`PSSValves::set_oxidizer_main`].
    oss_ml_vlv,
}

impl std::str::FromStr for ValveName {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        use serde::Deserialize as _;
        Self::deserialize(<&str as serde::de::IntoDeserializer>::into_deserializer(s))
            .map_err(|e| e.to_string())
    }
}

pub trait AsSettables {
    /// Returns a map of [`ValveName`]s with their associated [`Settable`]
    /// that builds and sends the right CAN message when [`Settable::set_value`]
    /// is called.
    fn as_settables(&self) -> BTreeMap<ValveName, Box<dyn Settable + Send>>;
}

impl AsSettables for PSSValves {
    fn as_settables(&self) -> BTreeMap<ValveName, Box<dyn Settable + Send>> {
        let mut map: BTreeMap<ValveName, Box<dyn Settable + Send>> = BTreeMap::new();
        let sender = self.sender;

        map.insert(
            ValveName::prz_copv_vnt,
            make_setter(self.tx.clone(), move |s: NormallyOpenValve| {
                messages::Message::FuelControlBoard(match sender {
                    CommandSender::FC => {
                        dp_fuel_control_board::Message::PressurizationVentValveControlFC(s)
                    }
                    CommandSender::RFS => {
                        dp_fuel_control_board::Message::PressurizationVentValveControlRFS(s)
                    }
                })
            }),
        );

        map.insert(
            ValveName::prz_fss_dpr,
            make_setter(self.tx.clone(), move |s: DPRValve| {
                messages::Message::FuelControlBoard(match sender {
                    CommandSender::FC => dp_fuel_control_board::Message::FuelDprValveControlFC(s),
                    CommandSender::RFS => dp_fuel_control_board::Message::FuelDprValveControlRFS(s),
                })
            }),
        );

        map.insert(
            ValveName::fss_tnk_vnt,
            make_setter(self.tx.clone(), move |s: NormallyOpenValve| {
                messages::Message::FuelControlBoard(match sender {
                    CommandSender::FC => dp_fuel_control_board::Message::FuelVentValveControlFC(s),
                    CommandSender::RFS => {
                        dp_fuel_control_board::Message::FuelVentValveControlRFS(s)
                    }
                })
            }),
        );

        map.insert(
            ValveName::prz_oss_dpr,
            make_setter(self.tx.clone(), move |s: DPRValve| {
                messages::Message::OxidizerControlBoard(match sender {
                    CommandSender::FC => {
                        dp_oxidizer_control_board::Message::OxidizerDprValveControlFC(s)
                    }
                    CommandSender::RFS => {
                        dp_oxidizer_control_board::Message::OxidizerDprValveControlRFS(s)
                    }
                })
            }),
        );

        map.insert(
            ValveName::oss_tnk_vnt,
            make_setter(self.tx.clone(), move |s: NormallyOpenValve| {
                messages::Message::OxidizerControlBoard(match sender {
                    CommandSender::FC => {
                        dp_oxidizer_control_board::Message::OxidizerVentValveControlFC(s)
                    }
                    CommandSender::RFS => {
                        dp_oxidizer_control_board::Message::OxidizerVentValveControlRFS(s)
                    }
                })
            }),
        );

        map.insert(
            ValveName::fss_ml_vlv,
            make_setter(self.tx.clone(), move |s: NormallyClosedValve| {
                messages::Message::EngineControlBoard(match sender {
                    CommandSender::FC => {
                        dp_engine_control_board::Message::FuelMainValveControlFC(s)
                    }
                    CommandSender::RFS => {
                        dp_engine_control_board::Message::FuelMainValveControlRFS(s)
                    }
                })
            }),
        );

        map.insert(
            ValveName::oss_ml_vlv,
            make_setter(self.tx.clone(), move |s: NormallyClosedValve| {
                messages::Message::EngineControlBoard(match sender {
                    CommandSender::FC => {
                        dp_engine_control_board::Message::OxidizerMainValveControlFC(s)
                    }
                    CommandSender::RFS => {
                        dp_engine_control_board::Message::OxidizerMainValveControlRFS(s)
                    }
                })
            }),
        );

        map
    }
}

struct ChannelSetter<V, F>
where
    V: Settable + Default + Send + 'static,
    F: Fn(V) -> messages::Message + Send + 'static,
{
    tx: mpsc::Sender<messages::Message>,
    build_msg: F,
    _phantom: PhantomData<V>,
}

impl<V, F> Settable for ChannelSetter<V, F>
where
    V: Settable + Default + Send + 'static,
    F: Fn(V) -> messages::Message + Send + 'static,
{
    fn set_value(&mut self, value: f32) -> Result<(), OutOfRange> {
        let mut state = V::default();
        state.set_value(value)?;
        let msg = (self.build_msg)(state);
        let _ = self.tx.send(msg);
        Ok(())
    }
}

fn make_setter<V, F>(tx: mpsc::Sender<messages::Message>, build_msg: F) -> Box<dyn Settable + Send>
where
    V: Settable + Default + Send + 'static,
    F: Fn(V) -> messages::Message + Send + 'static,
{
    Box::new(ChannelSetter {
        tx,
        build_msg,
        _phantom: PhantomData,
    })
}
