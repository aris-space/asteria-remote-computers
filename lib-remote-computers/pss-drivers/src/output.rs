use datatypes::actuator::{DPRValve, NormallyClosedValve, NormallyOpenValve, OutOfRange, Settable};
use messages::datapoints::{
    dp_engine_control_board, dp_fuel_control_board, dp_oxidizer_control_board,
};
use std::sync::mpsc;
use std::{collections::BTreeMap, marker::PhantomData};
use tokio::time::Duration;

/// The valves in the propellant supply system.
#[derive(Debug, Clone)]
pub struct PSSValves {
    /// Sender
    tx: std::sync::mpsc::Sender<messages::Message>,
}

impl PSSValves {
    /// This method is needed for the OutputTree constructor.
    /// We cannot derive Default for PSSValves because channels do
    /// not have a sensible default value.
    pub fn new(tx: mpsc::Sender<messages::Message>) -> Self {
        Self { tx }
    }
}

/// Seven functions for the different PSSValves messages.
/// The appropriate CAN message for each valve is generated
/// and sent twice within an interval of 100ms.
impl PSSValves {
    /// Controls the pressurization vent valve (`ValveName::prz_copv_vnt`).
    pub fn set_pressurization_vent(&self, state: NormallyOpenValve) {
        can_helper(
            self.tx.clone(),
            messages::Message::FuelControlBoard(
                dp_fuel_control_board::Message::PressurizationVentValveControlFC(state),
            ),
        );
    }

    /// Controls the fuel pressurization valve (`ValveName::prz_fss_dpr`).
    pub fn set_fuel_pressurization(&self, state: DPRValve) {
        can_helper(
            self.tx.clone(),
            messages::Message::FuelControlBoard(
                dp_fuel_control_board::Message::FuelDprValveControlFC(state),
            ),
        );
    }

    /// Controls the fuel vent valve (`ValveName::fss_tnk_vnt`).
    pub fn set_fuel_vent(&self, state: NormallyOpenValve) {
        can_helper(
            self.tx.clone(),
            messages::Message::FuelControlBoard(
                dp_fuel_control_board::Message::FuelVentValveControlFC(state),
            ),
        );
    }

    /// Controls the oxidizer pressurization valve (`ValveName::prz_oss_dpr`).
    pub fn set_oxidizer_pressurization(&self, state: DPRValve) {
        can_helper(
            self.tx.clone(),
            messages::Message::OxidizerControlBoard(
                dp_oxidizer_control_board::Message::OxidizerDprValveControlFC(state),
            ),
        );
    }

    /// Controls the oxidizer vent valve (`ValveName::oss_tnk_vnt`).
    pub fn set_oxidizer_vent(&self, state: NormallyClosedValve) {
        can_helper(
            self.tx.clone(),
            messages::Message::OxidizerControlBoard(
                dp_oxidizer_control_board::Message::OxidizerVentValveControlFC(state),
            ),
        );
    }

    /// Controls the fuel main valve (`ValveName::fss_ml_vlv`).
    pub fn set_fuel_main(&self, state: NormallyClosedValve) {
        can_helper(
            self.tx.clone(),
            messages::Message::EngineControlBoard(
                dp_engine_control_board::Message::FuelMainValveControlFC(state),
            ),
        );
    }

    /// Controls the oxidizer main valve (`ValveName::oss_ml_vlv`).
    pub fn set_oxidizer_main(&self, state: NormallyClosedValve) {
        can_helper(
            self.tx.clone(),
            messages::Message::EngineControlBoard(
                dp_engine_control_board::Message::OxidizerMainValveControlFC(state),
            ),
        );
    }
}

/// Helper function for sending CAN messages twice with interval of 100ms
fn can_helper(tx: mpsc::Sender<messages::Message>, can_msg: messages::Message) {
    tokio::spawn(async move {
        for _ in 0..2 {
            let msg = can_msg.clone();
            if tx.send(msg).is_err() {
                tracing::warn!("CAN channel closed; dropping PSS burst");
                return;
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    });
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

impl PSSValves {
    /// Returns a map keyed by `ValveName`, with each entry being a boxed
    /// `Settable` that builds and sends the right CAN message when
    /// `set_value` is called.
    pub fn as_settables(&self) -> BTreeMap<ValveName, Box<dyn Settable + Send>> {
        let mut map: BTreeMap<ValveName, Box<dyn Settable + Send>> = BTreeMap::new();

        map.insert(
            ValveName::prz_copv_vnt,
            make_setter(self.tx.clone(), |s: NormallyOpenValve| {
                messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::PressurizationVentValveControlFC(s),
                )
            }),
        );

        map.insert(
            ValveName::prz_fss_dpr,
            make_setter(self.tx.clone(), |s: DPRValve| {
                messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::FuelDprValveControlFC(s),
                )
            }),
        );

        map.insert(
            ValveName::fss_tnk_vnt,
            make_setter(self.tx.clone(), |s: NormallyOpenValve| {
                messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::FuelVentValveControlFC(s),
                )
            }),
        );

        map.insert(
            ValveName::prz_oss_dpr,
            make_setter(self.tx.clone(), |s: DPRValve| {
                messages::Message::OxidizerControlBoard(
                    dp_oxidizer_control_board::Message::OxidizerDprValveControlFC(s),
                )
            }),
        );

        map.insert(
            ValveName::oss_tnk_vnt,
            make_setter(self.tx.clone(), |s: NormallyClosedValve| {
                messages::Message::OxidizerControlBoard(
                    dp_oxidizer_control_board::Message::OxidizerVentValveControlFC(s),
                )
            }),
        );

        map.insert(
            ValveName::fss_ml_vlv,
            make_setter(self.tx.clone(), |s: NormallyClosedValve| {
                messages::Message::EngineControlBoard(
                    dp_engine_control_board::Message::FuelMainValveControlFC(s),
                )
            }),
        );

        map.insert(
            ValveName::oss_ml_vlv,
            make_setter(self.tx.clone(), |s: NormallyClosedValve| {
                messages::Message::EngineControlBoard(
                    dp_engine_control_board::Message::OxidizerMainValveControlFC(s),
                )
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
