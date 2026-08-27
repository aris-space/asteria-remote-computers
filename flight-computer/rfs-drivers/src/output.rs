use std::sync::mpsc;
use std::{collections::BTreeMap, marker::PhantomData};

use datatypes::actuator::{DigitalOut, NormallyClosedValve, OutOfRange, Settable};
use messages::datapoints::dp_rfs_computer;
use serde::{Deserialize, Serialize};

/// A channel that implements `as_settables` for the actuators of the RFS: [`ActuatorName`].
#[derive(Debug, Clone)]
pub struct ActuatorChannel {
    tx: mpsc::Sender<messages::Message>,
}

/// See the matching types in [`crate::input::Actuators`].
///
/// ```
/// use rfs_privers::output::ActuatorName;
/// assert_eq!("gox_ign_vlv".parse::<ActuatorName>().unwrap(), ActuatorName::gox_ign_vlv)
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
#[allow(non_camel_case_types)]
pub enum ActuatorName {
    /// The name for [`crate::input::Actuators::h2_igniter_valve`].
    h2_ign_vlv,
    /// The name for [`crate::input::Actuators::gox_igniter_valve`].
    gox_ign_vlv,
    /// The name for [`crate::input::Actuators::igniter_spark`].
    ign_spk_plg,
    /// The name for [`crate::input::Actuators::gox_purge_valve`].
    gox_prg_vlv,
    /// The name for [`crate::input::Actuators::h2_purge_valve`].
    h2_prg_vlv,
}

impl std::str::FromStr for ActuatorName {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        use serde::Deserialize as _;
        Self::deserialize(<&str as serde::de::IntoDeserializer>::into_deserializer(s))
            .map_err(|e| e.to_string())
    }
}

impl ActuatorChannel {
    /// This method is needed for the OutputTree constructor.
    /// We cannot derive Default for ActuatorChannel for the same 
    /// reason as PSSValves
    pub fn new(tx: mpsc::Sender<messages::Message>) -> Self {
        Self { tx }
    }
}

impl ActuatorChannel {
    /// Returns a map keyed by `ActuatorName`, with each entry being a boxed
    /// `Settable` that builds and sends the right CAN message when
    /// `set_value` is called.
    pub fn as_settables(&self) -> BTreeMap<ActuatorName, Box<dyn Settable + Send>> {
        let mut map: BTreeMap<ActuatorName, Box<dyn Settable + Send>> = BTreeMap::new();

        map.insert(
            ActuatorName::gox_ign_vlv,
            make_setter(self.tx.clone(), move |s: NormallyClosedValve| {
                messages::Message::RfsComputer(dp_rfs_computer::Message::GoxIgniterValveControl(s))
            }),
        );
        map.insert(
            ActuatorName::h2_ign_vlv,
            make_setter(self.tx.clone(), move |s: NormallyClosedValve| {
                messages::Message::RfsComputer(dp_rfs_computer::Message::H2IgniterValveControl(s))
            }),
        );
        map.insert(
            ActuatorName::ign_spk_plg,
            make_setter(self.tx.clone(), move |s: DigitalOut| {
                messages::Message::RfsComputer(dp_rfs_computer::Message::IgniterSparkPlugControl(s))
            }),
        );
        map.insert(
            ActuatorName::h2_prg_vlv,
            make_setter(self.tx.clone(), move |s: NormallyClosedValve| {
                messages::Message::RfsComputer(dp_rfs_computer::Message::H2IgniterPurgeControl(s))
            }),
        );
        map.insert(
            ActuatorName::gox_prg_vlv,
            make_setter(self.tx.clone(), move |s: NormallyClosedValve| {
                messages::Message::RfsComputer(dp_rfs_computer::Message::GoxIgniterPurgeControl(s))
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
