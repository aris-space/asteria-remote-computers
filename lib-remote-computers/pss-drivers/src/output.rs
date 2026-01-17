use std::marker::PhantomData;
use std::sync::Arc;
use std::{collections::BTreeMap, time::Duration};

use lib_mission::actuator_control::Settable;
use lib_mission::actuator_control::valves::{NormallyClosedValve, NormallyOpenValve};
use lib_mission::reactive::{IntoWatched as _, Watched};
use tokio::task::AbortHandle;
use watchable::Watchable;

use crate::types::DPRValve;

enum DummyMessage {
    A(NormallyClosedValve),
    C(NormallyOpenValve),
    D(DPRValve),
    Q(NormallyClosedValve),
    B(u32),
}

type CanOutChannel<T> = OutChannel<T, DummyMessage>;

struct OutChannel<T, M> {
    ch: std::sync::mpsc::Sender<M>,
    mapper: Arc<dyn Fn(T) -> M + Send + Sync>,
    _input: PhantomData<T>,
}

impl<T, M> Clone for OutChannel<T, M> {
    fn clone(&self) -> Self {
        Self {
            ch: self.ch.clone(),
            mapper: self.mapper.clone(),
            _input: self._input.clone(),
        }
    }
}

impl<T, M> OutChannel<T, M> {
    pub fn new(
        ch: std::sync::mpsc::Sender<M>,
        mapper: impl Fn(T) -> M + Send + Sync + 'static,
    ) -> Self {
        Self {
            ch,
            mapper: Arc::new(mapper),
            _input: PhantomData,
        }
    }

    pub fn send(&self, msg: T) {
        self.ch.send((self.mapper)(msg)).expect("TODO");
    }
}

impl<T: Settable + Default, M> Settable for OutChannel<T, M> {
    fn set_value(&mut self, value: f32) -> Result<(), lib_mission::actuator_control::OutOfRange> {
        let mut x = T::default();
        x.set_value(value)?;
        Ok(self.send(x))
    }
}

#[test]
fn foo() {
    let (tx, rx) = std::sync::mpsc::channel::<DummyMessage>();
    let x = OutChannel::new(tx, DummyMessage::A);
    x.send(NormallyClosedValve::Closed);
}

/// The valves in the propellant supply system.
#[derive(Clone)]
pub struct PSSValves {
    /// The target state for the Pressurization Vent Valve (`PRZ_VNT_VLV`).
    pub pressurization_vent_valve: CanOutChannel<NormallyOpenValve>,
    /// The target state for the Fuel Pressurization Valve (DPR) (`FSS_PRZ_VLV`).
    pub fuel_pressurization_valve: CanOutChannel<DPRValve>,
    /// The target state for the Fuel Vent Valve (`FSS_VNT_VLV`).
    pub fuel_vent_valve: CanOutChannel<NormallyOpenValve>,
    /// The target state for the Oxidizer Pressurization Valve (DPR) (`OSS_PRZ_VLV`).
    pub oxidizer_pressurization_valve: CanOutChannel<DPRValve>,
    /// The target state for the Oxidizer Vent Valve (`OSS_VNT_VLV`).
    /// Note that this should be opened in an abort, despite being normally closed.
    pub oxidizer_vent_valve: CanOutChannel<NormallyClosedValve>,
    /// The target state for the Fuel Main Valve (`FSS_MNL_VLV`).
    pub fuel_main_valve: CanOutChannel<NormallyClosedValve>,
    /// The target state for the Oxidizer Main Valve (`OSS_MNL_VLV`).
    pub oxidizer_main_valve: CanOutChannel<NormallyClosedValve>,
}

// generated
impl PSSValves {
    pub fn new(can_out: std::sync::mpsc::Sender<DummyMessage>) -> Self {
        Self {
            pressurization_vent_valve: CanOutChannel::new(can_out.clone(), DummyMessage::C),
            fuel_pressurization_valve: CanOutChannel::new(can_out.clone(), DummyMessage::D),
            fuel_vent_valve: CanOutChannel::new(can_out.clone(), DummyMessage::C),
            oxidizer_pressurization_valve: CanOutChannel::new(can_out.clone(), DummyMessage::D),
            oxidizer_vent_valve: CanOutChannel::new(can_out.clone(), DummyMessage::A),
            fuel_main_valve: CanOutChannel::new(can_out.clone(), DummyMessage::A),
            oxidizer_main_valve: CanOutChannel::new(can_out.clone(), DummyMessage::Q),
        }
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

impl PSSValves {
    pub fn as_settables(&self) -> BTreeMap<ValveName, Box<dyn Settable + Send>> {
        macro_rules! boxx {
            ($e:expr) => {
                Box::new($e.clone()) as Box<dyn Settable + Send>
            };
        }
        #[allow(clippy::enum_glob_use)] // rustfmt is nice then
        use ValveName::*;
        BTreeMap::from([
            (prz_vnt_vlv, boxx!(self.pressurization_vent_valve)),
            (fss_prz_vlv, boxx!(self.fuel_pressurization_valve)),
            (fss_vnt_vlv, boxx!(self.fuel_vent_valve)),
            (oss_prz_vlv, boxx!(self.oxidizer_pressurization_valve)),
            (oss_vnt_vlv, boxx!(self.oxidizer_vent_valve)),
            (fss_mnl_vlv, boxx!(self.fuel_main_valve)),
            (oss_mnl_vlv, boxx!(self.oxidizer_main_valve)),
        ])
    }
}

impl PSSValves {
    /// Starts logging valve timeseries to csv files in `folder`.
    /// Used for data analysis.
    ///
    /// This spawns tokio tasks and returns immediately after all setup as been completed.
    pub async fn start_logging(
        &self,
        folder: &std::path::Path,
        time: Watchable<lib_mission::util::time::MissionTime>,
    ) -> Result<[AbortHandle; 8], Box<dyn std::error::Error>> {
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

        let PSSValves {
            is_active,
            pressurization_vent_valve,
            fuel_pressurization_valve,
            fuel_vent_valve,
            oxidizer_pressurization_valve,
            oxidizer_vent_valve,
            fuel_main_valve,
            oxidizer_main_valve,
        } = self;

        // Log valve target states
        Ok([
            spawn_serializable_logger!("pss_mode", is_active, ["active [bool]"],),
            spawn_serializable_logger!(
                "target_state_PRZ_VNT_VLV",
                pressurization_vent_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_FSS_PRZ_VLV",
                fuel_pressurization_valve,
                ["target state setpoint or NaN [barg?]"],
            ),
            spawn_serializable_logger!(
                "target_state_FSS_VNT_VLV",
                fuel_vent_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_OSS_PRZ_VLV",
                oxidizer_pressurization_valve,
                ["target state setpoint or NaN [barg?]"],
            ),
            spawn_serializable_logger!(
                "target_state_OSS_VNT_VLV",
                oxidizer_vent_valve,
                ["target state [open|closed]"],
            ),
            spawn_serializable_logger!(
                "target_state_FSS_MNL_VLV",
                fuel_main_valve,
                ["target state [openness %]"],
            ),
            spawn_serializable_logger!(
                "target_state_OSS_MNL_VLV",
                oxidizer_main_valve,
                ["target state [openness %]"],
            ),
        ])
    }
}
