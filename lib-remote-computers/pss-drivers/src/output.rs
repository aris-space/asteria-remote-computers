use std::{collections::BTreeMap, marker::PhantomData, path::{Path, PathBuf}, sync::OnceLock};
use csv_async::{self, AsyncSerializer, AsyncWriterBuilder};
use datatypes::actuator::{DPRValve, NormallyClosedValve, NormallyOpenValve, OutOfRange, Settable};
use messages::datapoints::{
    dp_engine_control_board, dp_fuel_control_board, dp_oxidizer_control_board
};
use std::sync::mpsc;
use tokio::{fs::File, sync::Mutex, time::Duration};
use futures_util::io::BufWriter;
use tokio_util::compat::TokioAsyncWriteCompatExt;

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
    pub fn new(tx: std::sync::mpsc::Sender<messages::Message>) -> Self {
        Self { tx }
    }
}

impl PSSValves {
    /// Seven functions for the different PSSValves messages.
    /// The appropriate CAN message is generated
    /// and sent twice within an interval of 100ms.
    pub fn set_pressurization_vent(&self, state: NormallyOpenValve) {
        can_helper(self.tx.clone(), move || {
            messages::Message::FuelControlBoard(
                dp_fuel_control_board::Message::PressurizationVentValveControlFC(
                    state.into()
                )
            )
        });
    }

    pub fn set_fuel_pressurization(&self, state: DPRValve) {
        can_helper(self.tx.clone(), move || {
            messages::Message::FuelControlBoard(
                dp_fuel_control_board::Message::FuelDprValveControlFC(
                    state
                ))
        });
    }

    pub fn set_fuel_vent(&self, state: NormallyOpenValve) {
        can_helper(self.tx.clone(), move || {
            messages::Message::FuelControlBoard(
                dp_fuel_control_board::Message::FuelVentValveControlFC(
                state.into(),
            ))
        });
    }

    pub fn set_oxidizer_pressurization(&self, state: DPRValve) {
        can_helper(self.tx.clone(), move || {
            messages::Message::OxidizerControlBoard(
                dp_oxidizer_control_board::Message::OxidizerDprConfigFC(state),
            )
        });
    }

    pub fn set_oxidizer_vent(&self, state: NormallyClosedValve) {
        can_helper(self.tx.clone(), move || {
            messages::Message::OxidizerControlBoard(
                dp_oxidizer_control_board::Message::OxidizerVentControlFC(state.into()),
            )
        });
    }

    pub fn set_fuel_main(&self, state: NormallyClosedValve) {
        can_helper(self.tx.clone(), move || {
            messages::Message::EngineControlBoard(
                dp_engine_control_board::Message::FuelMainControlFC(state.into()),
            )
        });
    }

    pub fn set_oxidizer_main(&self, state: NormallyClosedValve) {
        can_helper(self.tx.clone(), move || {
            messages::Message::EngineControlBoard(
                dp_engine_control_board::Message::OxidizerMainControlFC(state.into()),
            )
        });
    }
}

/// Helper function for sending CAN messages twice with interval of 100ms
/// This method is also used for CSV logging.
fn can_helper<F>(tx: mpsc::Sender<messages::Message>, build_msg: F)
where
    F: Fn() -> messages::Message + Send + 'static,
{
    tokio::spawn(async move {
        // log to csv
        for _ in 0..2 {
            let msg = build_msg();
            if let Err(e) = csv_logger(&msg).await {
                tracing::warn!("PSS log failed: {e}");
            }
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
    /// Returns a map keyed by `ValveName`, with each entry being a boxed
    /// `Settable` that builds and sends the right CAN message when
    /// `set_value` is called.
    pub fn as_settables(&self) -> BTreeMap<ValveName, Box<dyn Settable + Send>> {
        let mut map: BTreeMap<ValveName, Box<dyn Settable + Send>> = BTreeMap::new();

        map.insert(
            ValveName::prz_vnt_vlv,
            make_setter(self.tx.clone(), |s: NormallyOpenValve| {
                messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::PressurizationVentValveControlFC(s.into()),
                )
            }),
        );

        map.insert(
            ValveName::fss_prz_vlv,
            make_setter(self.tx.clone(), |s: DPRValve| {
                messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::FuelDprValveControlFC(s),
                )
            }),
        );

        map.insert(
            ValveName::fss_vnt_vlv,
            make_setter(self.tx.clone(), |s: NormallyOpenValve| {
                messages::Message::FuelControlBoard(
                    dp_fuel_control_board::Message::FuelVentValveControlFC(s.into()),
                )
            }),
        );

        map.insert(
            ValveName::oss_prz_vlv,
            make_setter(self.tx.clone(), |s: DPRValve| {
                messages::Message::OxidizerControlBoard(
                    dp_oxidizer_control_board::Message::OxidizerDprConfigFC(s),
                )
            }),
        );

        map.insert(
            ValveName::oss_vnt_vlv,
            make_setter(self.tx.clone(), |s: NormallyClosedValve| {
                messages::Message::OxidizerControlBoard(
                    dp_oxidizer_control_board::Message::OxidizerVentControlFC(s.into()),
                )
            }),
        );

        map.insert(
            ValveName::fss_mnl_vlv,
            make_setter(self.tx.clone(), |s: NormallyClosedValve| {
                messages::Message::EngineControlBoard(
                    dp_engine_control_board::Message::FuelMainControlFC(s.into()),
                )
            }),
        );

        map.insert(
            ValveName::oss_mnl_vlv,
            make_setter(self.tx.clone(), |s: NormallyClosedValve| {
                messages::Message::EngineControlBoard(
                    dp_engine_control_board::Message::OxidizerMainControlFC(s.into()),
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

static LOG_DIR: OnceLock<PathBuf> = OnceLock::new();
static LOGGER: OnceLock<Mutex<AsyncSerializer<BufWriter<tokio_util::compat::Compat<File>>>>> = OnceLock::new();

pub async fn init_csv_logger(folder: &Path) -> Result<(), anyhow::Error> {
    LOG_DIR.set(folder.to_path_buf()).ok();
    let file = File::create(folder.join("pss_valves.csv")).await?;
    let serializer = AsyncWriterBuilder::new()
        .has_headers(true)
        .create_serializer(BufWriter::new(file.compat_write()));
    LOGGER.set(Mutex::new(serializer)).ok();
    Ok(())
}

// CSV logger: logger is initialized
// once and before in can_helper message is logged to CSV.

async fn csv_logger(msg: &messages::Message) -> Result<(), anyhow::Error> {
    let logger = LOGGER.get().ok_or_else(|| anyhow::anyhow!("CSV logger not initialised"))?;
    
    let mut csv = logger.lock().await;
    let timestamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis();
    
    csv.serialize((timestamp, format!("{:?}", msg)))
        .await
        .map_err(anyhow::Error::from)
}