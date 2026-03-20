/// Implementors of this trait can store messages of type `T`.
///
/// This is best derived by [`Collector`][macro@Collector].
///
/// ## Composable Message Handling
///
/// The `Err(msg)` return allows chaining multiple collectors:
///
/// ```rust,ignore
/// #[derive(Collector, Default)]
/// #[collector(message_type = "CanMessage", update_expr = "let _ = #field.update(#value);")]
/// struct FuelSensors {
///     #[datapoint(pattern = "CanMessage::FuelPressure(#value)")]
///     fuel_tank_pressure: Watchable<BarG>,
/// }
///
/// #[derive(Collector, Default)]
/// #[collector(message_type = "CanMessage", update_expr = "let _ = #field.update(#value);")]
/// struct OxidizerSensors {
///     #[datapoint(pattern = "CanMessage::OxidizerPressure(#value)")]
///     oxidizer_tank_pressure: Watchable<BarG>,
/// }
///
///
/// fn handle_message(&self, msg: CanMessage) {
///     let fuel = FuelSensors::default();
///     let oxidizer = OxidizerSensors::default();
///
///     fuel.update_from(msg)
///         .or_else(|msg| oxidizer.update_from(msg))
///         .unwrap_or_else(|msg| {
///             tracing::warn!("Unhandled message: {:?}", msg);
///         });
/// }
/// ```
///
/// ## Ownership choices
///
/// This trait opted for `&mut self` to allow straight forward field assignment.
/// In cases where instead Cells are used, one should have a cloneable cell type,
/// e.g. `Watchable` or `Arc<Mutex>`.
///
/// `T` is passed by ownership and potentially returned,
/// as often the stored type will be a significant part of the message.
pub trait Collector<T> {
    fn update_from(&mut self, msg: T) -> Result<(), T>;
}

pub use collector_derive::Collector;

/// The feedback for the valves in the propellant supply system.
#[derive(Debug, Default, Clone)]
// derivation to spawn a `broadcast_loop` for every field.
#[derive(Broadcast)]
#[broadcast(message_type = "DummyMessage", loop = "PeriodicLoop")]
pub struct PSSValves {
    /// The current state for the Pressurization Vent Valve (`PRZ_VNT_VLV`).
    pub pressurization_vent_valve: Watchable<NormallyOpenValve>,
    /// The current state for the Fuel Pressurization Valve (DPR) (`FSS_PRZ_VLV`).
    pub fuel_pressurization_valve: Watchable<DPRValve>,
    /// The current state for the Fuel Vent Valve (`FSS_VNT_VLV`).
    pub fuel_vent_valve: Watchable<NormallyOpenValve>,
    /// The current state for the Oxidizer Pressurization Valve (DPR) (`OSS_PRZ_VLV`).
    pub oxidizer_pressurization_valve: Watchable<DPRValve>,
    /// The current state for the Oxidizer Vent Valve (`OSS_VNT_VLV`).
    /// Note that this should be opened in an abort, despite being normally closed.
    #[datapoint(pattern = "DummyMessage::A(#value)")]
    #[broadcast(min_freq_hz = 10, max_freq_hz = 20)]
    pub oxidizer_vent_valve: Watchable<NormallyClosedValve>,
    /// The current state for the Fuel Main Valve (`FSS_MNL_VLV`).
    #[datapoint(pattern = "DummyMessage::B(#value)")]
    #[broadcast(min_freq_hz = 10, max_freq_hz = 20)]
    pub fuel_main_valve: Watchable<NormallyClosedValve>,
    /// The current state for the Oxidizer Main Valve (`OSS_MNL_VLV`).
    pub oxidizer_main_valve: Watchable<NormallyClosedValve>,
}

// generated
impl Broadcast<DummyMessage> for PSSValves {
    fn start_broadcasting(
        &self,
        telemetry_sender: std::sync::mpsc::Sender<DummyMessage>,
    ) -> Vec<Self::Handle> {
        vec![
            tokio::spawn(
                <PeriodicLoop as BroadcastLoop<DummyMessage>>::broadcast_loop(
                    self.oxidizer_vent_valve.clone(), // from field name
                    |v| DummyMessage::A(v),           // from datapoint pattern
                    telemetry_sender.clone(),         // fixed
                    10.,                              // from broadcast attr
                    20.,                              // from broadcast attr
                ),
            ),
            tokio::spawn(
                <PeriodicLoop as BroadcastLoop<DummyMessage>>::broadcast_loop(
                    self.fuel_main_valve.clone(), // from field name
                    |v| DummyMessage::B(v),       // from datapoint pattern
                    telemetry_sender.clone(),     // fixed
                    10.,                          // from broadcast attr
                    20.,                          // from broadcast attr
                ),
            ),
        ]
    }
}
pub trait BroadcastLoop<M> {
    async fn broadcast_loop<T: Send + Sync + 'static + Clone>(
        field: Watchable<T>,
        map: impl FnMut(T) -> M,
        channel: std::sync::mpsc::Sender<M>,
        min_freq_hz: f32,
        max_freq_hz: f32,
    ) -> !;
}

struct PeriodicLoop;

impl BroadcastLoop<M> for PeriodicLoop {
    async fn broadcast_loop<T: Send + Sync + 'static + Clone>(
        field: Watchable<T>,
        mut map: impl FnMut(T) -> DummyMessage,
        channel: std::sync::mpsc::Sender<DummyMessage>,
        min_freq_hz: f32,
        max_freq_hz: f32,
    ) -> ! {
        use lib_mission::reactive::IntoWatched as _;
        use lib_mission::reactive::Watched as _;
        let mut watcher = field.watched();
        loop {
            let val = watcher.next().await;
            let msg = map(val);
            if channel.send(msg).is_err() {
                tracing::error!("buhu :(");
            }
        }
    }
}

pub trait Broadcast<T> {
    fn start_broadcasting(&self, channel: std::sync::mpsc::Sender) -> Vec<tokio::AbortHandle>;
}
