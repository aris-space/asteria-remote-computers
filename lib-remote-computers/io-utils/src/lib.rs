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

/// Spawns a telemetry broadcast task for all tracked values in the struct.
///
/// This is best derived by [`Broadcast`][macro@Broadcast].
///
/// ## Usage
///
/// ```rust,ignore
/// #[derive(Broadcast)]
/// #[broadcast(message_type = "CanMessage", loop_type = "PeriodicLoop")]
/// struct ValveState {
///     #[datapoint(pattern = "CanMessage::FuelValve(#value)")]
///     #[broadcast(min_freq_hz = 1., max_freq_hz = 10.)]
///     fuel_valve: Watchable<bool>,
/// }
///
/// let state = ValveState { ... };
/// let (tx, rx) = std::sync::mpsc::channel();
/// let handles: Vec<tokio::task::AbortHandle> = state.start_broadcasting(tx);
///
/// // Later, to stop broadcasting:
/// for handle in handles {
///     handle.abort();
/// }
/// ```
pub trait Broadcast<T> {
    /// When derived, this function spawns one tokio task per annotated field.
    /// Each task reads updates from the field's [`Watchable`][watchable::Watchable]
    /// and forwards them to `channel` in a time aware fashion,
    /// depending on the [`BroadcastLoop`] implementation given to a macro attribute.
    fn start_broadcasting(
        &self,
        channel: std::sync::mpsc::Sender<T>,
    ) -> Vec<tokio::task::AbortHandle>;
}

pub use broadcast_derive::Broadcast;

/// Defines the timing and polling strategy for a [`Broadcast`]-driven task.
///
/// Implement this on a unit struct to control how field values are read and
/// forwarded to the telemetry channel. The macro-generated [`Broadcast`] impl
/// calls this for every annotated field.
///
/// ## Example
///
/// ```rust,ignore
/// struct PeriodicLoop;
///
/// impl<M: Send + 'static> BroadcastLoop<M> for PeriodicLoop {
///     fn broadcast_loop<T: Send + Sync + 'static + Clone>(
///         field: watchable::Watchable<T>,
///         mut map: impl FnMut(T) -> M + Send + 'static,
///         channel: std::sync::mpsc::Sender<M>,
///         min_freq_hz: f32,
///         max_freq_hz: f32,
///     ) -> impl std::future::Future<Output = ()> + Send {
///         use watchable::IntoWatched as _;
///         use watchable::Watched as _;
///         let mut watcher = field.watched();
///         loop {
///             let val = watcher.next().await;
///             if channel.send(map(val)).is_err() {
///                 tracing::error!("telemetry channel closed");
///             }
///         }
///     }
/// }
/// ```
///
/// `min_freq_hz` and `max_freq_hz` are hints for implementations that want to
/// throttle or coalesce updates; a simple change-driven loop may ignore them.
pub trait BroadcastLoop<M> {
    fn broadcast_loop<T: Send + Sync + 'static + Clone>(
        field: watchable::Watchable<T>,
        map: impl FnMut(T) -> M + Send + 'static,
        channel: std::sync::mpsc::Sender<M>,
        min_freq_hz: f32,
        max_freq_hz: f32,
    ) -> impl std::future::Future<Output = NeverReturns> + Send;
}

/// A type which can't be constructed and as such a function which returns this cannot return.
pub enum NeverReturns {}
