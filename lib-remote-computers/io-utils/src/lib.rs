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
///     #[datapoint("CanMessage::FuelPressure(p) => p")]
///     fuel_tank_pressure: Watchable<BarG>,
/// }
///
/// #[derive(Collector, Default)]
/// #[collector(message_type = "CanMessage", update_expr = "let _ = #field.update(#value);")]
/// struct OxidizerSensors {
///     #[datapoint("CanMessage::OxidizerPressure(p) => p")]
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
