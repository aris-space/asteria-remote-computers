//! This `Collector` derive macro can be used to set fields of a struct based on an enum.
//!
//! This reduces boilerplate when implementing message dispatching patterns for structs
//! containing the message's variants. It generates trait implementations that pattern-match on
//! enum message types and update the corresponding fields.
//!
//! This is particularly useful for driver code that receives CAN messages, telemetry updates,
//! or other enum-based message types and needs to dispatch them to the appropriate field.
//!
//! ## Basic Usage
//!
//! ```rust,ignore
//! enum TelemetryMessage {
//!     Temperature(f32),
//!     Pressure(f32),
//! }
//!
//! #[derive(Collector)]
//! #[collector(
//!     message_type = "TelemetryMessage",
//!     update_expr = "let _ = #field.update(#value);"
//! )]
//! struct Sensors {
//!     #[datapoint("TelemetryMessage::Temperature(temp) => Celsius(temp)")]
//!     temperature: Watchable<Celsius>,
//!
//!     #[datapoint("TelemetryMessage::Pressure(p) => BarG(p)")]
//!     pressure: Watchable<BarG>,
//! }
//!
//! // Usage:
//! let msg = TelemetryMessage::Temperature(25.0);
//! sensors.update_from(msg).expect("matched");
//! ```
//!
//! This generates something like:
//!
//! ```rust,ignore
//! impl Collector<TelemetryMessage> for Sensors {
//!     fn update_from(&mut self, msg: TelemetryMessage) -> Result<(), TelemetryMessage> {
//!         match msg {
//!             TelemetryMessage::Temperature(temp) => {
//!                 let __value = Celsius(*temp);
//!                 { let _ = self.temperature.update(__value); }
//!                 Ok(())
//!             }
//!             TelemetryMessage::Pressure(p) => {
//!                 let __value = BarG(*p);
//!                 { let _ = self.pressure.update(__value); }
//!                 Ok(())
//!             }
//!             _ => Err(msg),
//!         }
//!     }
//! }
//! ```
//!
//! ## Explaination
//!
//! Each field with a `datapoint` generates an match arm and create a value (bound to a generated identifier `__value`).
//! This value is then fed into the `update_expr` along with the field, which can update in an arbitrary way.
//! Match arms are generated in the order fields are declared in the struct.
//! The last arm is always includes `_ => Err(msg)` to return unmatched messages for composability.
//!
//! ## Struct Attributes
//!
//! `#[collector(message_type = "...", update_expr = "...")]`
//!
//! - `message_type` (required): The enum type to match against. Must be a valid type path.
//! - `update_expr` (required): A template expression executed when a match occurs.
//!   - `#field`: Replaced with `self.<field_name>`
//!   - `#value`: Replaced with the extracted value from the datapoint pattern
//!
//! ### Examples:
//!
//! ```rust,ignore
//! // Log updates
//! #[collector(
//!     message_type = "LoggedMessage",
//!     update_expr = r#"{
//!         tracing::debug!("Updating {} with {:?}", stringify!(#field), &#value);
//!         #field.update(#value);
//!     }"#
//! )]
//!
//! // Conditional updates
//! #[collector(
//!     message_type = "ValidatedMessage",
//!     update_expr = "if validate(&#value) { #field = #value); }"
//! )]
//! ```
//!
//! ## Field Attributes
//!
//! `#[datapoint("pattern => expression")]`
//!
//! Maps an enum variant to a field update.
//! At most one per field, but at least one field must have an attribute (otherwise the derive is useless).
//!
//! Format: `"<enum_pattern> => <value_expression>"`
//!
//! - `<enum_pattern>`: The match pattern for the enum variant (e.g., `Message::Temperature(t)`),
//!   this must be unique in the struct
//! - `<value_expression>`: Rust expression to extract/transform the value (e.g., `Celsius(t)`),
//!   this becomes available as `#value` in `update_expr`
//!
//! ### Examples:
//!
//! ```rust,ignore
//! // Simple extraction
//! #[datapoint("Msg::Value(v) => v")]
//!
//! // Field access
//! #[datapoint("Msg::Complex(data) => data.field.value")]
//!
//! // Nested enum
//! #[datapoint("Msg::SystemA(system_a::Msg::Foo(x)) => x")]
//! ```
//!
