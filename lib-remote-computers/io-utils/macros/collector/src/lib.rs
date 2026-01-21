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
//!     #[datapoint(pattern = "TelemetryMessage::Temperature(#value)")]
//!     temperature: Watchable<f32>,
//!
//!     #[datapoint(pattern = "TelemetryMessage::Pressure(#value)")]
//!     pressure: Watchable<f32>,
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
//!             TelemetryMessage::Temperature(__value) => {
//!                 { let _ = self.temperature.update(__value); }
//!                 Ok(())
//!             }
//!             TelemetryMessage::Pressure(__value) => {
//!                 { let _ = self.pressure.update(__value); }
//!                 Ok(())
//!             }
//!             _ => Err(msg),
//!         }
//!     }
//! }
//! ```
//!
//! ## Explanation
//!
//! Each field with a `datapoint` generates a match arm. The `#value` placeholder in the pattern
//! is replaced with `__value` which captures the matched value and makes it available in `update_expr`.
//! Match arms are generated in the order fields are declared in the struct.
//! The last arm always includes `_ => Err(msg)` to return unmatched messages for composability.
//!
//! ## Struct Attributes
//!
//! `#[collector(message_type = ..., update_expr = "...")]`
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
//!     update_expr = "if validate(&#value) { #field = #value; }"
//! )]
//! ```
//!
//! ## Field Attributes
//!
//! `#[datapoint(pattern = "...")]`
//!
//! Maps an enum variant to a field update.
//! At most one per field, but at least one field must have an attribute (otherwise the derive is useless).
//!
//! The pattern must contain `#value` which will be replaced with `__value` to capture the matched value.
//! This captured value becomes available in `update_expr`.
//!
//! ### Examples:
//!
//! ```rust,ignore
//! // Simple extraction
//! #[datapoint(pattern = "Msg::Value(#value)")]
//!
//! // Nested enum
//! #[datapoint(pattern = "Msg::SystemA(system_a::Msg::Foo(#value))")]
//! ```
//!
//! ## Hygiene
//!
//! The macro generates a single impl block for `io_utils::Collector<{message_type}>`.
//!

use darling::{FromDeriveInput, FromField, ast};
use proc_macro::TokenStream;
use quote::quote;
use syn::parse_macro_input;

/// Struct-level attributes parsed from `#[collector(...)]`.
#[derive(Debug, FromDeriveInput)]
#[darling(attributes(collector), supports(struct_named))]
struct CollectorInput {
    ident: syn::Ident,
    data: ast::Data<(), CollectorField>,

    /// The enum type to match against (e.g., `"TelemetryMessage"`).
    message_type: syn::Type,

    /// Template expression executed when a match occurs.
    /// Use `#field` for `self.<field_name>` and `#value` for the captured value.
    update_expr: String,
}

/// Field-level attributes parsed from `#[datapoint(pattern = "...")]`.
#[derive(Debug, FromField)]
#[darling(attributes(datapoint))]
struct CollectorField {
    ident: Option<syn::Ident>,
    /// The match pattern containing `#value` placeholder.
    #[darling(default)]
    pattern: Option<String>,
}

/// Derive macro for implementing the `Collector` trait.
///
/// See the crate-level documentation for usage examples.
#[proc_macro_derive(Collector, attributes(collector, datapoint))]
pub fn derive_collector(input: TokenStream) -> TokenStream {
    let raw_input = parse_macro_input!(input as syn::DeriveInput);
    derive_collector_impl(&raw_input).into()
}

/// Inner implementation using proc_macro2 types (testable).
fn derive_collector_impl(raw_input: &syn::DeriveInput) -> proc_macro2::TokenStream {
    let collector_input = match CollectorInput::from_derive_input(raw_input) {
        Ok(v) => v,
        Err(e) => return e.write_errors(),
    };

    let struct_name = &collector_input.ident;
    let message_type = &collector_input.message_type;

    // Extract fields from the parsed data
    let fields = collector_input
        .data
        .as_ref()
        .take_struct()
        .expect("darling restricted to struct_named")
        .fields;

    let mut match_arms = Vec::new();

    for field in fields {
        let field_name = field
            .ident
            .as_ref()
            .expect("darling restricted to struct_named");
        let Some(ref datapoint_pattern) = field.pattern else {
            // Field without #[datapoint(...)] - skip it
            continue;
        };

        // Replace #value with __value in the pattern
        let pattern: proc_macro2::TokenStream =
            match datapoint_pattern.replace("#value", "__value").parse() {
                Ok(t) => t,
                Err(e) => {
                    return darling::Error::custom(format!("invalid pattern: {e}")).write_errors();
                }
            };

        // Generate the update expression with substitutions
        let update_code = collector_input
            .update_expr
            .replace("#field", &format!("self.{field_name}"))
            .replace("#value", "__value");

        let update_tokens: proc_macro2::TokenStream = match update_code.parse() {
            Ok(t) => t,
            Err(e) => {
                return darling::Error::custom(format!("invalid update_expr: {e}")).write_errors();
            }
        };

        match_arms.push(quote! {
            #pattern => {
                #update_tokens
                Ok(())
            }
        });
    }

    if match_arms.is_empty() {
        return darling::Error::custom(
            "at least one field must have a #[datapoint(...)] attribute",
        )
        .write_errors();
    }

    quote! {
        impl io_utils::Collector<#message_type> for #struct_name {
            fn update_from(&mut self, msg: #message_type) -> Result<(), #message_type> {
                match msg {
                    #(#match_arms)*
                    _ => Err(msg),
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use quote::{ToTokens, quote};

    #[test]
    fn test_darling_parsing() {
        let input = quote! {
            #[collector(
                message_type = "TelemetryMessage",
                update_expr = "let _ = #field.update(#value);"
            )]
            struct Sensors {
                #[datapoint(pattern = "TelemetryMessage::Temperature(#value)")]
                temperature: f32,

                #[datapoint(pattern = "TelemetryMessage::Pressure(#value)")]
                pressure: f32,

                // Field without datapoint should be skipped
                other: String,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let collector_input =
            CollectorInput::from_derive_input(&parsed).expect("darling parsing failed");

        assert_eq!(
            collector_input.message_type.to_token_stream().to_string(),
            "TelemetryMessage"
        );
        assert_eq!(
            collector_input.update_expr,
            "let _ = #field.update(#value);"
        );

        let fields = collector_input
            .data
            .as_ref()
            .take_struct()
            .expect("should be struct")
            .fields;

        assert_eq!(fields.len(), 3);
        assert_eq!(
            fields[0].pattern,
            Some("TelemetryMessage::Temperature(#value)".to_string())
        );
        assert_eq!(
            fields[1].pattern,
            Some("TelemetryMessage::Pressure(#value)".to_string())
        );
        assert_eq!(fields[2].pattern, None);
    }

    #[test]
    fn test_code_generation() {
        let input = quote! {
            #[collector(
                message_type = "Msg",
                update_expr = "#field = #value;"
            )]
            struct Data {
                #[datapoint(pattern = "Msg::A(#value)")]
                a: i32,

                #[datapoint(pattern = "Msg::B(#value)")]
                b: String,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_collector_impl(&parsed);
        let output_str = output.to_string();

        // Verify the generated code contains expected elements
        assert!(
            output_str.contains("impl io_utils :: Collector < Msg > for Data"),
            "missing impl header in: {output_str}"
        );
        assert!(
            output_str.contains("fn update_from"),
            "missing update_from in: {output_str}"
        );
        assert!(
            output_str.contains("Msg :: A (__value)"),
            "missing pattern A in: {output_str}"
        );
        assert!(
            output_str.contains("Msg :: B (__value)"),
            "missing pattern B in: {output_str}"
        );
        assert!(
            output_str.contains("self . a = __value"),
            "missing field a assignment in: {output_str}"
        );
        assert!(
            output_str.contains("self . b = __value"),
            "missing field b assignment in: {output_str}"
        );
        assert!(
            output_str.contains("Err (msg)"),
            "missing fallback arm in: {output_str}"
        );
    }

    #[test]
    fn test_error_on_no_datapoints() {
        let input = quote! {
            #[collector(
                message_type = "Msg",
                update_expr = "#field = #value;"
            )]
            struct Empty {
                field: i32,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_collector_impl(&parsed);
        let output_str = output.to_string();

        // Should contain a compile error
        assert!(
            output_str.contains("error"),
            "expected error in: {output_str}"
        );
    }
}
