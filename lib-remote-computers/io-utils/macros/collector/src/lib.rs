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
//!     #[datapoint("TelemetryMessage::Temperature(#value)")]
//!     temperature: Watchable<f32>,
//!
//!     #[datapoint("TelemetryMessage::Pressure(#value)")]
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
//! `#[datapoint("pattern")]`
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
//! #[datapoint("Msg::Value(#value)")]
//!
//! // Nested enum
//! #[datapoint("Msg::SystemA(system_a::Msg::Foo(#value))")]
//! ```
//!

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, Data, DeriveInput, Fields, Lit, Meta, parse::Parser};

/// Derive macro for implementing the `Collector` trait.
///
/// See the crate-level documentation for usage examples.
#[proc_macro_derive(Collector, attributes(collector, datapoint))]
pub fn derive_collector(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);

    let struct_name = &input.ident;

    // Parse the #[collector(...)] attributes
    let mut message_type = None;
    let mut update_expr = None;

    for attr in &input.attrs {
        if attr.path().is_ident("collector") {
            if let Meta::List(meta_list) = &attr.meta {
                let _ = meta_list.parse_nested_meta(|meta| {
                    if meta.path.is_ident("message_type") {
                        let value = meta.value()?;
                        let s: Lit = value.parse()?;
                        if let Lit::Str(lit_str) = s {
                            message_type = Some(lit_str.value());
                        }
                        Ok(())
                    } else if meta.path.is_ident("update_expr") {
                        let value = meta.value()?;
                        let s: Lit = value.parse()?;
                        if let Lit::Str(lit_str) = s {
                            update_expr = Some(lit_str.value());
                        }
                        Ok(())
                    } else {
                        Err(meta.error("unknown collector attribute"))
                    }
                });
            }
        }
    }

    let message_type = message_type.expect("missing required #[collector(message_type = \"...\")] attribute");
    let update_expr = update_expr.expect("missing required #[collector(update_expr = \"...\")] attribute");

    // Parse message_type as a type
    let message_type: syn::Type = syn::parse_str(&message_type)
        .expect("message_type must be a valid Rust type");

    // Extract field datapoints
    let fields = match &input.data {
        Data::Struct(data_struct) => match &data_struct.fields {
            Fields::Named(fields) => &fields.named,
            _ => panic!("Collector can only be derived for structs with named fields"),
        },
        _ => panic!("Collector can only be derived for structs"),
    };

    let mut match_arms = Vec::new();

    for field in fields {
        let field_name = field.ident.as_ref().expect("named field");

        // Look for #[datapoint("...")] attribute
        for attr in &field.attrs {
            if attr.path().is_ident("datapoint") {
                if let Meta::List(meta_list) = &attr.meta {
                    let tokens = &meta_list.tokens;
                    let lit: Lit = syn::parse2(tokens.clone())
                        .expect("datapoint attribute must contain a string literal");

                    if let Lit::Str(lit_str) = lit {
                        let datapoint_spec = lit_str.value();

                        // Replace #value with __value in the pattern
                        let pattern_str = datapoint_spec.replace("#value", "__value");

                        // Parse pattern
                        let pattern: syn::Pat = syn::Pat::parse_multi
                            .parse_str(&pattern_str)
                            .expect("invalid pattern in datapoint");

                        // Generate the update expression with substitutions
                        let update_code = update_expr
                            .replace("#field", &format!("self.{}", field_name))
                            .replace("#value", "__value");

                        let update_tokens: proc_macro2::TokenStream = update_code.parse()
                            .expect("invalid update_expr");

                        match_arms.push(quote! {
                            #pattern => {
                                #update_tokens
                                Ok(())
                            }
                        });
                    }
                }
            }
        }
    }

    if match_arms.is_empty() {
        panic!("at least one field must have a #[datapoint(...)] attribute");
    }

    let expanded = quote! {
        impl io_utils::Collector<#message_type> for #struct_name {
            fn update_from(&mut self, msg: #message_type) -> Result<(), #message_type> {
                match msg {
                    #(#match_arms)*
                    _ => Err(msg),
                }
            }
        }
    };

    TokenStream::from(expanded)
}
