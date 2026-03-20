//! The `Broadcast` derive macro spawns a telemetry broadcast tokio task for each annotated field.
//!
//! For each field annotated with `#[broadcast]`, this generates a
//! [`tokio::spawn`] call that invokes the configured [`BroadcastLoop`][io_utils::BroadcastLoop]
//! implementation.
//!
//! ## Basic Usage
//!
//! ```rust,ignore
//! enum SensorMessage {
//!     Temperature(f32),
//!     Pressure(f32),
//! }
//!
//! #[derive(Broadcast, Default)]
//! #[broadcast(message_type = "SensorMessage", loop_type = "PeriodicLoop")]
//! struct Sensors {
//!     #[datapoint(pattern = "SensorMessage::Temperature(#value)")]
//!     #[broadcast(min_freq_hz = 1., max_freq_hz = 10.)]
//!     temperature: Watchable<f32>,
//!
//!     // Fields without #[broadcast] are ignored.
//!     other: String,
//! }
//! ```
//!
//! This generates something like:
//!
//! ```rust,ignore
//! impl io_utils::Broadcast<SensorMessage> for Sensors {
//!     fn start_broadcasting(
//!         &self,
//!         channel: std::sync::mpsc::Sender<SensorMessage>,
//!     ) -> Vec<tokio::task::AbortHandle> {
//!         vec![
//!             tokio::spawn(
//!                 <PeriodicLoop as io_utils::BroadcastLoop<SensorMessage>>::broadcast_loop(
//!                     self.temperature.clone(),
//!                     |v| SensorMessage::Temperature(v),
//!                     channel.clone(),
//!                     1_f32,
//!                     10_f32,
//!                 ),
//!             )
//!             .abort_handle(),
//!         ]
//!     }
//! }
//! ```
//!
//! ## Struct Attributes
//!
//! `#[broadcast(message_type = ..., loop_type = ...)]`
//!
//! - `message_type` (required): The message type produced by broadcast tasks.
//! - `loop_type` (required): The [`BroadcastLoop`][io_utils::BroadcastLoop] implementor to use.
//!
//! ## Field Attributes
//!
//! For a field to be broadcast it needs to have:
//!
//! - `#[broadcast(min_freq_hz = ..., max_freq_hz = ...)]`: Frequency bounds passed to
//!   [`BroadcastLoop::broadcast_loop`][io_utils::BroadcastLoop::broadcast_loop].
//!   Float literals are required (e.g. `1.` or `1.0`).
//! - `#[datapoint(pattern = "...")]`: Maps the field value to a message variant.
//!   The `#value` placeholder becomes the closure argument.
//!
//! ## Hygiene
//!
//! The macro generates a single impl block for `io_utils::Broadcast<{message_type}>`.
//! It depends on `::std` and `::tokio` to be available.
//!
// The [`darling::FromVariant`] macro seems to have needless continues
#![allow(clippy::needless_continue)]

use darling::{FromDeriveInput, FromField, ast};
use proc_macro::TokenStream;
use proc_macro2::Literal;
use quote::quote;
use syn::parse_macro_input;

/// Struct-level attributes parsed from `#[broadcast(...)]`.
#[derive(Debug, FromDeriveInput)]
#[darling(attributes(broadcast), supports(struct_named))]
struct BroadcastInput {
    ident: syn::Ident,
    data: ast::Data<(), BroadcastField>,

    /// The message type produced by broadcast tasks (e.g., `"SensorMessage"`).
    message_type: syn::Type,

    /// The [`BroadcastLoop`][io_utils::BroadcastLoop] implementor (e.g., `"PeriodicLoop"`).
    /// Named `loop_type` because `loop` is a reserved keyword.
    loop_type: syn::Type,
}

/// Field-level attributes parsed from `#[datapoint(...)]` and `#[broadcast(...)]`.
#[derive(Debug, FromField)]
#[darling(attributes(datapoint, broadcast))]
struct BroadcastField {
    ident: Option<syn::Ident>,

    /// The match pattern containing `#value`, from `#[datapoint(pattern = "...")]`.
    #[darling(default)]
    pattern: Option<String>,

    /// Minimum broadcast frequency in Hz, from `#[broadcast(min_freq_hz = ...)]`.
    #[darling(default)]
    min_freq_hz: Option<f32>,

    /// Maximum broadcast frequency in Hz, from `#[broadcast(max_freq_hz = ...)]`.
    #[darling(default)]
    max_freq_hz: Option<f32>,
}

/// Derive macro for implementing the `Broadcast` trait.
///
/// See the crate-level documentation for usage examples.
#[proc_macro_derive(Broadcast, attributes(broadcast, datapoint))]
pub fn derive_broadcast(input: TokenStream) -> TokenStream {
    let raw_input = parse_macro_input!(input as syn::DeriveInput);
    derive_broadcast_impl(&raw_input).into()
}

/// Inner implementation using proc_macro2 types (testable).
fn derive_broadcast_impl(raw_input: &syn::DeriveInput) -> proc_macro2::TokenStream {
    let broadcast_input = match BroadcastInput::from_derive_input(raw_input) {
        Ok(v) => v,
        Err(e) => return e.write_errors(),
    };

    let struct_name = &broadcast_input.ident;
    let message_type = &broadcast_input.message_type;
    let loop_type = &broadcast_input.loop_type;

    let fields = broadcast_input
        .data
        .as_ref()
        .take_struct()
        .expect("darling restricted to struct_named")
        .fields;

    let mut spawn_exprs = Vec::new();

    for field in fields {
        let field_name = field
            .ident
            .as_ref()
            .expect("darling restricted to struct_named");

        let (Some(min_freq_hz), Some(max_freq_hz)) = (field.min_freq_hz, field.max_freq_hz) else {
            if field.max_freq_hz.is_some() || field.min_freq_hz.is_some() {
                return darling::Error::custom(format!(
                    "field `{field_name}` has #[broadcast] with min_freq_hz or max_freq_hz, but not both"
                ))
                .write_errors();
            }
            // skip fields without #[broadcast]
            continue;
        };

        let Some(ref pattern) = field.pattern else {
            return darling::Error::custom(format!(
                "field `{field_name}` has #[broadcast] but is missing #[datapoint]"
            ))
            .write_errors();
        };

        let closure_body: proc_macro2::TokenStream =
            match pattern.replace("#value", "__value").parse() {
                Ok(t) => t,
                Err(e) => {
                    return darling::Error::custom(format!("invalid pattern: {e}")).write_errors();
                }
            };

        let min_lit = Literal::f32_suffixed(min_freq_hz);
        let max_lit = Literal::f32_suffixed(max_freq_hz);

        spawn_exprs.push(quote! {
            ::tokio::spawn(
                <#loop_type as io_utils::BroadcastLoop<#message_type>>::broadcast_loop(
                    self.#field_name.clone(),
                    |__value| #closure_body,
                    channel.clone(),
                    #min_lit,
                    #max_lit,
                ),
            )
            .abort_handle()
        });
    }

    if spawn_exprs.is_empty() {
        return darling::Error::custom(
            "at least one field must have a #[broadcast(...)] attribute",
        )
        .write_errors();
    }

    quote! {
        impl io_utils::Broadcast<#message_type> for #struct_name {
            fn start_broadcasting(
                &self,
                channel: ::std::sync::mpsc::Sender<#message_type>,
            ) -> ::std::vec::Vec<::tokio::task::AbortHandle> {
                vec![
                    #(#spawn_exprs),*
                ]
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use quote::quote;

    #[test]
    fn test_darling_parsing() {
        let input = quote! {
            #[broadcast(
                message_type = "SensorMessage",
                loop_type = "PeriodicLoop"
            )]
            struct Sensors {
                #[datapoint(pattern = "SensorMessage::Temperature(#value)")]
                #[broadcast(min_freq_hz = 1., max_freq_hz = 10.)]
                temperature: Watchable<f32>,

                // Field without datapoint should be skipped
                other: String,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let broadcast_input =
            BroadcastInput::from_derive_input(&parsed).expect("darling parsing failed");

        let fields = broadcast_input
            .data
            .as_ref()
            .take_struct()
            .expect("should be struct")
            .fields;

        assert_eq!(fields.len(), 2);
        assert_eq!(
            fields[0].pattern,
            Some("SensorMessage::Temperature(#value)".to_string())
        );
        assert!(fields[0].min_freq_hz.is_some());
        assert!(fields[0].max_freq_hz.is_some());
        assert_eq!(fields[1].pattern, None);
    }

    #[test]
    fn test_code_generation() {
        let input = quote! {
            #[broadcast(message_type = "Msg", loop_type = "MyLoop")]
            struct Data {
                #[datapoint(pattern = "Msg::A(#value)")]
                #[broadcast(min_freq_hz = 5., max_freq_hz = 20.)]
                a: Watchable<i32>,

                #[datapoint(pattern = "Msg::B(#value)")]
                #[broadcast(min_freq_hz = 1., max_freq_hz = 2.5)]
                b: Watchable<String>,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_broadcast_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains("impl io_utils :: Broadcast < Msg > for Data"),
            "missing impl header in: {output_str}"
        );
        assert!(
            output_str.contains("fn start_broadcasting"),
            "missing start_broadcasting in: {output_str}"
        );
        assert!(
            output_str.contains("MyLoop as io_utils :: BroadcastLoop < Msg >"),
            "missing loop type in: {output_str}"
        );
        assert!(
            output_str.contains("self . a . clone ()"),
            "missing field a clone in: {output_str}"
        );
        assert!(
            output_str.contains("self . b . clone ()"),
            "missing field b clone in: {output_str}"
        );
        assert!(
            output_str.contains("| v | Msg :: A (v)"),
            "missing closure A in: {output_str}"
        );
        assert!(
            output_str.contains("| v | Msg :: B (v)"),
            "missing closure B in: {output_str}"
        );
        assert!(
            output_str.contains("abort_handle"),
            "missing abort_handle in: {output_str}"
        );
    }

    #[test]
    fn test_error_on_no_datapoints() {
        let input = quote! {
            #[broadcast(message_type = "Msg", loop_type = "MyLoop")]
            struct Empty {
                field: i32,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_broadcast_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains("error"),
            "expected error in: {output_str}"
        );
    }

    #[test]
    fn test_error_on_missing_freq() {
        let input = quote! {
            #[broadcast(message_type = "Msg", loop_type = "MyLoop")]
            struct Data {
                #[datapoint(pattern = "Msg::A(#value)")]
                a: Watchable<i32>,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_broadcast_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains("error"),
            "expected error in: {output_str}"
        );
    }
}
