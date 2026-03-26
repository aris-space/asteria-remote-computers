//! The `Broadcast` derive macro spawns a telemetry broadcast tokio task for each annotated field.
//!
//! For each field annotated with `#[broadcast]`, this generates a
//! `tokio::spawn` call that invokes the configured `io_utils::BroadcastLoop`
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
//!
//!     #[broadcast(flatten)]
//!     nested_fields: NestedSensors,
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
//! - `loop_type` (required): The `io_utils::BroadcastLoop` implementor to use.
//!
//! ## Field Attributes
//!
//! For a field to be broadcast it needs to have:
//!
//! - `#[broadcast(min_freq_hz = ..., max_freq_hz = ...)]`: Frequency bounds passed to
//!   `io_utils::BroadcastLoop::broadcast_loop`.
//!   Float literals are required (e.g. `1.` or `1.0`).
//! - `#[datapoint(pattern = "...")]`: Maps the field value to a message variant.
//!   The `#value` placeholder becomes the closure argument.
//!
//! Alternatively, a field may have the `#[broadcast(flatten)]` attribute,
//! then it delegates broadcasting to the field that itself
//! implements `Broadcast<T>`.
//!
//! ## Hygiene
//!
//! The macro generates a single impl block for `io_utils::Broadcast<{message_type}>`.
//! It depends on `::std` and `::tokio` to be available.
//!
// The [`darling::FromVariant`] macro seems to have needless continues
#![allow(clippy::needless_continue)]

use darling::{FromDeriveInput, FromField, ast, util::Flag};
use proc_macro::TokenStream;
use proc_macro2::Literal;
use quote::quote;
use syn::{parse_macro_input, spanned::Spanned};

/// Struct-level attributes parsed from `#[broadcast(...)]`.
#[derive(Debug, FromDeriveInput)]
#[darling(attributes(broadcast), supports(struct_named))]
struct BroadcastInput {
    ident: syn::Ident,
    data: ast::Data<(), BroadcastField>,

    /// The message type produced by broadcast tasks (e.g., `"SensorMessage"`).
    message_type: syn::Type,

    /// The `BroadcastLoop` implementor (e.g., `"PeriodicLoop"`).
    /// Named `loop_type` because `loop` is a reserved keyword.
    loop_type: syn::Type,
}

/// Field-level attributes parsed from `#[datapoint(...)]` and `#[broadcast(...)]`.
#[derive(Debug, FromField)]
#[darling(attributes(datapoint, broadcast))]
struct BroadcastField {
    ident: Option<syn::Ident>,
    ty: syn::Type,

    /// The match pattern containing `#value`, from `#[datapoint(pattern = "...")]`.
    pattern: Option<String>,

    /// Minimum broadcast frequency in Hz, from `#[broadcast(min_freq_hz = ...)]`.
    min_freq_hz: Option<f32>,

    /// Maximum broadcast frequency in Hz, from `#[broadcast(max_freq_hz = ...)]`.
    max_freq_hz: Option<f32>,

    /// If true, delegate to the field's own `Broadcast` impl via `start_broadcasting`.
    flatten: Flag,
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
#[allow(clippy::too_many_lines)] // but it flows quite linearly
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

    let mut error = darling::Error::accumulator();

    // Each element is a statement that pushes/extends __handles.
    let handle_stmts: Vec<_> = fields
        .iter()
        .filter_map(|field| {
            let field_name = field
                .ident
                .as_ref()
                .expect("darling restricted to struct_named");

            if field.flatten.is_present() {
                if field.min_freq_hz.is_some()
                    || field.max_freq_hz.is_some()
                    || field.pattern.is_some()
                {
                    error.push(
                        darling::Error::custom(format!(
                            "field `{field_name}` has #[broadcast(flatten)] \
                             and must not have min_freq_hz, max_freq_hz, or #[datapoint]"
                        ))
                        .with_span(&field.ident.span()),
                    );
                    return None;
                }
                let field_ty = &field.ty;
                return Some(quote! {
                    __handles.extend(
                        <#field_ty as io_utils::Broadcast<#message_type>>::start_broadcasting(
                            &self.#field_name,
                            channel.clone(),
                        )
                    );
                });
            }

            let (Some(min_freq_hz), Some(max_freq_hz)) = (field.min_freq_hz, field.max_freq_hz)
            else {
                if field.max_freq_hz.is_some() || field.min_freq_hz.is_some() {
                    error.push(darling::Error::custom(format!(
                    "field `{field_name}` has #[broadcast] with min_freq_hz or max_freq_hz, \
                     but not both"
                )).with_span(&field.ident.span()));
                    return None;
                }
                // skip fields without #[broadcast]
                return None;
            };

            let Some(ref pattern) = field.pattern else {
                error.push(
                    darling::Error::custom(format!(
                        "field `{field_name}` has #[broadcast] but is missing #[datapoint]"
                    ))
                    .with_span(&field.ident.span()),
                );
                return None;
            };

            let closure_body = pattern
                .replace("#value", "__value")
                .parse()
                .unwrap_or_else(|err| {
                    darling::Error::custom(format!("invalid pattern: {err}")).write_errors()
                });

            let min_lit = Literal::f32_suffixed(min_freq_hz);
            let max_lit = Literal::f32_suffixed(max_freq_hz);

            Some(quote! {
                __handles.push(
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
                );
            })
        })
        .collect();

    if let Err(err) = error.finish() {
        return err.write_errors();
    }

    if handle_stmts.is_empty() {
        return darling::Error::custom(
            "at least one field must have a #[broadcast(...)] or #[broadcast(flatten)] attribute",
        )
        .write_errors();
    }

    quote! {
        impl io_utils::Broadcast<#message_type> for #struct_name {
            fn start_broadcasting(
                &self,
                channel: ::std::sync::mpsc::Sender<#message_type>,
            ) -> ::std::vec::Vec<::tokio::task::AbortHandle> {
                let mut __handles = ::std::vec::Vec::new();
                #(#handle_stmts)*
                __handles
            }
        }
    }
}

/// Checks some error messages.
/// The functional generated code is checked in an integration test in `io-utils`.
#[cfg(test)]
mod tests {
    use super::*;
    use quote::quote;

    #[test]
    fn test_error_on_flatten_with_freq() {
        let input = quote! {
            #[broadcast(message_type = "Msg", loop_type = "MyLoop")]
            struct Data {
                #[broadcast(flatten, min_freq_hz = 1., max_freq_hz = 2.)]
                inner: Inner,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_broadcast_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains("field `inner` has #[broadcast(flatten)] and must not have min_freq_hz, max_freq_hz, or #[datapoint]"),
            "expected error in: {output_str}"
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
            output_str.contains("at least one field must have a #[broadcast(...)] or #[broadcast(flatten)] attribute"),
            "expected error in: {output_str}"
        );
    }

    #[test]
    fn test_error_on_missing_freq() {
        let input = quote! {
            #[broadcast(message_type = "Msg", loop_type = "MyLoop")]
            struct Data {
                #[datapoint(pattern = "Msg::A(#value)")]
                #[broadcast(min_freq_hz = 1.)]
                a: Watchable<i32>,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_broadcast_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains(
                "field `a` has #[broadcast] with min_freq_hz or max_freq_hz, but not both"
            ),
            "expected error in: {output_str}"
        );
    }

    #[test]
    fn test_error_on_missing_datapoint() {
        let input = quote! {
            #[broadcast(message_type = "Msg", loop_type = "MyLoop")]
            struct Data {
                #[broadcast(min_freq_hz = 1., max_freq_hz = 1.)]
                a: Watchable<i32>,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_broadcast_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains("field `a` has #[broadcast] but is missing #[datapoint]"),
            "expected error in: {output_str}"
        );
    }
}
