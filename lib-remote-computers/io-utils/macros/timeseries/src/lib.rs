//! The 'Log_Timeseries' derive macro can be used to turn a struct into a 
//! CSV row schema.
//! 
//! ## Basic Usage
//! 
//! ```rust,ignore
//! #[derive(Clone, LogTimeseries)]
//! pub struct PositionData {
//!     #[log(header = "location latitude [°]")]
//!     pub location_latitude: f64,
//!     #[log(header = "location longitude [°]")]
//!     pub location_longitude: f64,
//!     #[log(header = "location altitude [m above sea level]")]
//!     pub location_hamsl: f64,
//!     #[log(header = "location horizontal accuracy [1-sigma]")]
//!     pub horizontal_accuracy: f32,
//!     #[log(header = "location vertical accuracy [1-sigma]")]
//!     pub vertical_accuracy: f32,
//!     /// Not logged.
//!     #[log(skip)]
//!     pub not_logged_field: u64,
//! }
//! ```
//!
//! This generates:
//!
//! ```rust,ignore
//! impl ::lib_mission::util::log::LogTimeseries for PositionData {
//!     const HEADERS: &'static [&'static str] = &[
//!         "location latitude [°]",
//!         "location longitude [°]",
//!         "location altitude [m above sea level]",
//!         "location horizontal accuracy [1-sigma]",
//!         "location vertical accuracy [1-sigma]",
//!     ];
//!     type Row = (f64, f64, f64, f32, f32,);
//!     fn to_row(&self) -> Self::Row {
//!         (
//!             ::core::clone::Clone::clone(&self.location_latitude),
//!             ::core::clone::Clone::clone(&self.location_longitude),
//!             ::core::clone::Clone::clone(&self.location_hamsl),
//!             ::core::clone::Clone::clone(&self.horizontal_accuracy),
//!             ::core::clone::Clone::clone(&self.vertical_accuracy),
//!         )
//!     }
//! }
//! ```
//! 
//! ## Field attributes
//!
//! - `#[log(header = "...")]`: is required for every logged field.
//! - `#[log(skip)]`: omit this field from the CSV.
//!
//! Fields needing a transform (enum→bool, `Option` propagation, reaching into
//! nested structs) should stay as manual `spawn_watchable_logger` calls.
//
//#![allow(clippy::needless_continue)]

use darling::{ast, util::Flag,  FromDeriveInput, FromField};
use proc_macro::TokenStream;
use quote::{quote, quote_spanned};
use syn::{parse_macro_input, spanned::Spanned};

/// Struct-level parse target. Darling enforces the shape at parse time via
/// `supports(struct_named)` — tuple structs, unit structs, enums, and unions
/// are rejected with a clean error before our code runs.
#[derive(Debug, FromDeriveInput)]
#[darling(attributes(log), supports(struct_named))]
struct LogInput {
    ident: syn::Ident,
    generics: syn::Generics,
    data: ast::Data<(), LogField>,
}
 
/// Field-level parse target. Darling populates `header` and `skip` from
/// `#[log(...)]` attributes; unknown keys become errors automatically.
#[derive(Debug, FromField)]
#[darling(attributes(log))]
struct LogField {
    ident: Option<syn::Ident>,
    ty: syn::Type,
 
    /// `#[log(header = "...")]`
    header: Option<String>,
 
    /// `#[log(skip)]`
    skip: Flag,
}

/// Derive macro for implementing the 'LogTimeseries' trait.
/// 
/// See the crate-level docs for usage examples.
#[proc_macro_derive(LogTimeseries, attributes(log))]
pub fn derive_log_timeseries(input: TokenStream) -> TokenStream {
    let raw_input = parse_macro_input!(input as syn::DeriveInput);
    derive_log_timeseries_impl(&raw_input).into()
}

fn derive_log_timeseries_impl(raw_input: &syn::DeriveInput) -> proc_macro2::TokenStream {
    let log_input = match LogInput::from_derive_input(raw_input) {
        Ok(v) => v,
        Err(e) => return e.write_errors(),
    };
    
    let struct_name = &log_input.ident;
    let (impl_generics, _, _) = log_input.generics.split_for_impl();
     
    let fields = log_input
        .data
        .as_ref()
        .take_struct()
        .expect("darling restricted to struct_named")
        .fields;
  
    let mut error = darling::Error::accumulator();
    
    let mut header_strs: Vec<String> = Vec::new();
    let mut row_tys: Vec<&syn::Type> = Vec::new();
    let mut row_exprs: Vec<proc_macro2::TokenStream> = Vec::new();
    
    for field in &fields {
        let field_name = field
            .ident
            .as_ref()
            .expect("darling restricted to struct_named");
        
        if field.skip.is_present() {
            continue;
        }
            
        if field.skip.is_present() && field.header.is_some() {
            error.push(
                darling::Error::custom(
                    "`#[log(skip)]` cannot be combined with `#[log(header = ...)]`",
                )
                .with_span(&field_name.span()),
            );
            continue;
        }
        
        let Some(header) = field.header.clone() else {
            error.push(
                darling::Error::custom(
                    "field needs `#[log(header = \"...\")]` or `#[log(skip)]`",
                )
                .with_span(&field_name.span()),
            );
            continue;
        };
        
        header_strs.push(header);
        row_tys.push(&field.ty);
        row_exprs.push(quote_spanned! { field_name.span() =>
            ::core::clone::Clone::clone(&self.#field_name)
        });
    }
    
    if let Err(err) = error.finish() {
        return err.write_errors();
    }
    
    if header_strs.is_empty() {
        return darling::Error::custom(
            "LogTimeseries struct must have at least one logged field \
             (annotate fields with `#[log(header = \"...\")]`)",
        )
        .with_span(&raw_input.span())
        .write_errors();
    }
    
    quote!{
        #[automatically_derived]
        impl #impl_generics LogTimeseries for #struct_name {
            const HEADERS: &'static [&'static str] = &["lat [°]", "lon [°]", "alt"];
            type Row = ( #( #row_tys, )* );
            fn to_row(&self) -> Self::Row {
                ( #( #row_exprs, )* )
            }
        }
    }
}
// TODO: tests