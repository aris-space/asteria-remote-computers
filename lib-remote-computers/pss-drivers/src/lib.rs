//! This crate gives type definitions and drivers for the propellant supply system, connected to the FC and RFS via CAN.

pub mod can_input_drivers;
pub mod can_output_drivers;
pub mod input;
pub mod output;
pub mod telemetry;
