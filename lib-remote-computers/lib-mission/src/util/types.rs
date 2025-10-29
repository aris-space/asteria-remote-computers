//! This module defines common newtypes for physical sensor readings.

use serde::{Deserialize, Serialize};

/// A temperature in degrees Celsius.
///
/// Is `NAN` by default.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Celsius(pub f32);

impl Celsius {
    /// Gets the degrees Celsius as a raw float.
    pub fn as_f32(self) -> f32 {
        self.0
    }
}

impl Default for Celsius {
    fn default() -> Self {
        Self(f32::NAN)
    }
}

impl std::fmt::Display for Celsius {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.as_f32().is_finite() {
            write!(f, "{:.1}°C", self.as_f32())
        } else {
            write!(f, "<unknown>°C")
        }
    }
}

/// A pressure in bar relative to atmospheric pressure (`barg`).
///
/// Is `NAN` by default.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct BarG(pub f32);

impl BarG {
    /// Gets the barg as a raw float.
    pub fn as_f32(self) -> f32 {
        self.0
    }
}

impl Default for BarG {
    fn default() -> Self {
        Self(f32::NAN)
    }
}

impl std::fmt::Display for BarG {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.as_f32().is_finite() {
            write!(f, "{:.2} barg", self.as_f32())
        } else {
            write!(f, "<unknown> barg")
        }
    }
}

/// An optional percentage (`%`) precise to 1 percent.
///
/// Is some unknown, out of range value by default.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Percent(pub u8);

impl Percent {
    /// Gets the percentage as a raw integer.
    /// This gives a value outide of [0, 100] if the percentage is unknown.
    pub fn as_u8(self) -> u8 {
        self.0
    }
}

impl Default for Percent {
    fn default() -> Self {
        Self(255)
    }
}

impl std::fmt::Display for Percent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.0 <= 100 {
            write!(f, "{}%", self.as_u8())
        } else {
            write!(f, "<unknown>%")
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_celsius_display() {
        assert_eq!(Celsius(32.125).to_string(), "32.1°C");
        assert_eq!(Celsius(20.0).to_string(), "20.0°C");
        assert_eq!(Celsius(f32::NAN).to_string(), "<unknown>°C");
        assert_eq!(Celsius::default().to_string(), "<unknown>°C");
    }
    #[test]
    fn test_barg_display() {
        assert_eq!(BarG(32.125).to_string(), "32.12 barg");
        assert_eq!(BarG(20.0).to_string(), "20.00 barg");
        assert_eq!(BarG(f32::NAN).to_string(), "<unknown> barg");
        assert_eq!(BarG::default().to_string(), "<unknown> barg");
    }
    #[test]
    fn test_percent_display() {
        assert_eq!(Percent(32).to_string(), "32%");
        assert_eq!(Percent(20).to_string(), "20%");
        assert_eq!(Percent::default().to_string(), "<unknown>%");
    }
}
