//! This module defines common newtypes for physical sensor readings.

use serde::{Deserialize, Serialize};

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
    fn test_percent_display() {
        assert_eq!(Percent(32).to_string(), "32%");
        assert_eq!(Percent(20).to_string(), "20%");
        assert_eq!(Percent::default().to_string(), "<unknown>%");
    }
}
