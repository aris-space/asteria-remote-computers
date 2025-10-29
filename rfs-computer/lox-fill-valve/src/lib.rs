use tokio_modbus::ExceptionCode;
use tokio_modbus::client::{Reader, Writer};

mod registers {
    /// Input registers: current valve position (f32, 0.0–1.0)
    /// where 0.0 is fully closed and 1.0 is fully open.
    pub const CURRENT_POS: u16 = 0x0000;

    /// Holding registers: target valve position (f32, 0.0–1.0), where
    /// 0.0 is fully closed and 1.0 is fully open.
    pub const TARGET_POS: u16 = 0x0000;
}

/// High-level driver for the Lox Valve Modbus interface.
pub struct LoxValve<I> {
    interface: I,
}

#[derive(Debug, thiserror::Error)]
pub enum LoxValveError {
    #[error("Modbus error: {0}")]
    Interface(#[from] tokio_modbus::Error),

    #[error("Modbus exception: {0:?}")]
    Exception(#[from] ExceptionCode),

    #[error("Invalid response from device")]
    InvalidResponse,
}

impl LoxValveError {
    /// Determines whether a board driver error is because of a timeout.
    /// The time out happens in `tokio_modbus_shared`.
    pub fn is_timeout_err(self) -> bool {
        let LoxValveError::Interface(modbus_e) = self else {
            return false;
        };
        let tokio_modbus::Error::Transport(std_e) = modbus_e else {
            return false;
        };
        std_e.kind() == std::io::ErrorKind::TimedOut
    }
}

impl<I> LoxValve<I> {
    /// Create a new driver instance.
    pub fn new(interface: I) -> Self {
        Self { interface }
    }

    /// Consume the sensor instance and return the underlying interface
    pub fn into_inner(self) -> I {
        self.interface
    }

    /// Get a shared reference to the underlying interface
    ///
    /// **Note**: this function allows access to the underlying interface.
    /// Bypassing driver abstractions may break contracts in the driver if used incorrectly
    pub fn interface(&self) -> &I {
        &self.interface
    }

    /// Get a mutable reference to the underlying interface
    ///
    /// **Note**: this function allows access to the underlying interface.
    /// Bypassing driver abstractions may break contracts in the driver if used incorrectly
    pub fn interface_mut(&mut self) -> &mut I {
        &mut self.interface
    }
}

impl<I> LoxValve<I>
where
    I: Reader,
{
    /// Read current position (0.0–1.0), where 0.0 is fully closed and 1.0 is fully open.
    pub async fn read_current_position(&mut self) -> Result<f32, LoxValveError> {
        let resp = self
            .interface
            .read_input_registers(registers::CURRENT_POS, 2)
            .await??;
        if resp.len() != 2 {
            return Err(LoxValveError::InvalidResponse);
        }
        let bits = (u32::from(resp[0]) << 16) | u32::from(resp[1]);
        Ok(f32::from_bits(bits))
    }

    /// Read the last‐set target position (0.0–1.0), where 0.0 is fully closed and 1.0 is fully open.
    pub async fn read_target_position(&mut self) -> Result<f32, LoxValveError> {
        let resp = self
            .interface
            .read_holding_registers(registers::TARGET_POS, 2)
            .await??;
        if resp.len() != 2 {
            return Err(LoxValveError::InvalidResponse);
        }
        let bits = (u32::from(resp[0]) << 16) | u32::from(resp[1]);
        Ok(f32::from_bits(bits))
    }
}

// todo: implement properly and see what makes sense
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoxValveStatus {
    Initializing,
    Error,
    Ready,
}

impl<I> LoxValve<I>
where
    I: Writer,
{
    /// Set target position (clamped to 0.0–1.0), where 0.0 is fully closed and 1.0 is fully open.
    #[allow(clippy::cast_possible_truncation)] // intentionally cut off bits
    pub async fn set_target_position(&mut self, target: f32) -> Result<(), LoxValveError> {
        let ratio = target.clamp(0.0, 1.0);
        let bits = ratio.to_bits();
        let words = &[(bits >> 16) as u16, bits as u16];
        self.interface
            .write_multiple_registers(registers::TARGET_POS, words)
            .await??;
        Ok(())
    }
}
