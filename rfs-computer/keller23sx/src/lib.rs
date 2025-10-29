//! # Keller 23SX Pressure Sensor Driver
//!
//! This driver provides an async interface for communicating with Keller 23SX pressure sensors
//! over Modbus RTU. The Keller 23SX is a high-precision pressure sensor with optional temperature
//! measurement capabilities.
use bitflags::bitflags;
use tokio_modbus::ExceptionCode;
use tokio_modbus::client::{Reader, Writer};

/// Modbus register addresses for the Keller 23SX device
mod registers {
    /// Main pressure sensor value (32-bit float)
    pub const REG_P1: u16 = 0x0002;

    /// Secondary pressure sensor value (32-bit float)
    pub const REG_P2: u16 = 0x0004;

    /// Temperature outside sensor 1 value (32-bit float)
    pub const REG_TOB_1: u16 = 0x0008;

    /// Temperature outside sensor 2 value (32-bit float)
    pub const REG_TOB_2: u16 = 0x000A;

    /// Internal temperature sensor value (32-bit float)
    pub const REG_T: u16 = 0x0006;

    /// Base address for bulk reading pressure and temperature of sensor 1
    /// (P1 followed by TOB1), using the alternative mapping.
    pub const REG_P1_TOB1_BASE: u16 = 0x0100;

    /// Base address for bulk reading pressure and temperature of sensor 2
    /// (P2 followed by TOB2), using the alternative mapping.
    pub const REG_P2_TOB2_BASE: u16 = 0x0104;

    /// UART configuration register (baud rate, parity, stop bits) - Read/Write
    pub const REG_UART_CONFIG: u16 = 0x0200;

    /// Device serial number (32-bit value) - Read Only
    pub const REG_SERIAL_NUMBER: u16 = 0x0202;

    /// Pressure channel presence flags - Read Only
    pub const REG_CFG_P: u16 = 0x0204;

    /// Temperature channel presence flags - Read Only
    pub const REG_CFG_T: u16 = 0x0205;

    /// Device status and error flags - Read Only
    pub const REG_STATUS: u16 = 0x020C;

    /// Modbus device address (1-247) - Read/Write
    pub const REG_DEV_ADDR: u16 = 0x020D;

    /// Firmware class and group version - Read Only
    pub const REG_FW_CLASS: u16 = 0x020E;

    /// Firmware build date (year and week) - Read Only
    pub const REG_FW_DATE: u16 = 0x020F;
}

/// Driver for the [Keller 23SX pressure sensor](https://keller-pressure.com/en/products/pressure-transmitters/standard-pressure-transmitters/series-23sx)
///
/// This struct wraps a MODBUS RTU interface and provides high-level methods
/// for interacting with the sensor according to the KELLER X-Line protocol.
///
/// After each command, the device requires a minimum wait time (T2) of 0.5 ms before the next command,
/// as per the datasheet (see section 3.3, T2). Only one command may be processed at a time.
///
/// Note: The interface must respect Modbus RTU timing, most importantly the 3.5 character idle time
/// between commands.
#[derive(Debug)]
pub struct Keller23SX<I> {
    interface: I,
}

/// Error types that can occur during sensor operations
#[derive(Debug, thiserror::Error)]
pub enum Keller23SXError {
    /// A user-provided parameter is invalid.
    #[error("Invalid parameter")]
    InvalidParameter,

    /// Modbus communication error
    #[error("Modbus interface error: {0}")]
    Interface(#[from] tokio_modbus::Error),

    /// Modbus exception code received from device
    #[error("Modbus exception: {0:?}")]
    Exception(#[from] ExceptionCode),

    /// Invalid data received from device
    #[error("Invalid data received from device")]
    InvalidData,

    /// UART settings error
    #[error("Invalid UART settings: {0}")]
    UartSettings(#[from] UartSettingsError),
}

impl Keller23SXError {
    /// Determines whether a pressure sensor error is because of a timeout.
    /// The time out happens in `tokio_modbus_shared`.
    pub fn is_timeout_err(self) -> bool {
        let Keller23SXError::Interface(modbus_e) = self else {
            return false;
        };
        let tokio_modbus::Error::Transport(std_e) = modbus_e else {
            return false;
        };
        std_e.kind() == std::io::ErrorKind::TimedOut
    }
}

/// Trait for types that can be read from Modbus registers
trait ModbusReadType: Sized {
    /// Read a value of this type from the specified register
    async fn read<I: Reader>(interface: &mut I, reg_addr: u16) -> Result<Self, Keller23SXError>;
}

impl ModbusReadType for u8 {
    /// Truncated 8-bit value read from a 16-bit Modbus register.
    async fn read<I: Reader>(interface: &mut I, reg_addr: u16) -> Result<Self, Keller23SXError> {
        let buffer = interface.read_holding_registers(reg_addr, 1).await??;
        if buffer.len() != 1 {
            return Err(Keller23SXError::InvalidData);
        }
        Ok((buffer[0] & 0xFF) as u8)
    }
}

impl ModbusReadType for u16 {
    async fn read<I: Reader>(interface: &mut I, reg_addr: u16) -> Result<Self, Keller23SXError> {
        let buffer = interface.read_holding_registers(reg_addr, 1).await??;
        if buffer.len() != 1 {
            return Err(Keller23SXError::InvalidData);
        }
        Ok(buffer[0])
    }
}

impl ModbusReadType for f32 {
    async fn read<I: Reader>(interface: &mut I, reg_addr: u16) -> Result<Self, Keller23SXError> {
        let buffer = interface.read_holding_registers(reg_addr, 2).await??;
        if buffer.len() != 2 {
            return Err(Keller23SXError::InvalidData);
        }
        let raw_bits = (u32::from(buffer[0]) << 16) | u32::from(buffer[1]);
        Ok(f32::from_bits(raw_bits))
    }
}

impl ModbusReadType for u32 {
    async fn read<I: Reader>(interface: &mut I, reg_addr: u16) -> Result<Self, Keller23SXError> {
        let buffer = interface.read_holding_registers(reg_addr, 2).await??;
        if buffer.len() != 2 {
            return Err(Keller23SXError::InvalidData);
        }
        Ok((u32::from(buffer[0]) << 16) | u32::from(buffer[1]))
    }
}

impl<const N: usize> ModbusReadType for [f32; N] {
    async fn read<I: Reader>(interface: &mut I, reg_addr: u16) -> Result<Self, Keller23SXError> {
        // We can read at most 120 registers in one go
        if N > 60 {
            return Err(Keller23SXError::InvalidParameter);
        }

        #[allow(clippy::cast_possible_truncation)]
        let buffer = interface
            .read_holding_registers(reg_addr, N as u16 * 2)
            .await??;
        if buffer.len() != N * 2 {
            return Err(Keller23SXError::InvalidData);
        }
        let mut result = [0.0; N];
        for i in 0..N {
            let raw_bits = (u32::from(buffer[i * 2]) << 16) | u32::from(buffer[i * 2 + 1]);
            result[i] = f32::from_bits(raw_bits);
        }
        Ok(result)
    }
}

impl<I> Keller23SX<I> {
    /// Create a new Keller 23SX sensor instance
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

impl<I> Keller23SX<I>
where
    I: Reader,
{
    /// Generic method to read a value from a register
    async fn read<T: ModbusReadType>(&mut self, reg: u16) -> Result<T, Keller23SXError> {
        T::read(&mut self.interface, reg).await
    }

    /// Reads the primary pressure sensor value in `bar`.
    pub async fn read_pressure_sensor1(&mut self) -> Result<f32, Keller23SXError> {
        self.read::<f32>(registers::REG_P1).await
    }

    /// Reads the secondary pressure sensor value in `bar`.
    pub async fn read_pressure_sensor2(&mut self) -> Result<f32, Keller23SXError> {
        self.read::<f32>(registers::REG_P2).await
    }

    /// Read the internal temperature sensor value in `°C`.
    pub async fn read_temperature(&mut self) -> Result<f32, Keller23SXError> {
        self.read::<f32>(registers::REG_T).await
    }

    /// Read the ambient temperature at sensor 1 in `°C`.
    pub async fn read_temperature_outside_sensor_1(&mut self) -> Result<f32, Keller23SXError> {
        self.read::<f32>(registers::REG_TOB_1).await
    }

    /// Read the ambient temperature at sensor 2 in `°C`.
    pub async fn read_temperature_outside_sensor_2(&mut self) -> Result<f32, Keller23SXError> {
        self.read::<f32>(registers::REG_TOB_2).await
    }

    /// Bulk read of the pressure and temperature of sensor 1. This is more efficient than reading
    /// each value separately.
    pub async fn read_pressure_temperature_sensor_1(
        &mut self,
    ) -> Result<(f32, f32), Keller23SXError> {
        let [pres, temp] = self.read::<[f32; 2]>(registers::REG_P1_TOB1_BASE).await?;
        Ok((pres, temp))
    }

    /// Bulk read of pressure and temperature of sensor 2. This is more efficient than reading
    /// each value separately.
    pub async fn read_pressure_temperature_sensor_2(
        &mut self,
    ) -> Result<(f32, f32), Keller23SXError> {
        let [pres, temp] = self.read::<[f32; 2]>(registers::REG_P2_TOB2_BASE).await?;
        Ok((pres, temp))
    }

    /// Read the device serial number
    pub async fn read_serial_number(&mut self) -> Result<u32, Keller23SXError> {
        self.read::<u32>(registers::REG_SERIAL_NUMBER).await
    }

    /// Read the current Modbus device address
    pub async fn read_device_address(&mut self) -> Result<u8, Keller23SXError> {
        let addr = self.read::<u16>(registers::REG_DEV_ADDR).await?;
        if !(1..=247).contains(&addr) {
            return Err(Keller23SXError::InvalidData);
        }
        // no truncation here since we checked the range
        #[allow(clippy::cast_possible_truncation)]
        Ok(addr as u8)
    }

    /// Read the firmware version information (class and group)
    pub async fn read_firmware_class_group(
        &mut self,
    ) -> Result<FirmwareClassGroup, Keller23SXError> {
        let raw = self.read::<u16>(registers::REG_FW_CLASS).await?;
        Ok(FirmwareClassGroup {
            class: (raw >> 8) as u8,
            group: (raw & 0xFF) as u8,
        })
    }

    /// Read the firmware build date information
    pub async fn read_firmware_date(&mut self) -> Result<FirmwareDate, Keller23SXError> {
        let raw = self.read::<u16>(registers::REG_FW_DATE).await?;
        Ok(FirmwareDate {
            year: (raw >> 8) as u8,
            week: (raw & 0xFF) as u8,
        })
    }

    /// Read which pressure channels are available on this device
    pub async fn read_pressure_channel_flags(&mut self) -> Result<ActivePressure, Keller23SXError> {
        let bits = self.read::<u16>(registers::REG_CFG_P).await?;
        Ok(ActivePressure::from_bits_truncate(bits))
    }

    /// Read which temperature channels are available on this device
    pub async fn read_temperature_channel_flags(&mut self) -> Result<ActiveTemp, Keller23SXError> {
        let bits = self.read::<u16>(registers::REG_CFG_T).await?;
        Ok(ActiveTemp::from_bits_truncate(bits))
    }

    /// Read the current device status and error flags
    pub async fn read_status(&mut self) -> Result<Status, Keller23SXError> {
        let bits = self.read::<u16>(registers::REG_STATUS).await?;
        Ok(Status::from_bits_truncate(bits))
    }

    /// Read the current UART configuration
    pub async fn read_uart_settings(&mut self) -> Result<UartSettings, Keller23SXError> {
        let raw = self.read::<u16>(registers::REG_UART_CONFIG).await?;
        Ok(UartSettings::try_from(raw)?)
    }
}

impl<I> Keller23SX<I>
where
    I: Writer + Reader,
{
    /// Set a new Modbus address (1–247).
    ///
    /// Consumes self. On success, returns the interface; on failure, returns self and error.
    pub async fn set_device_address(mut self, address: u16) -> Result<I, (Self, Keller23SXError)> {
        if !(1..=247).contains(&address) {
            return Err((self, Keller23SXError::InvalidParameter));
        }

        let res = async {
            self.interface
                .write_single_register(registers::REG_DEV_ADDR, address)
                .await??;
            Ok(())
        }
        .await;

        match res {
            Ok(()) => Ok(self.into_inner()),
            Err(e) => Err((self, e)),
        }
    }

    /// Write UART settings to the device.
    ///
    /// Consumes self. Returns interface on success, or self and error on failure.
    pub async fn write_uart_settings(
        mut self,
        settings: UartSettings,
    ) -> Result<I, (Self, Keller23SXError)> {
        let bits = settings.to_bits();
        let res = async {
            self.interface
                .write_single_register(registers::REG_UART_CONFIG, bits)
                .await??;
            Ok(())
        }
        .await;

        match res {
            Ok(()) => Ok(self.into_inner()),
            Err(e) => Err((self, e)),
        }
    }
}

/// Firmware version information (class.group format)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareClassGroup {
    /// Firmware class (major version)
    pub class: u8,
    /// Firmware group (minor version)
    pub group: u8,
}

impl std::fmt::Display for FirmwareClassGroup {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}", self.class, self.group)
    }
}

/// Firmware build date information
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareDate {
    /// Year (2-digit format, e.g. 20 for 2020)
    pub year: u8,
    /// Week number (1-52)
    pub week: u8,
}

impl std::fmt::Display for FirmwareDate {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "20{}.{:02}", self.year, self.week)
    }
}

/// Error for invalid raw UART configuration values.
#[derive(Debug, thiserror::Error)]
pub enum UartSettingsError {
    /// Raw value has unknown bits outside the defined UART flags.
    #[error(
        "Invalid UART value {0:#06X}: contains unknown bits outside the defined register flags."
    )]
    UnknownBitsSet(u16),

    /// Reserved bits (1, 2, 3) are set in the raw value.
    #[error(
        "Invalid UART value {0:#06X}: one or more reserved baud rate bits are set. Per the \
         datasheet, bits 1, 2, and 3 of the UART register must be zero."
    )]
    ReservedBaudBitsSet(u16),
}

/// Baud rate configuration for the Keller 23SX device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum BaudRate {
    #[default]
    Baud9600,
    Baud115200,
}

/// Turns the baud rate variant into the raw baud value.
impl From<BaudRate> for u32 {
    fn from(baud: BaudRate) -> u32 {
        match baud {
            BaudRate::Baud9600 => 9600,
            BaudRate::Baud115200 => 115_200,
        }
    }
}

/// UART parity configuration for the Keller 23SX device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Parity {
    #[default]
    None,
    Odd,
    Even,
}

/// UART stop bits configuration for the Keller 23SX device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StopBits {
    #[default]
    One,
    Two,
}

bitflags! {
    /// Each flag corresponds directly to a bit as defined in the datasheet.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct UartRegisterFlags: u16 {
        /// Baud rate setting for 115200 baud. If not set, baud rate is 9600.
        const BAUD_115200 = 0b0000_0000_0000_0001;
        // The remaining baud rate bits are reserved and must be zero.
        const BAUD_RESERVED = 0b0000_0000_0000_1110;

        /// Enables the use of a parity bit.
        const PARITY_ENABLE = 0b0000_0000_0001_0000;
        /// Selects the parity mode (0=Odd, 1=Even) when parity is enabled.
        const PARITY_MODE_EVEN = 0b0000_0000_0010_0000;

        /// Selects two stop bits instead of one.
        const STOP_BIT_TWO = 0b0000_0000_0100_0000;
    }
}

/// UART configuration for the Keller 23SX device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct UartSettings {
    /// Baud rate setting
    pub baud_rate: BaudRate,
    /// Parity bit configuration
    pub parity: Parity,
    /// Number of stop bits
    pub stop_bits: StopBits,
}

impl UartSettings {
    pub fn to_bits(self) -> u16 {
        let mut flags = UartRegisterFlags::empty();

        if self.baud_rate == BaudRate::Baud115200 {
            flags.insert(UartRegisterFlags::BAUD_115200);
        }

        match self.parity {
            Parity::None => {}
            Parity::Odd => flags.insert(UartRegisterFlags::PARITY_ENABLE),
            Parity::Even => {
                flags
                    .insert(UartRegisterFlags::PARITY_ENABLE | UartRegisterFlags::PARITY_MODE_EVEN);
            }
        }

        if self.stop_bits == StopBits::Two {
            flags.insert(UartRegisterFlags::STOP_BIT_TWO);
        }

        flags.bits()
    }
}

impl TryFrom<u16> for UartSettings {
    type Error = UartSettingsError;

    fn try_from(bits: u16) -> Result<Self, Self::Error> {
        let flags =
            UartRegisterFlags::from_bits(bits).ok_or(UartSettingsError::UnknownBitsSet(bits))?;

        // datasheet requires baud rate bits 1, 2, and 3 must be zero
        if flags.intersects(UartRegisterFlags::BAUD_RESERVED) {
            return Err(UartSettingsError::ReservedBaudBitsSet(bits));
        }

        let baud_rate = if flags.contains(UartRegisterFlags::BAUD_115200) {
            BaudRate::Baud115200
        } else {
            BaudRate::Baud9600
        };

        let parity = match (
            flags.contains(UartRegisterFlags::PARITY_ENABLE),
            flags.contains(UartRegisterFlags::PARITY_MODE_EVEN),
        ) {
            (false, _) => Parity::None,
            (true, false) => Parity::Odd,
            (true, true) => Parity::Even,
        };

        let stop_bits = if flags.contains(UartRegisterFlags::STOP_BIT_TWO) {
            StopBits::Two
        } else {
            StopBits::One
        };

        Ok(Self {
            baud_rate,
            parity,
            stop_bits,
        })
    }
}

bitflags! {
    /// Pressure channels available on the device
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ActivePressure: u16 {
        /// Primary pressure sensor (P1)
        const P1 = 1 << 1;
        /// Secondary pressure sensor (P2)
        const P2 = 1 << 2;
    }
}

bitflags! {
    /// Temperature channels available on the device
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ActiveTemp: u16 {
        /// Internal temperature sensor (T)
        const T = 1 << 3;
        /// External temperature sensor 1 (TOB1)
        const TOB1 = 1 << 4;
        /// External temperature sensor 2 (TOB2)
        const TOB2 = 1 << 5;
    }
}

bitflags! {
    /// Device status and error flags, based on the STAT byte definition.
    ///
    /// The STAT byte is an 8-bit value, but is read as part of a 16-bit Modbus register.
    /// The upper 8 bits are not defined.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Status: u16 {
        /// A set CH0 bit indicates that a measuring or computation error has occurred in the channel concerned.
        const CH0_ERROR = 1 << 0;

        /// A set P1 bit indicates that a measuring or computation error has occurred in the channel concerned.
        const P1_ERROR = 1 << 1;

        /// A set P2 bit indicates that a measuring or computation error has occurred in the channel concerned.
        const P2_ERROR = 1 << 2;

        /// A set T bit indicates that a measuring or computation error has occurred in the channel concerned.
        const T_ERROR = 1 << 3;

        /// A set TOB1 bit indicates that a measuring or computation error has occurred in the channel concerned.
        const TOB1_ERROR = 1 << 4;

        /// A set TOB2 bit indicates that a measuring or computation error has occurred in the channel concerned.
        const TOB2_ERROR = 1 << 5;

        /// Named `ERR2` in the datasheet.
        /// A set ERR2 bit denotes that a computation error has occurred in the calculation process for the analogue output.
        /// This occurs if the analogue Signal is in saturation (depends on the scaling).
        const ANALOGUE_OUT_ERROR = 1 << 6;

        /// Named `/STD` in the datasheet.
        /// A set /STD bit indicate whether the transmitter is in Power-up mode, otherwise it is in Standard mode.
        /// For version 5.21-XX.XX, /STD is used to indicate an error during measuring the conductivity.
        const POWER_UP_MODE = 1 << 7;
    }
}

impl Status {
    /// Check if the device has any channel or computation errors.
    pub fn has_errors(&self) -> bool {
        self.intersects(
            Self::CH0_ERROR
                | Self::P1_ERROR
                | Self::P2_ERROR
                | Self::T_ERROR
                | Self::TOB1_ERROR
                | Self::TOB2_ERROR
                | Self::ANALOGUE_OUT_ERROR,
        )
    }

    /// Check if the device is in the power-up state (not yet initialized).
    pub fn is_in_power_up_mode(&self) -> bool {
        self.contains(Self::POWER_UP_MODE)
    }
}
