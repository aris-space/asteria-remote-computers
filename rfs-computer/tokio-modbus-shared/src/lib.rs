//! Shared Modbus device implementation
//!
//! This module provides a shared wrapper around Modbus interfaces, allowing multiple
//! devices with different slave addresses to share the same underlying connection.
//! Inspired by Embassy's I2C shared bus pattern.
//!
//! # Example
//! ```ignore
//! let shared_interface = Arc::new(Mutex::new(tcp_client));
//! let device1 = ModbusDevice::new(shared_interface.clone(), Slave(0x1));
//! let device2 = ModbusDevice::new(shared_interface.clone(), Slave(0x2));
//! ```

use async_trait::async_trait;
use std::io;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use tokio_modbus::client::{Client, Reader, Writer};
use tokio_modbus::prelude::SlaveContext;
use tokio_modbus::{Address, Request, Response, Result, Slave};

/// A slave device on a Modbus that wraps a (shared) interface.
///
/// Useful for sharing a single Modbus interface across multiple (possibly stateful) sensor drivers.
#[derive(Clone, Debug)]
pub struct ModbusDevice<I> {
    /// Shared handle to underlying Modbus interface
    interface: Arc<Mutex<I>>,
    /// Slave address for this device instance
    slave: Slave,
    /// The max time without response before it is considered an error.
    timeout: Duration,
}

impl<I> ModbusDevice<I> {
    /// Creates a new `ModbusDevice` with a shared interface and a specific slave address.
    pub fn new(interface: Arc<Mutex<I>>, slave: Slave, timeout: Duration) -> Self {
        Self {
            interface,
            slave,
            timeout,
        }
    }
}

/// A Modbus register, represented as a 16-bit word.
type Word = u16;
/// A Modbus coil (a single bit), represented as a boolean.
type Coil = bool;
/// The quantity of items (coils or registers) to read/write.
type Quantity = u16;

impl<I> SlaveContext for ModbusDevice<I>
where
    I: SlaveContext,
{
    /// Sets or changes the slave address for this device instance.
    fn set_slave(&mut self, slave: Slave) {
        self.slave = slave;
    }
}

#[async_trait]
impl<I> Client for ModbusDevice<I>
where
    I: Client,
{
    /// Locks the shared interface, sets the target slave address, and forwards the raw request.
    async fn call(&mut self, request: Request<'_>) -> Result<Response> {
        let mut interface = self.interface.lock().await;
        interface.set_slave(self.slave);
        tokio::time::timeout(self.timeout, interface.call(request))
            .await
            .unwrap_or_else(timeout_err)
    }

    /// Locks the shared interface and forwards the disconnect call.
    async fn disconnect(&mut self) -> io::Result<()> {
        let mut interface = self.interface.lock().await;
        // The slave context is probably not needed for disconnect, but we set it for consistency
        interface.set_slave(self.slave);
        interface.disconnect().await
    }
}

#[async_trait]
impl<I> Writer for ModbusDevice<I>
where
    I: Writer,
{
    /// Locks interface, sets slave, and forwards the `write_single_coil` call.
    async fn write_single_coil(&mut self, addr: Address, coil: Coil) -> Result<()> {
        let mut interface = self.interface.lock().await;
        interface.set_slave(self.slave);
        tokio::time::timeout(self.timeout, interface.write_single_coil(addr, coil))
            .await
            .unwrap_or_else(timeout_err)
    }

    /// Locks interface, sets slave, and forwards the `write_single_register` call.
    async fn write_single_register(&mut self, addr: Address, word: Word) -> Result<()> {
        let mut interface = self.interface.lock().await;
        interface.set_slave(self.slave);
        tokio::time::timeout(self.timeout, interface.write_single_register(addr, word))
            .await
            .unwrap_or_else(timeout_err)
    }

    /// Locks interface, sets slave, and forwards the `write_multiple_coils` call.
    async fn write_multiple_coils(&mut self, addr: Address, coils: &'_ [Coil]) -> Result<()> {
        let mut interface = self.interface.lock().await;
        interface.set_slave(self.slave);
        tokio::time::timeout(self.timeout, interface.write_multiple_coils(addr, coils))
            .await
            .unwrap_or_else(timeout_err)
    }

    /// Locks interface, sets slave, and forwards the `write_multiple_registers` call.
    async fn write_multiple_registers(&mut self, addr: Address, words: &[Word]) -> Result<()> {
        let mut interface = self.interface.lock().await;
        interface.set_slave(self.slave);
        tokio::time::timeout(
            self.timeout,
            interface.write_multiple_registers(addr, words),
        )
        .await
        .unwrap_or_else(timeout_err)
    }

    /// Locks interface, sets slave, and forwards the `masked_write_register` call.
    async fn masked_write_register(
        &mut self,
        addr: Address,
        and_mask: Word,
        or_mask: Word,
    ) -> Result<()> {
        let mut interface = self.interface.lock().await;
        interface.set_slave(self.slave);
        tokio::time::timeout(
            self.timeout,
            interface.masked_write_register(addr, and_mask, or_mask),
        )
        .await
        .unwrap_or_else(timeout_err)
    }
}

#[async_trait]
impl<I> Reader for ModbusDevice<I>
where
    I: Reader,
{
    /// Locks interface, sets slave, and forwards the `read_coils` call.
    async fn read_coils(&mut self, addr: Address, cnt: Quantity) -> Result<Vec<Coil>> {
        let mut interface = self.interface.lock().await;
        interface.set_slave(self.slave);
        tokio::time::timeout(self.timeout, interface.read_coils(addr, cnt))
            .await
            .unwrap_or_else(timeout_err)
    }

    /// Locks interface, sets slave, and forwards the `read_discrete_inputs` call.
    async fn read_discrete_inputs(&mut self, addr: Address, cnt: Quantity) -> Result<Vec<Coil>> {
        let mut interface = self.interface.lock().await;
        interface.set_slave(self.slave);
        tokio::time::timeout(self.timeout, interface.read_discrete_inputs(addr, cnt))
            .await
            .unwrap_or_else(timeout_err)
    }

    /// Locks interface, sets slave, and forwards the `read_holding_registers` call.
    async fn read_holding_registers(&mut self, addr: Address, cnt: Quantity) -> Result<Vec<Word>> {
        let mut interface = self.interface.lock().await;
        interface.set_slave(self.slave);
        tokio::time::timeout(self.timeout, interface.read_holding_registers(addr, cnt))
            .await
            .unwrap_or_else(timeout_err)
    }

    /// Locks interface, sets slave, and forwards the `read_input_registers` call.
    async fn read_input_registers(&mut self, addr: Address, cnt: Quantity) -> Result<Vec<Word>> {
        let mut interface = self.interface.lock().await;
        interface.set_slave(self.slave);
        tokio::time::timeout(self.timeout, interface.read_input_registers(addr, cnt))
            .await
            .unwrap_or_else(timeout_err)
    }

    /// Locks interface, sets slave, and forwards the `read_write_multiple_registers` call.
    async fn read_write_multiple_registers(
        &mut self,
        read_addr: Address,
        read_count: Quantity,
        write_addr: Address,
        write_data: &[Word],
    ) -> Result<Vec<Word>> {
        let mut interface = self.interface.lock().await;
        interface.set_slave(self.slave);
        tokio::time::timeout(
            self.timeout,
            interface.read_write_multiple_registers(read_addr, read_count, write_addr, write_data),
        )
        .await
        .unwrap_or_else(timeout_err)
    }
}

/// Makes a modbus error for a timeout
fn timeout_err<T>(_e: tokio::time::error::Elapsed) -> Result<T> {
    Err(std::io::Error::new(io::ErrorKind::TimedOut, "response timeout").into())
}

/// Determines whether an error is based on a device timeout, indicating a not-connected device.
pub fn is_timeout_err(e: tokio_modbus::Error) -> bool {
    let tokio_modbus::Error::Transport(std_e) = e else {
        return false;
    };
    std_e.kind() == std::io::ErrorKind::TimedOut
}
