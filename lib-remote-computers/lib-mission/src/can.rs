//! This module provides abstractions to work with CAN an in particular with the Hermes CAN messages.
//!
//! All valid frames will be logged to a tracing trace-level event with the decimal id and [`hex`] encoded frames.
//! Additionally, the structured messages will be logged in debug representation to debug-level.

use std::collections::BTreeMap;

use can_hal::{CanDecode as _, CanEncode as _};
use embedded_can::{Frame, StandardId};
use io_utils::Collector;
use messages::Message as CanMessage;
use tokio::sync::mpsc;

/// This collects the consumers of specific can messages.
///
/// A driver should register itself using [`Self::register`], using a self-created sender.
/// It can reuse the channel for other message variants as well, by calling `register` again.
///
/// [`read_can`] will then use this registry to forward messages to the registerd consumers.
///
/// At the moment only one consumer can be registered for every ID, which I think should suffice. Otherwise store a vector.
#[derive(Default)]
pub struct ConsumerRegistry<C = NoopCollector> {
    consumers: BTreeMap<StandardId, mpsc::UnboundedSender<CanMessage>>,
    collector: C,
}

impl ConsumerRegistry {
    /// Create a new empty registry.
    pub fn new() -> Self {
        Self {
            consumers: BTreeMap::default(),
            collector: NoopCollector,
        }
    }
}

impl<C> ConsumerRegistry<C> {
    pub fn with_collector<C2>(self, collector: C2) -> ConsumerRegistry<C2> {
        ConsumerRegistry {
            consumers: self.consumers,
            collector,
        }
    }

    /// Registers to the given channel to receives messages of type `id`.
    ///
    /// Fails if there already is a consumer for this id.
    #[allow(clippy::result_unit_err)] // don't care
    pub fn register(
        &mut self,
        id: StandardId,
        tx: mpsc::UnboundedSender<CanMessage>,
    ) -> Result<(), ()> {
        if self.consumers.contains_key(&id) {
            return Err(());
        }
        self.consumers.insert(id, tx);
        Ok(())
    }
}

impl<C> ConsumerRegistry<C>
where
    C: Collector<CanMessage>,
{
    /// Forwards a message to the registered consumer or the collector.
    ///
    /// Returns the message back if there is no subscriber or the channel has closed.
    fn forward(&mut self, id: StandardId, msg: CanMessage) -> Result<(), CanMessage> {
        let Some(consumer) = self.consumers.get(&id) else {
            return self.collector.update_from(msg);
        };
        if let Err(e) = consumer.send(msg) {
            return Err(e.0);
        }
        Ok(())
    }
}

pub struct NoopCollector;
impl<T> Collector<T> for NoopCollector {
    fn update_from(&mut self, msg: T) -> Result<(), T> {
        Err(msg)
    }
}

impl std::fmt::Debug for ConsumerRegistry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ConsumerRegistry")
            .field("consumers_for", &self.consumers.keys().collect::<Vec<_>>())
            .finish()
    }
}

/// Blocks on reading CAN messages continuously, by calling `receiver`.
/// Received frames are parsed and passed to registers `consumers`.
///
/// Note that this uses the [`embedded_can::Frame`] trait, which technically is meant for CAN 2.0. We abuse this to also carry CAN FD frames.
#[tracing::instrument(level = "TRACE", name = "can::receiver", skip_all)]
pub fn read_can<S: embedded_can::blocking::Can, Col>(
    receiver: &mut S,
    consumers: &mut ConsumerRegistry<Col>,
) where
    Col: Collector<CanMessage>,
{
    let mut n = 0;
    loop {
        read_can_once(receiver, consumers, n);
        n += 1;
    }
}

/// Read the message, parse it and forward it accordingly.
#[tracing::instrument(level = "DEBUG", name = "can::receive", skip_all, fields(nr = n))]
fn read_can_once<S: embedded_can::blocking::Can, Col>(
    receiver: &mut S,
    consumers: &mut ConsumerRegistry<Col>,
    n: usize,
) where
    Col: Collector<CanMessage>,
{
    let frame = match receiver.receive() {
        Ok(f) => f,
        Err(e) => {
            tracing::warn!(error = ?e, "skipping bad frame, waiting 1ms");
            // If the link is severed, then this causes a infinite loop, thus keep it under control.
            std::thread::sleep(std::time::Duration::from_millis(1));
            return;
        }
    };
    let id = match frame.id() {
        embedded_can::Id::Standard(id) => id,
        embedded_can::Id::Extended(extended_id) => extended_id.standard_id(),
    };

    tracing::trace!(
        id = id.as_raw(),
        frame = %hex::encode(frame.data()),
        "received CAN frame"
    );

    let message = match CanMessage::from_parts(id, frame.data()) {
        Ok(m) => m,
        Err(e) => {
            tracing::warn!(error = %e, "failed to decode frame");
            return;
        }
    };
    tracing::debug!(msg = ?message, "received CAN message");

    if consumers.forward(id, message).is_err() {
        tracing::trace!("skipping frame without consumer");
    }
}

/// Blocks on `receiver` reading message by message, encoding and sending them via the `transmitter`.
///
/// This is needed, as only one thread can send at a time. This actor arbitrates the accesses.
/// Writing to the mpsc won't block.
///
/// Even though encoding should never fail, we log and otherwise ignore bad messages.
#[tracing::instrument(level = "DEBUG", name = "can::transmit", skip_all)]
pub fn run_can_transmit_actor<S: embedded_can::blocking::Can>(
    transmitter: &mut S,
    receiver: std::sync::mpsc::Receiver<CanMessage>,
) {
    // initialize writer for all CAN messages
    // Blocking iterator
    for msg in receiver {
        tracing::debug!(?msg, "transmitting CAN message");

        let mut buf = [0; 64];
        let (id, len) = match msg.encode_into(&mut buf) {
            Ok(x) => x,
            Err(e) => {
                tracing::error!(error = %e, "cannot encode message!");
                continue;
            }
        };
        let Some(frame) = S::Frame::new(id, &buf[0..(len as usize)]) else {
            // should always already fail above, since the buf is at most 64 bytes.
            tracing::error!(len, "unreachable: message too large!");
            continue;
        };
        tracing::trace!(
            id = id.as_raw(),
            frame = %hex::encode(frame.data()),
            "transmitting CAN frame"
        );

        let res = transmitter.transmit(&frame);
        if let Err(err) = res {
            tracing::error!(error = ?err, "failed to send CAN message. Dropping it.");
        }
    }
    tracing::warn!("Exiting run_can_transmit_actor, should only happen at shutdown");
}

/// A receive/transmit implementor which is utterly unusable execpt for disabling CAN.
pub struct DummyCan;

impl embedded_can::blocking::Can for DummyCan {
    type Frame = DummyFrame;
    type Error = core::convert::Infallible;

    fn transmit(&mut self, _frame: &Self::Frame) -> Result<(), Self::Error> {
        // ignore
        Ok(())
    }

    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        // block forever to trivially satisfy the type
        loop {
            std::thread::park();
        }
    }
}

/// To be used for testing.
/// Must make sure the vector has valid DLC length.
pub struct DummyFrame(StandardId, Vec<u8>);

/// An `embedded_can` implementation of our owned frame.
/// Technically non-spec, as Frame is for CAN 2.0 frames, but socketcan does the same.
impl Frame for DummyFrame {
    fn id(&self) -> embedded_can::Id {
        embedded_can::Id::Standard(self.0)
    }

    fn dlc(&self) -> usize {
        self.1.len()
    }

    fn data(&self) -> &[u8] {
        &self.1
    }

    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        let embedded_can::Id::Standard(id) = id.into() else {
            unimplemented!("we don't use extended IDs");
        };
        Some(DummyFrame(id, Vec::from(data)))
    }
    fn new_remote(_id: impl Into<embedded_can::Id>, _dlc: usize) -> Option<Self> {
        unimplemented!("new_remote will never be used")
    }
    fn is_extended(&self) -> bool {
        unimplemented!("is_extended will never be used")
    }
    fn is_remote_frame(&self) -> bool {
        unimplemented!("is_remote_frame will never be used")
    }
}
