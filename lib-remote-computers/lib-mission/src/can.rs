//! This module provides abstractions to work with CAN an in particular with the Hermes CAN messages.
//!
//! All valid frames will be logged to a tracing trace-level event with the decimal id and [`hex`] encoded frames.
//! Additionally, the structured messages will be logged in debug representation to debug-level. 
use embedded_can::{Frame, StandardId};
use hermes_can::messages::Message as CanMessage;
use hermes_can::next_valid_length;
use std::collections::BTreeMap;
#[cfg(feature = "benchmark")]
use std::time::{Duration, Instant};
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
pub struct ConsumerRegistry {
    consumers: BTreeMap<StandardId, mpsc::UnboundedSender<CanMessage>>,
}

impl ConsumerRegistry {
    /// Create a new empty registry.
    pub fn new() -> Self {
        Self::default()
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

    /// Forwards a message to all registered consumers.
    ///
    /// Returns the message back if there is no subscriber or the channel has closed.
    fn forward(&self, msg: CanMessage) -> Result<(), CanMessage> {
        let Some(consumer) = self.consumers.get(&msg.id()) else {
            return Err(msg);
        };
        if let Err(e) = consumer.send(msg) {
            return Err(e.0);
        }
        Ok(())
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
#[tracing::instrument(level = "TRACE", name = "can::receiver", skip_all, fields(?consumers))]
pub fn read_can<S: embedded_can::blocking::Can>(
    receiver: &mut S,
    consumers: &ConsumerRegistry,
) -> Option<BenchStats> {
    #[cfg(feature = "benchmark")]
    let init_send = Instant::now();
    #[cfg(feature = "benchmark")]
    let mut send_stats = BenchStats {
        total_time: Duration::ZERO,
        last_mess_time: Duration::ZERO,
        single_messages_time: Vec::new(),
        total_messages: 0,
        total_errors: 0,
        stat_name: "can_receive",
    };

    let mut n = 0;
    loop {
        #[cfg(feature = "benchmark")]
        {
            send_stats
                .single_messages_time
                .push(read_can_once(receiver, consumers, n)?);
            send_stats.total_messages += 1;
            if n > 50 {
                // read can 50 times for testing then return
                send_stats.total_time = init_send.elapsed();
                return Some(send_stats);
            }
        }
        #[cfg(not(feature = "benchmark"))]
        read_can_once(receiver, consumers, n);
        n += 1;
    }
}

/// Read the message, parse it and forward it accordingly.
#[tracing::instrument(level = "DEBUG", name = "can::receive", skip_all, fields(nr = n))]
fn read_can_once<S: embedded_can::blocking::Can>(
    receiver: &mut S,
    consumers: &ConsumerRegistry,
    n: usize,
) -> Option<std::time::Duration> {
    #[cfg(feature = "benchmark")]
    let init_rec = Instant::now();

    let frame = match receiver.receive() {
        Ok(f) => f,
        Err(e) => {
            tracing::warn!(error = ?e, "skipping bad frame, waiting 1ms");
            // If the link is severed, then this causes a infinite loop, thus keep it under control.
            std::thread::sleep(std::time::Duration::from_millis(1));
            return None;
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

    let message = match CanMessage::try_from_parts(id, frame.data()) {
        Ok(m) => m,
        Err(e) => {
            tracing::warn!(error = %e, "failed to decode frame");
            return None;
        }
    };
    #[cfg(feature = "benchmark")]
    let message_time = init_rec.elapsed();

    tracing::debug!(msg = ?message, "received CAN message");

    if consumers.forward(message).is_err() {
        tracing::trace!("skipping frame without consumer");
    }

    #[cfg(feature = "benchmark")]
    return Some(message_time);
    
    #[cfg(not(feature = "benchmark"))]
    None
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
) -> Option<BenchStats> {
    #[cfg(feature = "benchmark")]
    let init_send = Instant::now();
    #[cfg(feature = "benchmark")]
    let mut msg_send_init = Instant::now();
    #[cfg(feature = "benchmark")]
    let mut msg_time = Duration::ZERO;
    #[cfg(feature = "benchmark")]
    let mut send_stats = BenchStats {
        total_time: Duration::ZERO,
        last_mess_time: Duration::ZERO,
        single_messages_time: Vec::new(),
        total_messages: 0,
        total_errors: 0,
        stat_name: "can_transmit",
    };

    // Blocking iterator
    for msg in receiver {
        #[cfg(feature = "benchmark")]
        {
            msg_send_init = Instant::now();
            send_stats.total_messages += 1;
        }

        tracing::debug!(?msg, "transmitting CAN message");

        let mut buf = [0; 64];
        let (id, raw_len) = match msg.try_write_into(&mut buf) {
            Ok(x) => x,
            Err(e) => {
                tracing::error!(error = %e, "cannot encode message!");
                #[cfg(feature = "benchmark")]
                {
                    send_stats.total_errors += 1;
                }
                continue;
            }
        };
        // probably redundant, as `socketcan` also does this extension.
        let Some(len) = next_valid_length(raw_len) else {
            // should always already fail above, since the buf is at most 64 bytes.
            tracing::error!(len = raw_len, "unreachable: message too large!");
            #[cfg(feature = "benchmark")]
            {
                send_stats.total_errors += 1;
            }
            continue;
        };
        let Some(frame) = S::Frame::new(id, &buf[0..len]) else {
            // should always already fail above, since the buf is at most 64 bytes.
            tracing::error!(len = raw_len, "unreachable: message too large!");
            #[cfg(feature = "benchmark")]
            {
                send_stats.total_errors += 1;
            }
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
            #[cfg(feature = "benchmark")]
            {
                send_stats.total_errors += 1;
            }
        }

        #[cfg(feature = "benchmark")]
        {
            msg_time = msg_send_init.elapsed();
            send_stats.single_messages_time.push(msg_time);
            send_stats.last_mess_time = msg_time;
        }
    }

    #[cfg(feature = "benchmark")]
    {
        send_stats.total_time = init_send.elapsed();
    }

    tracing::warn!("Exiting run_can_transmit_actor, should only happen at shutdown");

    #[cfg(feature = "benchmark")]
    return Some(send_stats);

    #[cfg(not(feature = "benchmark"))]
    None
}

#[cfg_attr(not(feature = "benchmark"), allow(dead_code))]
#[derive(Clone)]
pub struct BenchStats {
    pub total_time: std::time::Duration,
    pub last_mess_time: std::time::Duration,
    pub single_messages_time: Vec<std::time::Duration>,
    pub total_messages: i32,
    pub total_errors: i32,
    pub stat_name: &'static str,
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
