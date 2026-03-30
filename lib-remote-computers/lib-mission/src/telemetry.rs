use std::collections::HashSet;
use std::num::Wrapping;
use std::time::Duration;

use crate::Result;
use crate::actuator_control;
use crate::reactive::{IntoWatched, Watched};
use crate::state_machine::StateName;
use crate::transition;
use crate::util::time::MissionTime;

use datatypes::status::ManualActuatorChange;
use datatypes::status::ManualStateTransition;
use datatypes::status::StateMachineState;
use datatypes::status::UplinkTelemetryHealth;
use mavio::Endpoint;
use mavio::MavLinkId;
use mavio::protocol::SystemId;
use messages::Message;
use messages::datapoints::dp_flight_computer::Message as FCMessage;
use messages::datapoints::dp_rfs_computer::Message as RFSCMessage;
use watchable::Watchable;

#[derive(Debug, Clone, Copy)]
pub enum RemoteSystem {
    FlightComputer,
    RfsComputer,
}

impl RemoteSystem {
    pub fn mavlink_system_id(self) -> SystemId {
        match self {
            RemoteSystem::FlightComputer => 1,
            RemoteSystem::RfsComputer => 2,
        }
    }
}

/// Starts a medium priority thread to read from `socket` and set the given commands.
///
/// Reasoning for medium is that the CAN interaction is more important, but uplink is more important than downlink.
pub fn start_uplink_actor<ActuatorName>(
    socket: std::net::UdpSocket,
    transition_command: Watchable<transition::Command>,
    actuator_command: Watchable<Option<actuator_control::Command<ActuatorName>>>,
    telemetry_health: Watchable<UplinkTelemetryHealth>,
    clock: impl Watched<MissionTime> + Send + 'static,
    states: HashSet<StateName>,
    this_system: RemoteSystem,
) -> std::thread::JoinHandle<io_utils::NeverReturns>
where
    ActuatorName: std::str::FromStr + std::fmt::Debug + PartialEq + Send + Sync + 'static,
{
    std::thread::spawn(move || {
        if let Err(e) = thread_priority::set_current_thread_priority(
            thread_priority::ThreadPriority::Crossplatform(
                thread_priority::ThreadPriorityValue::try_from(40)
                    .expect("constant is in range of 0..100"),
            ),
        ) {
            tracing::warn!("cannot set uplink actor to medium priority: {e}");
        }
        read_uplink(
            &socket,
            &transition_command,
            &actuator_command,
            &telemetry_health,
            &clock,
            &states,
            this_system,
        )
    })
}

/// Blocks on the connection by reading message by message and handling it.
///
/// Whenever a transition command is received through `reader` for the target `this_system`,
/// and the state name corresponds to one from `states`,
/// then the value at `transition_command` is updated.
///
/// When a manual actuator input is received,
/// the actuator name is parsed and written into `actuator_command`.
///
/// The value of `telemetry_health` is also updated, using `clock` as the current time.
#[tracing::instrument(
    level = "DEBUG",
    name = "telemetry::uplink",
    skip_all,
    fields(this_system)
)]
pub fn read_uplink<ActuatorName>(
    reader: &std::net::UdpSocket,
    transition_command: &Watchable<transition::Command>,
    actuator_command: &Watchable<Option<actuator_control::Command<ActuatorName>>>,
    telemetry_health: &Watchable<UplinkTelemetryHealth>,
    clock: &impl Watched<MissionTime>,
    states: &HashSet<StateName>,
    this_system: RemoteSystem,
) -> io_utils::NeverReturns
where
    ActuatorName: std::str::FromStr + std::fmt::Debug + PartialEq,
{
    let mut seq = Wrapping(0);
    loop {
        read_uplink_once(reader, telemetry_health, clock, &mut seq, |msg| {
            msg_processor(
                &msg,
                transition_command,
                actuator_command,
                states,
                this_system,
            )
        });
    }
}

#[tracing::instrument(level = "TRACE", skip_all)]
fn read_uplink_once(
    reader: &std::net::UdpSocket,
    telemetry_health: &Watchable<UplinkTelemetryHealth>,
    clock: &impl Watched<MissionTime>,
    expected_seq: &mut Wrapping<u8>,
    processor: impl FnOnce(Message) -> bool,
) {
    // blockingly receive a frame
    let received = mavlink_hal::transport::recv_message::<Message>(reader);

    let time = clock.current().as_instant_us();
    let (msg, lost) = match received {
        Ok((msg, sequence)) => {
            let lost = u32::from((sequence - *expected_seq).0);
            *expected_seq = sequence + Wrapping(1);
            (msg, lost)
        }
        Err(mavlink_hal::transport::RecvError::Decode { error, sequence }) => {
            tracing::warn!(%error, "skipping bad frame");
            {
                // Critical Section
                let mut health = telemetry_health.write();
                health.time_of_last_msg = time;
                health.num_msg_lost += u32::from((sequence - *expected_seq).0);
                health.num_msg_received += 1;
            }
            *expected_seq = sequence + Wrapping(1);
            return;
        }
        Err(e) => {
            tracing::warn!(error = %e, "failed to receive frame, waiting 5ms");
            // If the link is severed, then this causes a infinite loop, thus keep it under control. Shouldn't happen with UDP
            std::thread::sleep(std::time::Duration::from_millis(5));
            return;
        }
    };

    tracing::debug!(?msg, "received MavLink message"); // fixme: csv logging

    // Critical Section:
    let mut health = telemetry_health.write();
    health.time_of_last_msg = time;
    health.num_msg_lost += lost;
    health.num_msg_received += 1;
    if processor(msg) {
        health.time_of_last_processed_msg = time;
        health.num_msg_processed += 1;
    }
}

/// Process an uplink command, and return if it did so successfully.
///
/// This either sets the `transition_command` value to the received transition command,
/// or the `actuator_command` to the received actuator command.
fn msg_processor<ActuatorName>(
    msg: &Message,
    transition_command: &Watchable<transition::Command>,
    actuator_command: &Watchable<Option<actuator_control::Command<ActuatorName>>>,
    states: &HashSet<StateName>,
    this_system: RemoteSystem,
) -> bool
where
    ActuatorName: std::str::FromStr + std::fmt::Debug + PartialEq,
{
    match msg {
        Message::FlightComputer(FCMessage::ManualStateTransition(ManualStateTransition {
            transition_index,
            state,
        }))
        | Message::RfsComputer(RFSCMessage::ManualStateTransition(ManualStateTransition {
            transition_index,
            state,
        })) => {
            if matches!(msg, Message::FlightComputer(_))
                != matches!(this_system, RemoteSystem::FlightComputer)
            {
                tracing::trace!("skipping transition command for other system");
                return false;
            }

            let Ok(target) = StateName::try_from_array(state, states) else {
                tracing::warn!(
                    received = %String::from_utf8_lossy(state),
                    "skipping transition command to unknown state"
                );
                return false;
            };
            let index = if *transition_index == 0 {
                None
            } else {
                Some(*transition_index as usize)
            };

            let command: transition::Command = Some((index, target));
            tracing::debug!(?command, "received command");
            // we don't care about the old value or whether they were equal
            let _ = transition_command.update(command);

            true
        }
        Message::FlightComputer(FCMessage::ManualActuatorChange(ManualActuatorChange {
            path_index,
            actuator,
            value,
        }))
        | Message::RfsComputer(RFSCMessage::ManualActuatorChange(ManualActuatorChange {
            path_index,
            actuator,
            value,
        })) => {
            if matches!(msg, Message::FlightComputer(_))
                != matches!(this_system, RemoteSystem::FlightComputer)
            {
                tracing::trace!("skipping actuator command for other system");
                return false;
            }

            // Decode the name from zero padded utf8
            let Ok(Ok(actuator_name)) = std::str::from_utf8(actuator)
                .map(|s| ActuatorName::from_str(s.trim_end_matches('\0')))
            else {
                tracing::warn!(
                    received = %String::from_utf8_lossy(actuator),
                    "skipping actuator command for bad actuator"
                );
                return false;
            };

            let command = actuator_control::Command {
                in_state_index: *path_index as usize,
                actuator: actuator_name,
                value: *value,
            };
            tracing::debug!(?command, "received actuator command");

            // we don't care about the old value
            let _ = actuator_command.replace(Some(command));

            true
        }
        // ignore other signals
        _ => false,
    }
}

/// A simple implementor of [`io_utils::BroadcastLoop`] that periodically send the current value.
/// The period used is the average of min and max.
// TODO: make a responsive loop
pub struct PeriodicLoop;

impl io_utils::BroadcastLoop<Message> for PeriodicLoop {
    async fn broadcast_loop<T: Send + Sync + 'static + Clone>(
        field: watchable::Watchable<T>,
        mut map: impl FnMut(T) -> Message + Send + 'static,
        channel: std::sync::mpsc::Sender<Message>,
        min_freq_hz: f32,
        max_freq_hz: f32,
    ) -> io_utils::NeverReturns {
        let period = Duration::from_secs_f32(f32::midpoint(1. / min_freq_hz, 1. / max_freq_hz));
        let mut interval = tokio::time::interval(period);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            interval.tick().await;
            let val = map(field.get());
            if channel.send(val).is_err() {
                tracing::warn!("Telemetry channel was closed. Slowing down!");
                tokio::time::sleep(Duration::from_millis(5001)).await;
                // 5001 such that new clipyy doesn't complain yet
            }
        }
    }
}

/// Starts a low priority thread to read from `channel` and send items as mavlink messages to `socket`.
pub fn start_downlink_actor(
    socket: std::net::UdpSocket,
    channel: std::sync::mpsc::Receiver<Message>,
    this_system: RemoteSystem,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        if let Err(e) =
            thread_priority::set_current_thread_priority(thread_priority::ThreadPriority::Min)
        {
            tracing::warn!("cannot set downlink actor to low priority: {e}");
        }
        downlink_actor(&socket, channel, this_system);
    })
}

/// Blocks on `channel` reading message by message and sending them to MavLink.
///
/// This is needed, as only one thread can send at a time this actor arbitrates the accesses.
/// Writing to the mpsc won't block.
#[tracing::instrument(level = "TRACE", name = "telemetry::downlink", skip_all)]
fn downlink_actor(
    socket: &std::net::UdpSocket,
    channel: std::sync::mpsc::Receiver<Message>,
    this_system: RemoteSystem,
) {
    let endpoint = Endpoint::v2(MavLinkId::new(
        RemoteSystem::mavlink_system_id(this_system),
        1,
    ));

    // Blocking iterator
    for msg in channel {
        let res = mavlink_hal::transport::send_message(&msg, &endpoint, socket);
        if let Err(err) = res {
            tracing::error!(error = %err, "failed to send MAVLink message. Dropping it.");
        }
    }
    tracing::warn!("Exiting downlink_actor, should only happen at shutdown");
}

/// Encoding of `StateName` into/from Mavlink
impl StateName {
    /// Create a `StateName` from a Mavlink `char[30]`.
    ///
    /// Fails if the bytes are not valid utf8 until the first null byte or this substring doesn't denote a valid state name.
    /// Will not fail for a safely constructed `StateName` passed to [`StateName::to_array`].
    fn try_from_array(
        raw: &[u8; StateName::MAX_LEN],
        states: &HashSet<StateName>,
    ) -> Result<StateName> {
        // Find the end of the string. No state name contains null bytes.
        let len = raw
            .iter()
            .position(|&x| x == 0)
            .unwrap_or(StateName::MAX_LEN);
        let state = std::str::from_utf8(&raw[..len]).map_err(|_| {
            crate::Error::StateNameInvalidUTF8(String::from_utf8_lossy(raw).into_owned())
        })?;
        StateName::try_new(state, states)
    }

    /// Returns a Mavlink `char[30]` for this state name.
    ///
    /// Encodes the state as uft8 and leaves the remaining bytes 0.
    fn to_array(&self) -> [u8; StateName::MAX_LEN] {
        let mut char_array = [0u8; StateName::MAX_LEN]; // Initialize the array with null bytes
        let bytes = self.0.as_bytes(); // Convert the string to a byte slice

        // Copy the bytes into the array, ensuring it doesn't exceed the array size.
        // This should never cut off due to invariant.
        let len = bytes.len().min(StateName::MAX_LEN);
        char_array[..len].copy_from_slice(&bytes[..len]);

        char_array
    }
}

/// Encodes the current rich state as a message
pub fn encode_current_state((path_index, current_state): (u32, StateName)) -> StateMachineState {
    StateMachineState {
        path_index,
        state: current_state.to_array(),
    }
}

/// This function will return once the last received message has been longer than `timeout` ago.
pub async fn await_telemetry_loss(
    health: impl IntoWatched<UplinkTelemetryHealth>,
    clock: impl IntoWatched<MissionTime>,
    timeout: Duration,
) {
    use crate::reactive::WatchedExt as _;

    let mut clock = clock.watched();
    let health = health.watched();
    clock
        .next_satisfying(move |t| {
            *t >= MissionTime::from(Duration::from_micros(health.current().time_of_last_msg.0))
                + timeout
        })
        .await;
}

#[cfg(test)]
mod tests {
    use crate::Error;
    use crate::reactive::IntoWatched;
    use datatypes::units::InstantUs;
    use mavio::{Endpoint, MavLinkId};
    use messages::datapoints::dp_flight_computer::Message as FCMessage;
    use messages::datapoints::dp_rfs_computer::Message as RFSCMessage;

    use super::*;
    use std::collections::HashSet;
    use std::num::Wrapping;
    use std::str::FromStr;
    use std::time::Duration;
    use watchable::Watchable;

    #[derive(Debug, Clone, PartialEq)]
    enum ActuatorName {
        A,
        B,
        C,
        D,
    }

    impl FromStr for ActuatorName {
        type Err = ();

        fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
            match s {
                "A" => Ok(Self::A),
                "B" => Ok(Self::B),
                "C" => Ok(Self::C),
                "D" => Ok(Self::D),
                _ => Err(()),
            }
        }
    }

    // Helper function to create a set of valid state names
    fn create_state_names() -> HashSet<StateName> {
        let full_len = "A".repeat(StateName::MAX_LEN);
        let names = vec!["", "Idle", "Armed", "Main Disarmed", &full_len];
        names
            .into_iter()
            .map(|name| {
                StateName::new_unchecked(name).expect("all are short and without null bytes")
            })
            .collect()
    }

    /// Runs [`read_uplink_once`] with an optional message sent over UDP loopback.
    /// `this_system` is always `FlightComputer`; pass `Message::RfsComputer(...)` to simulate a message for another system.
    /// If `msg` is `None`, no message is sent and an IO error path is exercised.
    fn helper_read_uplink_once_with(
        msg: Option<&Message>,
    ) -> (
        transition::Command,
        Option<actuator_control::Command<ActuatorName>>,
        UplinkTelemetryHealth,
    ) {
        use std::net::UdpSocket;

        let receiver = UdpSocket::bind("127.0.0.1:0").expect("bind receiver");
        let receiver_addr = receiver.local_addr().expect("get receiver addr");

        match msg {
            Some(msg) => {
                let sender = UdpSocket::bind("127.0.0.1:0").expect("bind sender");
                sender.connect(receiver_addr).expect("connect sender");
                let endpoint = Endpoint::v2(MavLinkId::new(200, 1));
                mavlink_hal::transport::send_message(msg, &endpoint, &sender)
                    .expect("send message");
            }
            None => {
                receiver.set_nonblocking(true).expect("set nonblocking");
            }
        }

        let transition_command = Watchable::new(None);
        let actuator_command: Watchable<Option<actuator_control::Command<ActuatorName>>> =
            Watchable::new(None);
        let telemetry_health = Watchable::new(UplinkTelemetryHealth::default());
        let clock = Watchable::new(Duration::from_millis(100).into());
        let states = create_state_names();
        let this_system = RemoteSystem::FlightComputer;
        let mut seq = Wrapping(0u8);

        read_uplink_once(
            &receiver,
            &telemetry_health,
            &clock.watched(),
            &mut seq,
            |msg| {
                msg_processor(
                    &msg,
                    &transition_command,
                    &actuator_command,
                    &states,
                    this_system,
                )
            },
        );

        (
            transition_command.get(),
            actuator_command.get(),
            telemetry_health.get(),
        )
    }

    #[test]
    fn test_read_uplink_once_successful_message() {
        let (transition_command, _, health) = helper_read_uplink_once_with(Some(
            &Message::FlightComputer(FCMessage::ManualStateTransition(ManualStateTransition {
                transition_index: 0,
                state: *b"Idle\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
            })),
        ));

        assert_eq!(
            transition_command,
            Some((
                None,
                StateName::new_unchecked("Idle").expect("short enough")
            ))
        );
        assert_eq!(health.num_msg_received, 1);
        assert_eq!(health.num_msg_processed, 1);
        assert_eq!(health.time_of_last_msg, InstantUs(100_000));
        assert_eq!(health.time_of_last_processed_msg, InstantUs(100_000));
    }

    #[test]
    fn test_read_uplink_once_valid_state_with_index() {
        let (transition_command, _, health) = helper_read_uplink_once_with(Some(
            &Message::FlightComputer(FCMessage::ManualStateTransition(ManualStateTransition {
                transition_index: 1,
                state: *b"Idle\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
            })),
        ));

        assert_eq!(
            transition_command,
            Some((
                Some(1),
                StateName::new_unchecked("Idle").expect("short enough")
            ))
        );
        assert_eq!(health.num_msg_received, 1);
        assert_eq!(health.num_msg_processed, 1);
        assert_eq!(health.time_of_last_msg, InstantUs(100_000));
        assert_eq!(health.time_of_last_processed_msg, InstantUs(100_000));
    }

    #[test]
    fn test_read_uplink_once_error_in_reception() {
        // IO errors (no message available) do not update health — the uplink actor just retries.
        let (transition_command, _, health) = helper_read_uplink_once_with(None);

        assert_eq!(transition_command, None);
        assert_eq!(health.num_msg_received, 0);
        assert_eq!(health.num_msg_processed, 0);
        assert_eq!(health.time_of_last_msg, InstantUs(0));
        assert_eq!(health.time_of_last_processed_msg, InstantUs(0));
    }

    #[test]
    fn test_read_uplink_once_transition_for_other_system() {
        // A message addressed to the RFS system is ignored by the FC.
        let (transition_command, _, health) = helper_read_uplink_once_with(Some(
            &Message::RfsComputer(RFSCMessage::ManualStateTransition(ManualStateTransition {
                transition_index: 0,
                state: *b"Idle\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
            })),
        ));

        assert_eq!(transition_command, None);
        assert_eq!(health.num_msg_received, 1);
        assert_eq!(health.num_msg_processed, 0);
        assert_eq!(health.time_of_last_msg, InstantUs(100_000));
        assert_eq!(health.time_of_last_processed_msg, InstantUs(0));
    }

    #[test]
    fn test_read_uplink_once_unknown_state() {
        let (transition_command, _, health) = helper_read_uplink_once_with(Some(
            &Message::FlightComputer(FCMessage::ManualStateTransition(ManualStateTransition {
                transition_index: 0,
                state: *b"Unknown\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
            })),
        ));

        assert_eq!(transition_command, None);
        assert_eq!(health.num_msg_received, 1);
        assert_eq!(health.num_msg_processed, 0);
        assert_eq!(health.time_of_last_msg, InstantUs(100_000));
        assert_eq!(health.time_of_last_processed_msg, InstantUs(0));
    }

    /// Test encoding and decoding a valid state name.
    #[test]
    fn test_valid_state_name() {
        let states = create_state_names();
        let state_name = StateName::try_new("Armed", &states).expect("one of above");

        let encoded = state_name.to_array();
        let decoded = StateName::try_from_array(&encoded, &states).expect("roundtrip should work");

        assert_eq!(decoded, state_name);
    }

    /// Test encoding and decoding a state name that uses the full `MAX_LEN` bytes.
    #[test]
    fn test_max_length_state_name() {
        let states = create_state_names();
        let max_length_name = "A".repeat(StateName::MAX_LEN);
        let state_name = StateName::try_new(&max_length_name, &states).expect("one of above");

        let encoded = state_name.to_array();
        let decoded = StateName::try_from_array(&encoded, &states).expect("roundtrip should work");

        assert_eq!(decoded.0.as_ref(), max_length_name);
    }

    /// Test encoding and decoding a state name that has no bytes.
    #[test]
    fn test_empty_state_name() {
        let states = create_state_names();
        let state_name = StateName::try_new("", &states).expect("one of above");

        let encoded = state_name.to_array();
        let decoded = StateName::try_from_array(&encoded, &states).expect("roundtrip should work");

        assert_eq!(decoded.0.as_ref(), "");
    }

    /// Test decoding an array with invalid UTF-8 bytes.
    #[test]
    fn test_invalid_utf8() {
        let states = create_state_names();
        let invalid_utf8 = [
            0xC3, 0x28, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0,
        ]; // Invalid UTF-8 sequence

        let result = StateName::try_from_array(&invalid_utf8, &states);
        assert!(matches!(result, Err(Error::StateNameInvalidUTF8(_))));
    }

    /// Test decoding a valid UTF-8 string that is not a known state name.
    #[test]
    fn test_unknown_state_name() {
        let states = create_state_names();
        let unknown_name = StateName::new_unchecked("UNKNOWN").expect("is valid name");

        let encoded = unknown_name.to_array();
        let result = StateName::try_from_array(&encoded, &states);
        assert!(matches!(result, Err(Error::UnknownStateName(_))));
    }

    /// Test a valid actuator command
    #[test]
    fn test_read_uplink_once_valid_actuator_command() {
        let (_, actuator_command, health) = helper_read_uplink_once_with(Some(
            &Message::FlightComputer(FCMessage::ManualActuatorChange(ManualActuatorChange {
                value: 42.0,
                path_index: 1,
                actuator: *b"A\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
            })),
        ));

        assert_eq!(
            actuator_command,
            Some(actuator_control::Command {
                in_state_index: 1,
                actuator: ActuatorName::A,
                value: 42.0
            })
        );
        assert_eq!(health.num_msg_received, 1);
        assert_eq!(health.num_msg_processed, 1);
        assert_eq!(health.time_of_last_msg, InstantUs(100_000));
        assert_eq!(health.time_of_last_processed_msg, InstantUs(100_000));
    }

    /// Test ignoring a command for another system
    #[test]
    fn test_read_uplink_once_actuator_for_other_system() {
        let (_, actuator_command, health) = helper_read_uplink_once_with(Some(
            &Message::RfsComputer(RFSCMessage::ManualActuatorChange(ManualActuatorChange {
                value: 42.0,
                path_index: 1,
                actuator: *b"A\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
            })),
        ));

        assert_eq!(actuator_command, None);
        assert_eq!(health.num_msg_received, 1);
        assert_eq!(health.num_msg_processed, 0);
        assert_eq!(health.time_of_last_msg, InstantUs(100_000));
        assert_eq!(health.time_of_last_processed_msg, InstantUs(0));
    }

    /// Test ignoring a non-utf8 actuator name.
    #[test]
    fn test_read_uplink_once_invalid_actuator_id() {
        let (_, actuator_command, health) = helper_read_uplink_once_with(Some(
            &Message::FlightComputer(FCMessage::ManualActuatorChange(ManualActuatorChange {
                value: 42.0,
                path_index: 1,
                // Not utf-8
                actuator: [
                    0xC3, 0x28, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
            })),
        ));

        assert_eq!(actuator_command, None);
        assert_eq!(health.num_msg_received, 1);
        assert_eq!(health.num_msg_processed, 0);
        assert_eq!(health.time_of_last_msg, InstantUs(100_000));
        assert_eq!(health.time_of_last_processed_msg, InstantUs(0));
    }

    /// Test ignoring a non parseable actuator name
    #[test]
    fn test_read_uplink_once_invalid_actuator_name() {
        let (_, actuator_command, health) = helper_read_uplink_once_with(Some(
            &Message::FlightComputer(FCMessage::ManualActuatorChange(ManualActuatorChange {
                value: 42.0,
                path_index: 1,
                actuator: *b"E\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
            })),
        ));

        assert_eq!(actuator_command, None);
        assert_eq!(health.num_msg_received, 1);
        assert_eq!(health.num_msg_processed, 0);
        assert_eq!(health.time_of_last_msg, InstantUs(100_000));
        assert_eq!(health.time_of_last_processed_msg, InstantUs(0));
    }

    /// Test that the lost-message counter handles sequence-number wrap-around correctly.
    ///
    /// Scenario: we last saw sequence 249 (so `expected_seq = 250`).
    /// The next message arrives with sequence 0 (fresh endpoint), skipping over
    /// 250, 251, …, 255 — six sequences lost.
    /// `(0 - 250) mod 256 = 6` must be counted as lost.
    #[test]
    fn test_read_uplink_once_sequence_loss_with_wrap() {
        use std::net::UdpSocket;

        let receiver = UdpSocket::bind("127.0.0.1:0").expect("bind receiver");
        let receiver_addr = receiver.local_addr().expect("get receiver addr");
        let sender = UdpSocket::bind("127.0.0.1:0").expect("bind sender");
        sender.connect(receiver_addr).expect("connect sender");

        let endpoint = Endpoint::v2(MavLinkId::new(200, 1));
        let msg =
            Message::FlightComputer(FCMessage::ManualStateTransition(ManualStateTransition {
                transition_index: 0,
                state: *b"Idle\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",
            }));
        mavlink_hal::transport::send_message(&msg, &endpoint, &sender).expect("send message");

        let telemetry_health = Watchable::new(UplinkTelemetryHealth::default());
        let clock = Watchable::new(Duration::from_millis(0).into());
        let states = create_state_names();
        let transition_command = Watchable::new(None);
        let actuator_command: Watchable<Option<actuator_control::Command<ActuatorName>>> =
            Watchable::new(None);
        let this_system = RemoteSystem::FlightComputer;
        // Simulate having last received sequence 249, expecting 250 next.
        let mut seq = Wrapping(250u8);

        read_uplink_once(
            &receiver,
            &telemetry_health,
            &clock.watched(),
            &mut seq,
            |msg| {
                msg_processor(
                    &msg,
                    &transition_command,
                    &actuator_command,
                    &states,
                    this_system,
                )
            },
        );

        let health = telemetry_health.get();
        assert_eq!(health.num_msg_received, 1);
        assert_eq!(health.num_msg_lost, 6); // (0 - 250) mod 256 = 6
        assert_eq!(health.num_msg_processed, 1);
        // expected_seq should now be 1 (= 0 + 1)
        assert_eq!(seq, Wrapping(1u8));
    }
}
