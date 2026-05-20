use std::{fmt::Display, str::FromStr, time::Duration};

use chumsky::prelude::*;

use crate::parsers::{E, func, ident, junk};

pub mod parsers;

#[derive(Clone, Debug)]
pub enum Command<Actuator, Sensor, State>
where
    Actuator: Clone + FromStr,
    Sensor: Clone + FromStr,
    State: Clone + FromStr,
{
    // ----- System functions -----
    Msg(String),
    Warning(String),
    Error(String),
    Wait(Duration),
    ToState(State),
    // ----- Actuator functions -----
    ActuatorOpen(Actuator),
    ActuatorClose(Actuator),
    DprActivate {
        name: Actuator,
        setpoint: f32,
    },
    DprDeactivate(Actuator),
    // ----- Sensor functions -----
    SensorWatch {
        name: Sensor,
        abort_state: State,
        max: f32,
        min: f32,
        key: Option<String>,
    },
    SensorIgnore {
        name: Sensor,
        key: Option<String>,
    },
    SensorDetect {
        name: Sensor,
        value: f32,
        rising: bool,
        abort_state: State,
        patience: Duration,
        duration: Option<Duration>,
    },
}

type ParseResult<'a, Actuator, Sensor, State> =
    Result<Vec<Command<Actuator, Sensor, State>>, Vec<Rich<'a, char>>>;

/// Try to parse a sequence string into a vector of sequence [`Command`]s.
/// Should the parsing fail, a list with parser errors is returned in [`Err`].
pub fn parse_sequence_str<Actuator, Sensor, State>(
    sequence_str: &str,
) -> ParseResult<'_, Actuator, Sensor, State>
where
    Actuator: Clone + FromStr,
    <Actuator as FromStr>::Err: Display,
    Sensor: Clone + FromStr,
    <Sensor as FromStr>::Err: Display,
    State: Clone + FromStr,
    <State as FromStr>::Err: Display,
{
    let (commands, errors) = sequence::<Actuator, Sensor, State>()
        .parse(sequence_str)
        .into_output_errors();

    if errors.is_empty() {
        Ok(commands.unwrap_or_default())
    } else {
        Err(errors)
    }
}

fn sequence<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Vec<Command<Actuator, Sensor, State>>, E<'a>>
where
    Actuator: Clone + FromStr,
    <Actuator as FromStr>::Err: Display,
    Sensor: Clone + FromStr,
    <Sensor as FromStr>::Err: Display,
    State: Clone + FromStr,
    <State as FromStr>::Err: Display,
{
    let command = msg()
        .map(Some)
        .or(warning().map(Some))
        .or(error().map(Some))
        .or(wait().map(Some))
        .or(to_state())
        .or(actuator_open())
        .or(actuator_close())
        .or(dpr_activate())
        .or(dpr_deactivate())
        .or(sensor_watch())
        .or(sensor_ignore())
        .or(sensor_detect());

    command
        .padded_by(junk())
        .separated_by(just(";").padded_by(junk()))
        .allow_trailing()
        .collect::<_>()
        // Filter out `None`s which are invalid items.
        .map(|commands: Vec<Option<Command<Actuator, Sensor, State>>>| {
            commands.into_iter().flatten().collect()
        })
        .then_ignore(junk())
        .then_ignore(end())
}

/// Parser for the `sys.msg(text = "...")` command.
fn msg<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Command<Actuator, Sensor, State>, E<'a>>
where
    Actuator: Clone + FromStr,
    Sensor: Clone + FromStr,
    State: Clone + FromStr,
{
    func("sys.msg")
        .or(func("SYS.msg"))
        .validate(|args, e, emitter| {
            let text = extract_args! {
                args, emitter, e => { text: String }
            };

            Command::Msg(text.inner)
        })
}

/// Parser for the `sys.warning(text = "...")` command.
fn warning<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Command<Actuator, Sensor, State>, E<'a>>
where
    Actuator: Clone + FromStr,
    Sensor: Clone + FromStr,
    State: Clone + FromStr,
{
    func("sys.warning")
        .or(func("SYS.warning"))
        .validate(|args, e, emitter| {
            let text = extract_args! {
                args, emitter, e => { text: String }
            };

            Command::Warning(text.inner)
        })
}

/// Parser for the `sys.error(text = "...")` command.
fn error<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Command<Actuator, Sensor, State>, E<'a>>
where
    Actuator: Clone + FromStr,
    Sensor: Clone + FromStr,
    State: Clone + FromStr,
{
    func("sys.error")
        .or(func("SYS.error"))
        .validate(|args, e, emitter| {
            let text = extract_args! {
                args, emitter, e => { text: String }
            };

            Command::Warning(text.inner)
        })
}

/// Parser for the `sys.wait(time = <time>)` command.
fn wait<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Command<Actuator, Sensor, State>, E<'a>>
where
    Actuator: Clone + FromStr,
    Sensor: Clone + FromStr,
    State: Clone + FromStr,
{
    func("sys.wait")
        .or(func("SYS.wait"))
        .validate(|args, e, emitter| {
            let time = extract_args! {
                args, emitter, e => { time: Time }
            };

            Command::Wait(time.inner)
        })
}

/// Parser for the `sys.to_state(state = <state_name>)` command.
fn to_state<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Option<Command<Actuator, Sensor, State>>, E<'a>>
where
    Actuator: Clone + FromStr,
    Sensor: Clone + FromStr,
    State: Clone + FromStr,
    <State as FromStr>::Err: Display,
{
    func("sys.to_state")
        .or(func("SYS.to_state"))
        .validate(|args, e, emitter| {
            let state = extract_args! {
                args, emitter, e => { state: Ident as State }
            };

            Some(Command::ToState(state?.inner))
        })
}

/// Parser for the `<actuator_name>.open()` command.
fn actuator_open<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Option<Command<Actuator, Sensor, State>>, E<'a>>
where
    Actuator: Clone + FromStr,
    <Actuator as FromStr>::Err: Display,
    Sensor: Clone + FromStr,
    State: Clone + FromStr,
{
    ident::<Actuator>()
        .then_ignore(just("."))
        .then(func("open"))
        .validate(|(actuator, args), _e, emitter| {
            // Ensure that we don't have any arguments.
            extract_args! { args, emitter, e => {} };

            let actuator = actuator?;
            if !actuator.1.to_lowercase().contains("dpr") {
                return Some(Command::ActuatorOpen(actuator.inner.0));
            }

            emitter.emit(Rich::custom(
                actuator.span,
                "`open` can only be used to open non-DPR actuators",
            ));

            None
        })
}

/// Parser for the `<actuator_name>.close()` command.
fn actuator_close<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Option<Command<Actuator, Sensor, State>>, E<'a>>
where
    Actuator: Clone + FromStr,
    <Actuator as FromStr>::Err: Display,
    Sensor: Clone + FromStr,
    State: Clone + FromStr,
{
    ident::<Actuator>()
        .then_ignore(just("."))
        .then(func("close"))
        .validate(|(actuator, args), _e, emitter| {
            // Ensure that we don't have any arguments.
            extract_args! { args, emitter, e => {} };

            let actuator = actuator?;
            if !actuator.1.to_lowercase().contains("dpr") {
                return Some(Command::ActuatorClose(actuator.inner.0));
            }

            emitter.emit(Rich::custom(
                actuator.span,
                "`close` can only be used to close non-DPR actuators",
            ));

            None
        })
}

/// Parser for the `<actuator_name>.activate(setpoint = <float>)` command.
fn dpr_activate<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Option<Command<Actuator, Sensor, State>>, E<'a>>
where
    Actuator: Clone + FromStr,
    <Actuator as FromStr>::Err: Display,
    Sensor: Clone + FromStr,
    State: Clone + FromStr,
{
    ident::<Actuator>()
        .then_ignore(just("."))
        .then(func("activate"))
        .validate(|(actuator, args), e, emitter| {
            let setpoint = extract_args! { args, emitter, e => {
                setpoint: Float
            } };

            let actuator = actuator?;
            if actuator.1.to_lowercase().contains("dpr") {
                return Some(Command::DprActivate {
                    name: actuator.inner.0,
                    setpoint: setpoint.inner,
                });
            }

            emitter.emit(Rich::custom(
                actuator.span,
                "`activate` can only be used to activate DPR actuators",
            ));

            None
        })
}

/// Parser for the `<actuator_name>.deactivate()` command.
fn dpr_deactivate<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Option<Command<Actuator, Sensor, State>>, E<'a>>
where
    Actuator: Clone + FromStr,
    <Actuator as FromStr>::Err: Display,
    Sensor: Clone + FromStr,
    State: Clone + FromStr,
{
    ident::<Actuator>()
        .then_ignore(just("."))
        .then(func("deactivate"))
        .validate(|(actuator, args), _e, emitter| {
            // Ensure that we don't have any arguments.
            extract_args! { args, emitter, e => {} };

            let actuator = actuator?;
            if actuator.1.to_lowercase().contains("dpr") {
                return Some(Command::DprDeactivate(actuator.inner.0));
            }

            emitter.emit(Rich::custom(
                actuator.span,
                "`deactivate` can only be used to deactivate DPR actuators",
            ));

            None
        })
}

/// Parser for the `<sensor_name>.watch(abort = <state_name>, max = <optional float>, min = <optional float>, key = <optional identifier>)` command.
fn sensor_watch<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Option<Command<Actuator, Sensor, State>>, E<'a>>
where
    Actuator: Clone + FromStr,
    Sensor: Clone + FromStr,
    <Sensor as FromStr>::Err: Display,
    State: Clone + FromStr,
    <State as FromStr>::Err: Display,
{
    ident::<Sensor>()
        .then_ignore(just("."))
        .then(func("watch"))
        .validate(|(sensor, args), e, emitter| {
            let (abort, max, min, key) = extract_args! {
                args, emitter, e => {
                    abort: Ident as State
                    max?: Float
                    min?: Float
                    key?: Ident
                }
            };

            Some(Command::SensorWatch {
                name: sensor?.inner.0,
                abort_state: abort?.inner,
                max: max.inner.unwrap_or(f32::INFINITY),
                min: min.inner.unwrap_or(f32::INFINITY),
                key: key.inner,
            })
        })
}

/// Parser for the `<sensor_name>.ignore(key = <optional identifier>)` command.
fn sensor_ignore<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Option<Command<Actuator, Sensor, State>>, E<'a>>
where
    Actuator: Clone + FromStr,
    Sensor: Clone + FromStr,
    <Sensor as FromStr>::Err: Display,
    State: Clone + FromStr,
{
    ident::<Sensor>()
        .then_ignore(just("."))
        .then(func("ignore"))
        .validate(|(sensor, args), e, emitter| {
            let key = extract_args! {
                args, emitter, e => { key?: Ident }
            };

            Some(Command::SensorIgnore {
                name: sensor?.inner.0,
                key: key.inner,
            })
        })
}

/// Parser for the `<sensor_name>.detect(value = <float>, rising = <optional bool>, abort = <state_name>, patience = <time>, duration = <optional time>)` command.
fn sensor_detect<'a, Actuator, Sensor, State>()
-> impl Parser<'a, &'a str, Option<Command<Actuator, Sensor, State>>, E<'a>>
where
    Actuator: Clone + FromStr,
    Sensor: Clone + FromStr,
    <Sensor as FromStr>::Err: Display,
    State: Clone + FromStr,
    <State as FromStr>::Err: Display,
{
    ident::<Sensor>()
        .then_ignore(just("."))
        .then(func("detect"))
        .validate(|(sensor, args), e, emitter| {
            let (value, rising, abort, patience, duration) = extract_args! {
                args, emitter, e => {
                    value: Float
                    rising?: Bool
                    abort: Ident as State
                    patience: Time
                    duration?: Time
                }
            };

            Some(Command::SensorDetect {
                name: sensor?.inner.0,
                value: value.inner,
                rising: rising.inner.unwrap_or(true),
                abort_state: abort?.inner,
                patience: patience.inner,
                duration: duration.inner,
            })
        })
}
