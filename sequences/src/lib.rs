use std::time::Duration;

use chumsky::prelude::*;

use crate::parsers::{E, func, junk};

pub mod parsers;

#[derive(Clone, Debug)]
pub enum Command {
    // ----- System functions -----
    Msg(String),
    Warning(String),
    Error(String),
    Wait(Duration),
    ToState(String),
    // ----- Actuator functions -----
    AcutatorOpen(String),
    ActuatorClose(String),
    // ----- Sensor functions -----
    SensorWatch {
        name: String,
        abort_state: String,
        max: f32,
        min: f32,
        key: Option<String>,
    },
    SensorIgnore {
        name: String,
        key: Option<String>,
    },
    SensorDetect {
        name: String,
        value: f32,
        rising: bool,
        abort_state: String,
        patience: Duration,
        duration: Option<Duration>,
    },
}

pub fn sequence<'a>() -> impl Parser<'a, &'a str, Vec<Command>, E<'a>> {
    let command = msg()
        .or(warning())
        .or(error())
        .or(wait())
        .or(to_state())
        .or(actuator_open())
        .or(actuator_close())
        .or(sensor_watch())
        .or(sensor_ignore())
        .or(sensor_detect());

    command
        .padded_by(junk())
        .separated_by(just(";").padded_by(junk()))
        .allow_trailing()
        .collect::<_>()
        .then_ignore(junk())
        .then_ignore(end())
}

fn msg<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    func("sys.msg").validate(|args, e, emitter| {
        let text = extract_args! {
            args, emitter, e => { text: String }
        };

        Command::Msg(text)
    })
}

fn warning<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    func("sys.warning").validate(|args, e, emitter| {
        let text = extract_args! {
            args, emitter, e => { text: String }
        };

        Command::Warning(text)
    })
}

fn error<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    func("sys.error").validate(|args, e, emitter| {
        let text = extract_args! {
            args, emitter, e => { text: String }
        };

        Command::Warning(text)
    })
}

fn wait<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    func("sys.wait").validate(|args, e, emitter| {
        let time = extract_args! {
            args, emitter, e => { time: Time }
        };

        Command::Wait(time)
    })
}

fn to_state<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    func("sys.to_state").validate(|args, e, emitter| {
        let state = extract_args! {
            args, emitter, e => { state: Ident }
        };

        Command::ToState(state)
    })
}

fn actuator_open<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    text::ident()
        .spanned()
        .then_ignore(just("."))
        .then(func("open"))
        .validate(|(name, args), e, emitter| {
            // Ensure that we don't have any arguments.
            extract_args! { args, emitter, e => {} };
            // TODO: Validate that the actuator exists.

            Command::AcutatorOpen(name.to_string())
        })
}

fn actuator_close<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    text::ident()
        .spanned()
        .then_ignore(just("."))
        .then(func("close"))
        .validate(|(name, args), e, emitter| {
            // Ensure that we don't have any arguments.
            extract_args! { args, emitter, e => {} };
            // TODO: Validate that the actuator exists.

            Command::ActuatorClose(name.to_string())
        })
}

fn sensor_watch<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    text::ident()
        .spanned()
        .then_ignore(just("."))
        .then(func("watch"))
        .validate(|(name, args), e, emitter| {
            let (abort, max, min, key) = extract_args! {
                args, emitter, e => {
                    abort: Ident
                    max?: Float
                    min?: Float
                    key?: Ident
                }
            };
            // TODO: Validate that the sensor exists.

            Command::SensorWatch {
                name: name.to_string(),
                abort_state: abort,
                max: max.unwrap_or(f32::INFINITY),
                min: min.unwrap_or(f32::INFINITY),
                key,
            }
        })
}

fn sensor_ignore<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    text::ident()
        .spanned()
        .then_ignore(just("."))
        .then(func("ignore"))
        .validate(|(name, args), e, emitter| {
            let key = extract_args! {
                args, emitter, e => { key?: Ident }
            };
            // TODO: Validate that the sensor exists.

            Command::SensorIgnore {
                name: name.to_string(),
                key,
            }
        })
}

fn sensor_detect<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    text::ident()
        .spanned()
        .then_ignore(just("."))
        .then(func("detect"))
        .validate(|(name, args), e, emitter| {
            let (value, rising, abort, patience, duration) = extract_args! {
                args, emitter, e => {
                    value: Float
                    rising?: Bool
                    abort: Ident
                    patience: Time
                    duration?: Time
                }
            };
            // TODO: Validate that the sensor exists.

            Command::SensorDetect {
                name: name.to_string(),
                value,
                rising: rising.unwrap_or_default(),
                abort_state: abort,
                patience,
                duration,
            }
        })
}
