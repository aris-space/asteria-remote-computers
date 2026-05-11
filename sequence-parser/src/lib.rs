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

pub fn sequence<'a>(
    actuators: &[String],
    sensors: &[String],
    states: &[String],
) -> impl Parser<'a, &'a str, Vec<Command>, E<'a>> {
    let command = msg()
        .or(warning())
        .or(error())
        .or(wait())
        .or(to_state())
        .or(actuator_open(actuators))
        .or(actuator_close(actuators))
        .or(sensor_watch(sensors, states))
        .or(sensor_ignore(sensors))
        .or(sensor_detect(sensors, states));

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

        Command::Msg(text.to_string())
    })
}

fn warning<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    func("sys.warning").validate(|args, e, emitter| {
        let text = extract_args! {
            args, emitter, e => { text: String }
        };

        Command::Warning(text.to_string())
    })
}

fn error<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    func("sys.error").validate(|args, e, emitter| {
        let text = extract_args! {
            args, emitter, e => { text: String }
        };

        Command::Warning(text.to_string())
    })
}

fn wait<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    func("sys.wait").validate(|args, e, emitter| {
        let time = extract_args! {
            args, emitter, e => { time: Time }
        };

        Command::Wait(*time)
    })
}

fn to_state<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    func("sys.to_state").validate(|args, e, emitter| {
        let state = extract_args! {
            args, emitter, e => { state: Ident }
        };

        Command::ToState(state.to_string())
    })
}

fn actuator_open<'a>(actuators: &[String]) -> impl Parser<'a, &'a str, Command, E<'a>> {
    text::ident()
        .spanned()
        .then_ignore(just("."))
        .then(func("open"))
        .validate(|(name, args), e, emitter| {
            // Ensure that we don't have any arguments.
            extract_args! { args, emitter, e => {} };
            // Validate that the actuator exists.
            if !actuators.contains(&name.to_string()) {
                emitter.emit(Rich::custom(
                    name.span,
                    format!(
                        "`{}` is not a valid actuator name. Expected one of: {}",
                        *name,
                        actuators
                            .iter()
                            .map(|name| format!("`{name}`"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                ));
            }

            Command::AcutatorOpen(name.to_string())
        })
}

fn actuator_close<'a>(actuators: &[String]) -> impl Parser<'a, &'a str, Command, E<'a>> {
    text::ident()
        .spanned()
        .then_ignore(just("."))
        .then(func("close"))
        .validate(|(name, args), e, emitter| {
            // Ensure that we don't have any arguments.
            extract_args! { args, emitter, e => {} };
            // Validate that the actuator exists.
            if !actuators.contains(&name.to_string()) {
                emitter.emit(Rich::custom(
                    name.span,
                    format!(
                        "`{}` is not a valid actuator name. Expected one of: {}",
                        *name,
                        actuators
                            .iter()
                            .map(|name| format!("`{name}`"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                ));
            }

            Command::ActuatorClose(name.to_string())
        })
}

fn sensor_watch<'a>(
    sensors: &[String],
    states: &[String],
) -> impl Parser<'a, &'a str, Command, E<'a>> {
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
            // Validate that the sensor exists.
            if !sensors.contains(&name.to_string()) {
                emitter.emit(Rich::custom(
                    name.span,
                    format!(
                        "`{}` is not a valid sensor name. Expected one of: {}",
                        *name,
                        sensors
                            .iter()
                            .map(|name| format!("`{name}`"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                ));
            }
            // Validate that the abort state exists.
            if !states.contains(&abort) {
                emitter.emit(Rich::custom(
                    abort.span,
                    format!(
                        "`{}` is not a valid state. Expected one of: {}",
                        *abort,
                        states
                            .iter()
                            .map(|name| format!("`{name}`"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                ));
            }

            Command::SensorWatch {
                name: name.to_string(),
                abort_state: abort.to_string(),
                max: max.map_or(f32::INFINITY, |max| *max),
                min: min.map_or(f32::INFINITY, |min| *min),
                key: key.map(|key| key.to_string()),
            }
        })
}

fn sensor_ignore<'a>(sensors: &[String]) -> impl Parser<'a, &'a str, Command, E<'a>> {
    text::ident()
        .spanned()
        .then_ignore(just("."))
        .then(func("ignore"))
        .validate(|(name, args), e, emitter| {
            let key = extract_args! {
                args, emitter, e => { key?: Ident }
            };
            // Validate that the sensor exists.
            if !sensors.contains(&name.to_string()) {
                emitter.emit(Rich::custom(
                    name.span,
                    format!(
                        "`{}` is not a valid sensor name. Expected one of: {}",
                        *name,
                        sensors
                            .iter()
                            .map(|name| format!("`{name}`"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                ));
            }

            Command::SensorIgnore {
                name: name.to_string(),
                key: key.map(|key| key.to_string()),
            }
        })
}

fn sensor_detect<'a>(
    sensors: &[String],
    states: &[String],
) -> impl Parser<'a, &'a str, Command, E<'a>> {
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
            // Validate that the sensor exists.
            if !sensors.contains(&name.to_string()) {
                emitter.emit(Rich::custom(
                    name.span,
                    format!(
                        "`{}` is not a valid sensor name. Expected one of: {}",
                        *name,
                        sensors
                            .iter()
                            .map(|name| format!("`{name}`"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                ));
            }
            // Validate that the abort state exists.
            if !states.contains(&abort) {
                emitter.emit(Rich::custom(
                    abort.span,
                    format!(
                        "`{}` is not a valid state. Expected one of: {}",
                        *abort,
                        states
                            .iter()
                            .map(|name| format!("`{name}`"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                ));
            }

            Command::SensorDetect {
                name: name.to_string(),
                value: *value,
                rising: rising.is_some_and(|rising| *rising),
                abort_state: abort.to_string(),
                patience: *patience,
                duration: duration.map(|duration| *duration),
            }
        })
}
