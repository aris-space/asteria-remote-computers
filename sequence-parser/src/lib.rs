use std::time::Duration;

use chumsky::{input::Emitter, prelude::*};

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
            validate_actuator(&name, actuators, emitter);

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
            validate_actuator(&name, actuators, emitter);

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
            validate_sensor(&name, sensors, emitter);
            // Validate that the abort state exists.
            validate_state(&abort, states, emitter);

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
            validate_sensor(&name, sensors, emitter);

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
            validate_sensor(&name, sensors, emitter);
            // Validate that the abort state exists.
            validate_state(&abort, states, emitter);

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

/// Validate that `actuator_name` is a valid actuator of `actuators` and
/// otherwise emit an error into `emitter`.
fn validate_actuator(
    actuator_name: &Spanned<&str>,
    actuators: &[String],
    emitter: &mut Emitter<Rich<'_, char>>,
) {
    if !actuators.contains(&actuator_name.to_string()) {
        emitter.emit(Rich::custom(
            actuator_name.span,
            format!(
                "`{}` is not a valid actuator. Expected one of: {}",
                **actuator_name,
                actuators
                    .iter()
                    .map(|name| format!("`{name}`"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        ));
    }
}

/// Validate that `sensor_name` is a valid sensor of `sensors` and otherwise
/// emit an error into `emitter`.
fn validate_sensor(
    sensor_name: &Spanned<&str>,
    sensors: &[String],
    emitter: &mut Emitter<Rich<'_, char>>,
) {
    if !sensors.contains(&sensor_name.to_string()) {
        emitter.emit(Rich::custom(
            sensor_name.span,
            format!(
                "`{}` is not a valid sensor. Expected one of: {}",
                **sensor_name,
                sensors
                    .iter()
                    .map(|name| format!("`{name}`"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        ));
    }
}

/// Validate that `state_name` is a valid state in `states` and otherwise
/// emit an error into `emitter`.
fn validate_state(
    state_name: &Spanned<String>,
    states: &[String],
    emitter: &mut Emitter<Rich<'_, char>>,
) {
    if !states.contains(state_name) {
        emitter.emit(Rich::custom(
            state_name.span,
            format!(
                "`{}` is not a valid state. Expected one of: {}",
                **state_name,
                states
                    .iter()
                    .map(|name| format!("`{name}`"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        ));
    }
}
