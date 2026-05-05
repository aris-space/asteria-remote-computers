use std::time::Duration;

use chumsky::prelude::*;

#[derive(Clone)]
pub enum Command {
    // ----- Actuator functions -----
    AcutatorOpen {
        name: String,
    },
    ActuatorClose {
        name: String,
    },
    // ----- Sensor functions -----
    SensorWatch {
        abort: String,
        max: f32,
        min: f32,
        key: Option<String>,
    },
    SensorIgnore {
        key: Option<String>,
    },
    SensorDetect {
        value: f32,
        rising: bool,
        abort: String,
        patience: Duration,
        duration: Duration,
    },
}

pub fn sequence<'a>() -> impl Parser<'a, &'a str, Vec<Command>> {
    let command = actuator_open().or(actuator_close());

    command
        .separated_by(just(";"))
        .allow_trailing()
        .collect::<_>()
        .then_ignore(end())
}

/// Parser for actuator names. Allows any symbols except whitespaces.
fn actuator_name<'a>() -> impl Parser<'a, &'a str, String> {
    any()
        .filter(|c: &char| !c.is_whitespace())
        .repeated()
        .at_least(1)
        .collect::<_>()
}

fn actuator_open<'a>() -> impl Parser<'a, &'a str, Command> {
    actuator_name()
        .then_ignore(just("."))
        .then_ignore(just("open"))
        .then_ignore(just("("))
        .then_ignore(just(")"))
        .map(|name| Command::AcutatorOpen { name })
}

fn actuator_close<'a>() -> impl Parser<'a, &'a str, Command> {
    actuator_name()
        .then_ignore(just("."))
        .then_ignore(just("close"))
        .then_ignore(just("("))
        .then_ignore(just(")"))
        .map(|name| Command::ActuatorClose { name })
}
