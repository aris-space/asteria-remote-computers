use std::{collections::HashMap, time::Duration};

use chumsky::prelude::*;

pub type Span = std::ops::Range<usize>;
type E<'a> = extra::Err<Rich<'a, char>>;

#[derive(Clone, Debug)]
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
        name: String,
        abort: String,
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
        abort: String,
        patience: Duration,
        duration: Duration,
    },
}

pub fn sequence<'a>() -> impl Parser<'a, &'a str, Vec<Command>, E<'a>> {
    let command = actuator_open()
        .or(actuator_close())
        .or(sensor_watch())
        .or(sensor_ignore())
        .or(sensor_detect());

    command
        .padded()
        .separated_by(just(";"))
        .allow_trailing()
        .collect::<_>()
        .then_ignore(end())
}

/// Parser for identifiers. Allows any symbols except whitespaces and equal signs.
fn ident<'a>() -> impl Parser<'a, &'a str, String, E<'a>> {
    any()
        .filter(|c: &char| !c.is_whitespace() && *c != '=')
        .repeated()
        .at_least(1)
        .collect::<_>()
        .labelled("identifier")
}

/// Macro for parsing function arguments.
/// Inside a [`Parser::validate`] function, it can be called like this:
///
/// ```rust,unchecked
///func("my-function")
///    .validate(|args, e, emitter| {
///        // The arguments are returned in the order they are defined.
///        let (abort, max, min, key) = extract_args! {
///            args, emitter, e => {
///                // `abort` is a required argument of type string.
///                abort: String
///                // `max` is a required argument of type float.
///                max: Float
///                // `key` is an optional argument of type string; if it doesn't exist
///                // it's value is just `None`.
///                key?: String
///            }
///        };
///
///        Command::SensorWatch { name, abort, max, min, key }
///    })
/// ```
///
/// The macro will generate a parser error when there are unexpected or missing arguments.
macro_rules! extract_args {
    ($args:ident, $emitter:ident, $e:ident => { $($rest:tt)* }) => {{
        extract_args!(@process $args, $emitter, $e, [] ; $($rest)*)
    }};
    (@process $args:expr, $emitter:expr, $e:expr, [$($acc:tt)*] ; ) => {{
        extract_args!(@build $args, $emitter, $e, [$($acc)*])
    }};
    (@process $args:expr, $emitter:expr, $e:expr, [$($acc:tt)*] ;
        $name:ident : $ty:ident
        $($rest:tt)*
    ) => {
        extract_args!(
            @process
            $args,
            $emitter,
            $e,
            [$($acc)* ($name, /* optional */ false, $ty)];
            $($rest)*
        )
    };
    (@process $args:expr, $emitter:expr, $e:expr, [$($acc:tt)*] ;
        $name:ident ?: $ty:ident
        $($rest:tt)*
    ) => {
        extract_args!(
            @process
            $args,
            $emitter,
            $e,
            [$($acc)* ($name, /* optional */ true, $ty)];
            $($rest)*
        )
    };
    (@extract $args:ident, $emitter:expr, $e:expr, $name:ident, true, $ty:ident) => {{
        match $args.remove(stringify!($name)) {
            Some(Arg::$ty(v)) => Some(v),
            None => None,
            _ => {
                $emitter.emit(Rich::custom($e.span(), format!("expected argument `{}` of type `{}`", stringify!($name), stringify!($variant))));
                None
            }
        }
    }};

    (@extract $args:ident, $emitter:expr, $e:expr, $name:ident, false, $ty:ident) => {{
        match $args.remove(stringify!($name)) {
            Some(Arg::$ty(v)) => v,
            _ => {
                $emitter.emit(Rich::custom($e.span(), format!("expected argument `{}` of type `{}`", stringify!($name), stringify!($variant))));
                Default::default()
            }
        }
    }};
    (@build $args:expr, $emitter:expr, $e:expr, [
        $(
            ($name:ident, $opt:tt, $ty:ident)
        )*
    ]) => {{
        #[allow(unused_mut)]
        let mut args = $args;
        $(
            let $name = extract_args!(@extract args, $emitter, $e, $name, $opt, $ty);
        )*

        if !args.is_empty() {
            $emitter.emit(Rich::custom($e.span(), format!("unexpected arguments: {}", args.into_keys().map(|str| format!("`{str}`")).collect::<Vec<_>>().join(", "))))
        }

        #[allow(clippy::unused_unit)]
        ($($name),*)
    }};
}

enum Arg {
    String(String),
    Float(f32),
    Bool(bool),
    Time(Duration),
}

/// Parser for an argument.
/// It returns a tuple with an argument name and an [`Arg`] value.
fn arg<'a>() -> impl Parser<'a, &'a str, (String, Arg), E<'a>> {
    let int = text::int(10);
    let frac = just(".").then(text::digits(10));
    let float = int
        .then(frac.or_not())
        .to_slice()
        .from_str::<f32>()
        .unwrapped()
        .labelled("float");

    #[expect(clippy::cast_sign_loss)]
    #[expect(clippy::cast_possible_truncation)]
    let time = float
        .then(
            just("us")
                .to(Duration::from_micros(1))
                .or(just("ms").to(Duration::from_millis(1)))
                .or(just("s").to(Duration::from_secs(1)))
                .or(just("m").to(Duration::from_mins(1))),
        )
        .map(|(n, duration)| duration * n as u32)
        .labelled("time");

    let bool = just("true")
        .to(true)
        .or(just("false").to(false))
        .labelled("bool");

    ident().then_ignore(just("=").padded()).then(choice((
        float.map(Arg::Float),
        time.map(Arg::Time),
        bool.map(Arg::Bool),
        any().repeated().collect::<_>().map(Arg::String),
    )))
}

/// Parser for a function with name `name`.
/// It returns a map of argument names and [`Arg`] values.
fn func<'a>(name: &'static str) -> impl Parser<'a, &'a str, HashMap<String, Arg>, E<'a>> {
    just(name)
        .then(
            arg()
                .padded()
                .separated_by(just(","))
                .allow_trailing()
                .collect::<HashMap<_, _>>()
                .delimited_by(just("("), just(")")),
        )
        .map(|(_, args)| args)
}

fn actuator_open<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    ident()
        .then_ignore(just("."))
        .then(func("open"))
        .validate(|(name, args), e, emitter| {
            // Ensure that we don't have any arguments.
            extract_args! { args, emitter, e => {} };
            // TODO: Validate that the actuator exists.

            Command::AcutatorOpen { name }
        })
}

fn actuator_close<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    ident()
        .then_ignore(just("."))
        .then(func("close"))
        .validate(|(name, args), e, emitter| {
            // Ensure that we don't have any arguments.
            extract_args! { args, emitter, e => {} };
            // TODO: Validate that the actuator exists.

            Command::ActuatorClose { name }
        })
}

fn sensor_watch<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    ident()
        .then_ignore(just("."))
        .then(func("watch"))
        .validate(|(name, args), e, emitter| {
            let (abort, max, min, key) = extract_args! {
                args, emitter, e => {
                    abort: String
                    max: Float
                    min: Float
                    key?: String
                }
            };
            // TODO: Validate that the sensor exists.

            Command::SensorWatch {
                name,
                abort,
                max,
                min,
                key,
            }
        })
}

fn sensor_ignore<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    ident()
        .then_ignore(just("."))
        .then(func("ignore"))
        .validate(|(name, args), e, emitter| {
            let key = extract_args! {
                args, emitter, e => { key?: String }
            };
            // TODO: Validate that the sensor exists.

            Command::SensorIgnore { name, key }
        })
}

fn sensor_detect<'a>() -> impl Parser<'a, &'a str, Command, E<'a>> {
    ident()
        .then_ignore(just("."))
        .then(func("watch"))
        .validate(|(name, args), e, emitter| {
            let (value, rising, abort, patience, duration) = extract_args! {
                args, emitter, e => {
                    value: Float
                    rising: Bool
                    abort: String
                    patience: Time
                    duration: Time
                }
            };
            // TODO: Validate that the sensor exists.

            Command::SensorDetect {
                name,
                value,
                rising,
                abort,
                patience,
                duration,
            }
        })
}
