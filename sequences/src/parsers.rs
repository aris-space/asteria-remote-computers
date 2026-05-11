use std::{collections::HashMap, time::Duration};

use chumsky::{prelude::*, span::Spanned};

pub type E<'a> = extra::Err<Rich<'a, char>>;

/// Macro for parsing function arguments.
/// Inside a [`Parser::validate`] function, it can be called like this:
///
/// ```rust,ignore
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
#[macro_export]
macro_rules! extract_args {
    ($args:ident, $emitter:ident, $e:ident => { $($rest:tt)* }) => {{
        extract_args!(@process $args, $emitter, $e, [] ; $($rest)*)
    }};
    (@process $args:expr, $emitter:expr, $e:expr, [$($acc:tt)*] ; ) => {{
        extract_args!(@build $args, $emitter, $e, [$($acc)*])
    }};
    (@process $args:expr, $emitter:expr, $e:expr, [$($acc:tt)*] ; $name:ident : $ty:ident $($rest:tt)*) => {
        extract_args!(@process $args, $emitter, $e, [$($acc)* ($name, /* optional */ false, $ty)]; $($rest)*)
    };
    (@process $args:expr, $emitter:expr, $e:expr, [$($acc:tt)*] ; $name:ident ?: $ty:ident $($rest:tt)*) => {
        extract_args!(@process $args, $emitter, $e, [$($acc)* ($name, /* optional */ true, $ty)]; $($rest)*)
    };
    (@extract $args:ident, $emitter:expr, $e:expr, $name:ident, true, $ty:ident) => {{
        match $args.remove(stringify!($name)) {
            Some(Spanned { inner: parsers::Arg::$ty(v), .. }) => Some(v),
            None => None,
            Some(Spanned { span, .. }) => {
                $emitter.emit(Rich::custom(span, format!("expected argument `{}` of type `{}`", stringify!($name), stringify!($ty))));
                None
            }
        }
    }};

    (@extract $args:ident, $emitter:expr, $e:expr, $name:ident, false, $ty:ident) => {{
        match $args.remove(stringify!($name)) {
            Some(Spanned { inner: parsers::Arg::$ty(v), .. }) => v,
            None => {
                $emitter.emit(Rich::custom($e.span(), format!("missing argument `{}` of type `{}`", stringify!($name), stringify!($ty))));
                Default::default()
            },
            Some(Spanned { span, .. }) => {
                $emitter.emit(Rich::custom(span, format!("expected argument `{}` of type `{}`", stringify!($name), stringify!($ty))));
                Default::default()
            }
        }
    }};
    (@build $args:expr, $emitter:expr, $e:expr, [
        $(($name:ident, $opt:tt, $ty:ident))*
    ]) => {{
        #[allow(unused_mut)]
        let mut args = $args;
        #[allow(unused)]
        let e = $e;
        $(let $name = extract_args!(@extract args, $emitter, e, $name, $opt, $ty);)*

        for (name, arg) in args.into_iter() {
            $emitter.emit(Rich::custom(arg.span, format!("unexpected argument `{name}`")))
        }

        #[allow(clippy::unused_unit)]
        ($($name),*)
    }};
}

pub enum Arg {
    Ident(String),
    String(String),
    Float(f32),
    Bool(bool),
    Time(Duration),
}

/// Parser for an argument.
/// It returns a tuple with an argument name and an [`Arg`] value.
pub fn arg<'a>() -> impl Parser<'a, &'a str, (String, Spanned<Arg>), E<'a>> {
    let int = text::int(10);
    let frac = just(".").then(text::digits(10));
    let float = int
        .then(frac.or_not())
        .to_slice()
        .from_str::<f32>()
        .unwrapped()
        .labelled("float");

    let time = float
        .then(
            just("us")
                .to(Duration::from_micros(1))
                .or(just("ms").to(Duration::from_millis(1)))
                .or(just("s").to(Duration::from_secs(1)))
                .or(just("m").to(Duration::from_mins(1))),
        )
        .map(|(n, duration)| Duration::from_secs_f32(duration.as_secs_f32() * n))
        .labelled("time");

    let bool = just("true")
        .to(true)
        .or(just("false").to(false))
        .labelled("bool");

    let string = any()
        .filter(|c: &char| *c != '\'')
        .repeated()
        .collect::<String>()
        .delimited_by(just("'"), just("'"));

    let ident = text::ident().map(|ident: &str| ident.to_string());

    text::ident()
        .then_ignore(just("=").padded())
        .then(choice((
            time.map(Arg::Time),
            float.map(Arg::Float),
            bool.map(Arg::Bool),
            string.map(Arg::String),
            ident.map(Arg::Ident),
        )))
        .map_with(|(name, arg): (&str, Arg), e| {
            (
                name.to_string(),
                Spanned {
                    inner: arg,
                    span: e.span(),
                },
            )
        })
}

/// Parser for a function with name `name`.
/// It returns a map of argument names and [`Arg`] values.
pub fn func<'a>(
    name: &'static str,
) -> impl Parser<'a, &'a str, HashMap<String, Spanned<Arg>>, E<'a>> {
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

/// Parser for a comment.
pub fn comment<'a>() -> impl Parser<'a, &'a str, (), E<'a>> {
    just('#')
        .then(any().filter(|c: &char| *c != '\n').repeated())
        .ignored()
}

/// Everything we don't consider relevant; e.g. comments.
pub fn junk<'a>() -> impl Parser<'a, &'a str, (), E<'a>> {
    choice((
        text::whitespace().at_least(1).ignored(),
        text::newline().ignored(),
        comment(),
    ))
    .repeated()
}
