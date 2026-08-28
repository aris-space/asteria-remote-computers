use std::{collections::HashMap, fmt::Display, str::FromStr, time::Duration};

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
///                // It will be mapped into an `Actuator` which implements `std::str::FromStr`.
///                // Mapping to other types is only possible for strings.
///                abort: String as Actuator
///                // `max` is a required argument of type float.
///                max: Float
///                // `key` is an optional argument of type ident; if it doesn't exist
///                // it's value is just `None`.
///                key?: Ident
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
    /* required `String` with cast */
    (@process $args:expr, $emitter:expr, $e:expr, [$($acc:tt)*] ; $name:ident : String as $cast:ident $($rest:tt)*) => {
        extract_args!(@process $args, $emitter, $e, [$($acc)* ($name, /* optional */ false, String, as $cast)]; $($rest)*)
    };
    /* optional `String` with cast */
    (@process $args:expr, $emitter:expr, $e:expr, [$($acc:tt)*] ; $name:ident ?: String as $cast:ident $($rest:tt)*) => {
        extract_args!(@process $args, $emitter, $e, [$($acc)* ($name, /* optional */ true, String, as $cast)]; $($rest)*)
    };
    (@process $args:expr, $emitter:expr, $e:expr, [$($acc:tt)*] ; $name:ident : $ty:ident $($rest:tt)*) => {
        extract_args!(@process $args, $emitter, $e, [$($acc)* ($name, /* optional */ false, $ty, /* cast */ none)]; $($rest)*)
    };
    (@process $args:expr, $emitter:expr, $e:expr, [$($acc:tt)*] ; $name:ident ?: $ty:ident $($rest:tt)*) => {
        extract_args!(@process $args, $emitter, $e, [$($acc)* ($name, /* optional */ true, $ty, /* cast */ none)]; $($rest)*)
    };
    /* required argument */
    (@extract $args:ident, $emitter:expr, $e:expr, $name:ident, true, $ty:ident, none) => {{
        match $args.remove(stringify!($name)) {
            Some(Spanned { inner: parsers::Arg::$ty(v), span }) => Spanned { inner: Some(v), span },
            None => Spanned { inner: None, span: $e.span() },
            Some(Spanned { span, .. }) => {
                $emitter.emit(Rich::custom(span, format!("expected argument `{}` of type `{}`", stringify!($name), stringify!($ty))));
                Spanned { inner: None, span: $e.span() }
            }
        }
    }};
    /* optional argument */
    (@extract $args:ident, $emitter:expr, $e:expr, $name:ident, false, $ty:ident, none) => {{
        match $args.remove(stringify!($name)) {
            Some(Spanned { inner: parsers::Arg::$ty(v), span }) => Spanned { inner: v, span },
            None => {
                $emitter.emit(Rich::custom($e.span(), format!("missing argument `{}` of type `{}`", stringify!($name), stringify!($ty))));
                Spanned { inner: Default::default(), span: $e.span() }
            },
            Some(Spanned { span, .. }) => {
                $emitter.emit(Rich::custom(span, format!("expected argument `{}` of type `{}`", stringify!($name), stringify!($ty))));
                Spanned { inner: Default::default(), span }
            }
        }
    }};
    /* required `String` with cast */
    (@extract $args:ident, $emitter:expr, $e:expr, $name:ident, true, String, as $cast:ident) => {{
        match $args.remove(stringify!($name)) {
            Some(Spanned { inner: parsers::Arg::String(v), span }) => {
                let cast = <$cast as std::str::FromStr>::from_str(v.as_str()).map_err(|err| {
                    $emitter.emit(Rich::custom(span, format!("invalid `{}`: {err}", stringify!($cast))));
                }).ok();
                Some(Spanned { inner: cast, span })
            },
            None => None,
            Some(Spanned { span, .. }) => {
                $emitter.emit(Rich::custom(span, format!("expected argument `{}` of type `{}`", stringify!($name), stringify!(String))));
                None
            }
        }
    }};
    /* optional `String` with cast */
    (@extract $args:ident, $emitter:expr, $e:expr, $name:ident, false, String, as $cast:ident) => {{
        match $args.remove(stringify!($name)) {
            Some(Spanned { inner: parsers::Arg::String(v), span }) => {
                <$cast as std::str::FromStr>::from_str(v.as_str()).map_err(|err| {
                    $emitter.emit(Rich::custom(span, format!("invalid `{}`: {err}", stringify!($cast))));
                })
                .ok()
                .map(|cast| Spanned { inner: cast, span })
            },
            None => {
                $emitter.emit(Rich::custom($e.span(), format!("missing argument `{}` of type `{}`", stringify!($name), stringify!(String))));
                None
            },
            Some(Spanned { span, .. }) => {
                $emitter.emit(Rich::custom(span, format!("expected argument `{}` of type `{}`", stringify!($name), stringify!(String))));
                None
            }
        }
    }};
    (@build $args:expr, $emitter:expr, $e:expr, [
        $(($name:ident, $opt:tt, $ty:ident, $($cast:tt)*))*
    ]) => {{
        #[allow(unused_mut)]
        let mut args = $args;
        $(let $name = extract_args!(@extract args, $emitter, $e, $name, $opt, $ty, $($cast)*);)*

        for (name, arg) in args.into_iter() {
            $emitter.emit(Rich::custom(arg.span, format!("unexpected argument `{name}`")))
        }

        #[allow(clippy::unused_unit)]
        ($($name),*)
    }};
}

/// Different types of arguments.
/// Function arguments are specified in `name=<arg>` where `<arg>` is of
/// [`Arg`] type.
pub enum Arg {
    /// An identifier.
    /// Unlike [`Arg::String`] those are not wrapped in quotes.
    Ident(String),
    /// A string.
    /// Strings need to be wrapped in quotes.
    String(String),
    /// A float.
    Float(f32),
    /// A boolean.
    /// Either `true` or `false`.
    Bool(bool),
    /// A time duration.
    /// The duration is parsed from minutes, seconds, milliseconds, or microseconds.
    /// For example, `5s` will be parsed as a five second duration.
    Time(Duration),
}

/// Parser for an argument.
/// It returns a tuple with an argument name and an [`Arg`] value.
pub fn arg<'a>() -> impl Parser<'a, &'a str, (String, Spanned<Arg>), E<'a>> {
    let int = text::int(10);
    let frac = just(".").then(text::digits(10));
    let float = just("-")
        .or_not()
        .then(int)
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
        .validate(|(n, duration), e, emitter| {
            let seconds = duration.as_secs_f32() * n;
            if seconds.is_sign_negative() {
                emitter.emit(Rich::custom(
                    e.span(),
                    format!("`{seconds}s` is invalid time: time cannot be negative"),
                ));
                Duration::from_secs(0)
            } else {
                Duration::from_secs_f32(seconds)
            }
        })
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

/// Parse an identifier and convert it into a specific type which implements
/// [`FromStr`].
pub fn ident<'a, Transform>()
-> impl Parser<'a, &'a str, Option<Spanned<(Transform, &'a str)>>, E<'a>>
where
    Transform: FromStr,
    <Transform as FromStr>::Err: Display,
{
    text::ident().validate(|ident, e, emitter| {
        Transform::from_str(ident)
            .map_err(|err| {
                emitter.emit(Rich::custom(e.span(), format!("invalid identifier: {err}")));
            })
            .ok()
            .map(|value| Spanned {
                inner: (value, ident),
                span: e.span(),
            })
    })
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
