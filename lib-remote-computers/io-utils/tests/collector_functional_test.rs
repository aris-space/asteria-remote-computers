use io_utils::Collector;

// ---------------------------------------------------------------------------
// Message type
// ---------------------------------------------------------------------------

#[derive(Debug, PartialEq, Clone)]
enum Msg {
    A(u32),
    B(String),
    C(bool),
}

// ---------------------------------------------------------------------------
// Structs
// ---------------------------------------------------------------------------

/// Handles Msg::A only.
#[derive(Collector, Default)]
#[collector(message_type = "Msg", update_expr = "#field = #value;")]
struct Inner {
    #[datapoint(pattern = "Msg::A(#value)")]
    a: u32,
}

/// Handles Msg::B directly; delegates the rest to Inner via flatten.
/// Msg::C is not handled by either, so it falls through to Err.
#[derive(Collector, Default)]
#[collector(message_type = "Msg", update_expr = "#field = #value;")]
struct Outer {
    #[datapoint(pattern = "Msg::B(#value)")]
    b: String,

    #[collector(flatten)]
    inner: Inner,
}

/// Flatten-only struct: no direct patterns, purely delegates.
#[derive(Collector, Default)]
#[collector(message_type = "Msg")]
struct Wrapper {
    #[collector(flatten)]
    outer: Outer,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn direct_pattern_is_matched() {
    let mut outer = Outer::default();
    outer
        .update_from(Msg::B("hello".to_string()))
        .expect("B should match");
    assert_eq!(outer.b, "hello");
}

#[test]
fn flatten_delegates_unmatched_to_inner() {
    let mut outer = Outer::default();
    outer
        .update_from(Msg::A(42))
        .expect("A should reach Inner via flatten");
    assert_eq!(outer.inner.a, 42);
}

#[test]
fn unhandled_message_returns_err() {
    let mut outer = Outer::default();
    let err = outer
        .update_from(Msg::C(true))
        .expect_err("C is not handled");
    assert_eq!(err, Msg::C(true));
}

#[test]
fn flatten_only_wrapper_delegates_all() {
    let mut wrapper = Wrapper::default();
    wrapper
        .update_from(Msg::B("world".to_string()))
        .expect("B reaches Outer via Wrapper flatten");
    assert_eq!(wrapper.outer.b, "world");

    wrapper
        .update_from(Msg::A(7))
        .expect("A reaches Inner via two flatten levels");
    assert_eq!(wrapper.outer.inner.a, 7);

    let err = wrapper
        .update_from(Msg::C(false))
        .expect_err("C still unhandled");
    assert_eq!(err, Msg::C(false));
}
