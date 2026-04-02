use std::sync::mpsc;

use io_utils::{Broadcast, BroadcastLoop, NeverReturns};
use watchable::Watchable;

// ---------------------------------------------------------------------------
// Message type
// ---------------------------------------------------------------------------

#[derive(Debug, PartialEq)]
enum Msg {
    A(bool),
    B(u32),
}

// ---------------------------------------------------------------------------
// A BroadcastLoop that immediately forwards the current field value once,
// then idles. This lets the smoke test verify that the loop is called and
// the map closure is wired to the correct variant.
// Depending on the message type, the given frequencies are also asserted.
// ---------------------------------------------------------------------------

struct TestLoop;

impl BroadcastLoop<Msg> for TestLoop {
    async fn broadcast_loop<T: Send + Sync + 'static + Clone>(
        field: Watchable<T>,
        mut map: impl FnMut(T) -> Msg + Send + 'static,
        channel: mpsc::Sender<Msg>,
        min_freq_hz: f32,
        max_freq_hz: f32,
    ) -> NeverReturns {
        let val = map(field.get());
        #[allow(clippy::float_cmp)]
        match val {
            Msg::A(_) => {
                assert_eq!(min_freq_hz, 1.);
                assert_eq!(max_freq_hz, 10.);
            }
            Msg::B(_) => {
                assert_eq!(min_freq_hz, 2.);
                assert_eq!(max_freq_hz, 20.);
            }
        }
        let _ = channel.send(val);
        std::future::pending().await
    }
}

// ---------------------------------------------------------------------------
// Structs
// ---------------------------------------------------------------------------

#[derive(Broadcast)]
#[broadcast(message_type = "Msg")]
struct Inner {
    #[datapoint(pattern = "Msg::A(#value)")]
    #[broadcast(min_freq_hz = 1., max_freq_hz = 10.)]
    a: Watchable<bool>,
}

#[derive(Broadcast)]
#[broadcast(message_type = "Msg")]
struct Outer {
    #[broadcast(flatten)]
    inner: Inner,

    #[datapoint(pattern = "Msg::B(#value)")]
    #[broadcast(min_freq_hz = 2., max_freq_hz = 20.)]
    b: Watchable<u32>,

    /// not broadcast
    _c: Watchable<String>,
}

// ---------------------------------------------------------------------------
// Smoke test
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "current_thread")]
async fn smoke() {
    let outer = Outer {
        inner: Inner {
            a: Watchable::new(true),
        },
        b: Watchable::new(42),
        _c: Watchable::new(String::new()),
    };
    let (tx, rx) = mpsc::channel::<Msg>();

    let handles = outer.start_broadcasting::<TestLoop>(tx);

    // Flatten contributes Inner's handles; Outer adds its own — 2 total.
    assert_eq!(handles.len(), 2);

    // Let spawned tasks run and send their first value each.
    for _ in 0..4 {
        tokio::task::yield_now().await;
    }

    let msgs: Vec<_> = rx.try_iter().collect();

    assert_eq!(msgs.len(), 2, "each broadcast field should have fired once");
    assert!(
        msgs.contains(&Msg::A(true)),
        "expected Msg::A(7) from Inner::a via flatten; got {msgs:?}"
    );
    assert!(
        msgs.contains(&Msg::B(42)),
        "expected Msg::B(42) from Outer::b; got {msgs:?}"
    );

    for h in handles {
        h.abort();
    }
}

// ---------------------------------------------------------------------------
// extract_expr test
// ---------------------------------------------------------------------------

/// struct_level extract_expr wraps every field value before putting it in
/// the variant. Here `extract_expr = "#value * 2"` doubles the stored value.
#[derive(Broadcast)]
#[broadcast(message_type = "Msg", extract_expr = "#value * 2")]
struct Doubled {
    #[datapoint(pattern = "Msg::B(#value)")]
    #[broadcast(min_freq_hz = 2., max_freq_hz = 20.)]
    b: Watchable<u32>,
}

#[tokio::test(flavor = "current_thread")]
async fn extract_expr_transforms_value() {
    let doubled = Doubled {
        b: Watchable::new(21),
    };
    let (tx, rx) = mpsc::channel::<Msg>();

    let handles = doubled.start_broadcasting::<TestLoop>(tx);

    let msgs = tokio::task::spawn_blocking(move || {
        let timeout = std::time::Duration::from_millis(500);
        vec![rx.recv_timeout(timeout).expect("timed out")]
    })
    .await
    .expect("must be able await channel for test");

    assert_eq!(
        msgs,
        vec![Msg::B(42)],
        "extract_expr should double the value"
    );

    for h in handles {
        h.abort();
    }
}
