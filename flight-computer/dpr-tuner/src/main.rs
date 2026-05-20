//! Terminal UI for tuning DPR PID gains on the Fuel and Oxidizer Control Boards over CAN.
//!
//! Run with `dpr-tuner [can-interface]` (default: `can0`). Current gain values are read from
//! incoming `BoardStatus` messages; edited values are sent as individual gain messages.
//!
//! On non-linux the backend is intentionally disabled.

use std::io::{Result as IoResult, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crossterm::cursor::MoveTo;
use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use crossterm::style::{Color, Print, ResetColor, SetBackgroundColor, SetForegroundColor};
use crossterm::terminal::{
    self, disable_raw_mode, enable_raw_mode, Clear, ClearType, EnterAlternateScreen,
    LeaveAlternateScreen,
};
use crossterm::{ExecutableCommand, QueueableCommand as _};
use datatypes::status::DprGainInfo;
#[cfg(target_os = "linux")]
use socketcan::BlockingCan as _;

/// Which control board is currently focused.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Board {
    Fuel,
    Oxidizer,
}

/// The five tuneable DPR parameters, in display order.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Field {
    P,
    I,
    D,
    MinMs,
    MaxMs,
}

impl Field {
    const ALL: [Field; 5] = [Field::P, Field::I, Field::D, Field::MinMs, Field::MaxMs];

    /// Human-readable label shown in the TUI row.
    fn label(self) -> &'static str {
        match self {
            Field::P => "P Gain",
            Field::I => "I Gain",
            Field::D => "D Gain",
            Field::MinMs => "Min open ms",
            Field::MaxMs => "Max open ms",
        }
    }

    /// Extract this field's value from a `DprGainInfo`.
    fn from_gain(self, g: &DprGainInfo) -> f32 {
        match self {
            Field::P => g.p,
            Field::I => g.i,
            Field::D => g.d,
            Field::MinMs => g.min_ms,
            Field::MaxMs => g.max_ms,
        }
    }
}

/// Shared mutable state updated by the CAN receive thread and read by the render loop.
struct AppState {
    /// Latest gain values received from the Fuel Control Board, `None` until first status arrives.
    fuel: Option<DprGainInfo>,
    /// Latest gain values received from the Oxidizer Control Board.
    oxidizer: Option<DprGainInfo>,
    /// One-line status message shown at the bottom of the screen.
    status: String,
}

impl AppState {
    fn new() -> Self {
        Self {
            fuel: None,
            oxidizer: None,
            status: String::from("Listening for BoardStatus…"),
        }
    }
}

/// Tracks which cell is focused and holds the in-progress edit buffer.
struct Cursor {
    board: Board,
    /// Index into [`Field::ALL`].
    field: usize,
    editing: bool,
    input: String,
}

impl Cursor {
    fn new() -> Self {
        Self {
            board: Board::Fuel,
            field: 0,
            editing: false,
            input: String::new(),
        }
    }

    fn current_field(&self) -> Field {
        Field::ALL[self.field]
    }

    fn move_up(&mut self) {
        if self.field > 0 {
            self.field -= 1;
        }
    }

    fn move_down(&mut self) {
        if self.field + 1 < Field::ALL.len() {
            self.field += 1;
        }
    }

    fn toggle_board(&mut self) {
        self.board = match self.board {
            Board::Fuel => Board::Oxidizer,
            Board::Oxidizer => Board::Fuel,
        };
    }
}

/// Build the `messages::Message` for sending a single gain value.
fn build_message(board: Board, field: Field, value: f32) -> messages::Message {
    use dp_fuel_control_board::Message as Fuel;
    use dp_oxidizer_control_board::Message as Oxid;
    use messages::Message;

    match board {
        Board::Fuel => match field {
            Field::P => Message::FuelControlBoard(Fuel::FuelDprGainP(value)),
            Field::I => Message::FuelControlBoard(Fuel::FuelDprGainI(value)),
            Field::D => Message::FuelControlBoard(Fuel::FuelDprGainD(value)),
            Field::MinMs => Message::FuelControlBoard(Fuel::FuelDprMinOpeningTime(value)),
            Field::MaxMs => Message::FuelControlBoard(Fuel::FuelDprMaxOpeningTime(value)),
        },
        Board::Oxidizer => match field {
            Field::P => Message::OxidizerControlBoard(Oxid::OxidizerDprGainP(value)),
            Field::I => Message::OxidizerControlBoard(Oxid::OxidizerDprGainI(value)),
            Field::D => Message::OxidizerControlBoard(Oxid::OxidizerDprGainD(value)),
            Field::MinMs => Message::OxidizerControlBoard(Oxid::OxidizerDprMinOpeningTime(value)),
            Field::MaxMs => Message::OxidizerControlBoard(Oxid::OxidizerDprMaxOpeningTime(value)),
        },
    }
}

/// Spawns the CAN receive and transmit threads and returns the transmit sender.
///
/// On non-Linux the socket is replaced by a no-op drain so the rest of the code compiles and runs.
fn spawn_can_threads(
    shared: Arc<Mutex<AppState>>,
    interface: &str,
) -> std::sync::mpsc::Sender<messages::Message> {
    let (tx, rx) = std::sync::mpsc::channel::<messages::Message>();

    #[cfg(target_os = "linux")]
    {
        use can_hal::CanDecode as _;
        use can_hal::CanEncode as _;
        use embedded_can::Frame as _;
        use socketcan::Socket as _;

        let sock_rx = socketcan::CanFdSocket::open(interface).expect("open CAN socket");
        let sock_tx = {
            use std::os::fd::AsFd as _;
            socketcan::CanFdSocket::from(sock_rx.as_fd().try_clone_to_owned().expect("dup fd"))
        };

        let shared_tx = Arc::clone(&shared);
        std::thread::spawn(move || {
            let mut sock = sock_rx;
            loop {
                let frame = match sock.receive() {
                    Ok(f) => f,
                    Err(e) => {
                        shared.lock().expect("lock").status =
                            format!("CAN receive error: {e:?}");
                        std::thread::sleep(Duration::from_millis(1));
                        continue;
                    }
                };
                let id = match frame.id() {
                    embedded_can::Id::Standard(id) => id,
                    embedded_can::Id::Extended(e) => e.standard_id(),
                };
                // Unknown IDs are normal (other bus traffic) — skip silently.
                let Ok(msg) = messages::Message::from_parts(id, frame.data()) else {
                    continue;
                };
                let mut st = shared.lock().expect("lock");
                match &msg {
                    messages::Message::FuelControlBoard(
                        dp_fuel_control_board::Message::BoardStatus(s),
                    ) => st.fuel = Some(s.dpr_gain_info.clone()),
                    messages::Message::OxidizerControlBoard(
                        dp_oxidizer_control_board::Message::BoardStatus(s),
                    ) => st.oxidizer = Some(s.dpr_gain_info.clone()),
                    _ => {}
                }
            }
        });

        std::thread::spawn(move || {
            let mut sock = sock_tx;
            for msg in rx {
                let mut buf = [0u8; 64];
                let Ok((id, len)) = msg.encode_into(&mut buf) else {
                    continue;
                };
                let Some(frame) = socketcan::CanAnyFrame::new(id, &buf[..len as usize]) else {
                    continue;
                };
                if let Err(e) = sock.transmit(&frame) {
                    shared_tx.lock().expect("lock").status =
                        format!("CAN transmit error: {e:?}");
                }
            }
        });
    }

    #[cfg(not(target_os = "linux"))]
    {
        let _ = (interface, shared);
        std::thread::spawn(move || for _ in rx {});
    }

    tx
}

/// Redraws the full screen: two board panels side by side, then status and help lines.
fn render(out: &mut impl Write, state: &AppState, cursor: &Cursor) -> IoResult<()> {
    let (cols, rows) = terminal::size()?;
    let col_width = cols / 2;

    out.queue(Clear(ClearType::All))?;

    draw_board(out, 0, col_width, "FUEL", Board::Fuel, state, cursor)?;
    draw_board(
        out,
        col_width,
        col_width,
        "OXIDIZER",
        Board::Oxidizer,
        state,
        cursor,
    )?;

    out.queue(MoveTo(0, rows - 2))?
        .queue(SetForegroundColor(Color::Yellow))?
        .queue(Print(&state.status))?
        .queue(ResetColor)?;

    let help = if cursor.editing {
        " [Enter] Send  [Esc] Cancel"
    } else {
        " [↑↓] Navigate  [←→] Switch board  [Enter] Edit  [Q] Quit"
    };
    out.queue(MoveTo(0, rows - 1))?
        .queue(SetForegroundColor(Color::DarkGrey))?
        .queue(Print(help))?
        .queue(ResetColor)?;

    out.flush()?;
    Ok(())
}

/// Draws one board panel starting at the given column offset with the given width.
fn draw_board(
    out: &mut impl Write,
    col: u16,
    col_width: u16,
    title: &str,
    board: Board,
    state: &AppState,
    cursor: &Cursor,
) -> IoResult<()> {
    let width = col_width as usize;
    let is_active = cursor.board == board;
    let title_color = if is_active {
        Color::Cyan
    } else {
        Color::DarkGrey
    };
    let gains = match board {
        Board::Fuel => &state.fuel,
        Board::Oxidizer => &state.oxidizer,
    };

    out.queue(MoveTo(col, 0))?
        .queue(SetForegroundColor(title_color))?
        .queue(Print(format!(
            "─── {title} DPR {}",
            "─".repeat(width.saturating_sub(title.len() + 9))
        )))?
        .queue(ResetColor)?;

    for (i, field) in Field::ALL.iter().enumerate() {
        let selected = is_active && cursor.field == i;
        let current_val = gains
            .as_ref()
            .map(|g| format!("{:.4}", field.from_gain(g)))
            .unwrap_or_else(|| "--".to_string());

        let line = if selected && cursor.editing {
            format!(
                " {:<12} {:>8}  > {}_",
                field.label(),
                current_val,
                cursor.input
            )
        } else {
            format!(" {:<12} {:>8}", field.label(), current_val)
        };

        out.queue(MoveTo(col, (i as u16) + 1))?;
        if selected {
            out.queue(SetBackgroundColor(Color::DarkGrey))?;
        }
        out.queue(Print(format!("{line:<width$}")))?;
        if selected {
            out.queue(ResetColor)?;
        }
    }

    Ok(())
}

/// Entry point: opens the CAN socket and runs the TUI event loop.
fn main() -> IoResult<()> {
    let interface = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "can0".to_string());

    let shared = Arc::new(Mutex::new(AppState::new()));
    let tx = spawn_can_threads(Arc::clone(&shared), &interface);

    let mut stdout = std::io::stdout();
    enable_raw_mode()?;
    stdout.execute(EnterAlternateScreen)?;

    let mut cursor = Cursor::new();

    loop {
        {
            let st = shared.lock().expect("lock");
            render(&mut stdout, &st, &cursor)?;
        }

        if event::poll(Duration::from_millis(100))? {
            if let Event::Key(key) = event::read()? {
                if key.kind != KeyEventKind::Press {
                    continue;
                }
                if cursor.editing {
                    match key.code {
                        KeyCode::Esc => {
                            cursor.editing = false;
                            cursor.input.clear();
                        }
                        KeyCode::Backspace => {
                            cursor.input.pop();
                        }
                        KeyCode::Char(c) if c.is_ascii_digit() || c == '.' || c == '-' => {
                            cursor.input.push(c);
                        }
                        KeyCode::Enter => {
                            match cursor.input.parse::<f32>() {
                                Ok(v) => {
                                    let _ = tx.send(build_message(
                                        cursor.board,
                                        cursor.current_field(),
                                        v,
                                    ));
                                    let board_name = match cursor.board {
                                        Board::Fuel => "Fuel",
                                        Board::Oxidizer => "Oxidizer",
                                    };
                                    shared.lock().expect("lock").status = format!(
                                        "Sent {} {} = {v}",
                                        board_name,
                                        cursor.current_field().label(),
                                    );
                                }
                                Err(_) => {
                                    shared.lock().expect("lock").status =
                                        format!("Invalid number: '{}'", cursor.input);
                                }
                            }
                            cursor.editing = false;
                            cursor.input.clear();
                        }
                        _ => {}
                    }
                } else {
                    match key.code {
                        KeyCode::Char('q') | KeyCode::Char('Q') => break,
                        KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                            break;
                        }
                        KeyCode::Left | KeyCode::Right => cursor.toggle_board(),
                        KeyCode::Up => cursor.move_up(),
                        KeyCode::Down => cursor.move_down(),
                        KeyCode::Enter => {
                            cursor.editing = true;
                            cursor.input.clear();
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    disable_raw_mode()?;
    stdout.execute(LeaveAlternateScreen)?;
    Ok(())
}
