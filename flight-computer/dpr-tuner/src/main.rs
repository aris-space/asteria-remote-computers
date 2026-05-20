//! Terminal UI for tuning DPR PID gains on the Fuel and Oxidizer Control Boards over CAN.
//!
//! Run with `dpr-tuner [can-interface]` (default: `can0`). Current gain values are read from
//! incoming `BoardStatus` messages; edited values are sent as individual gain messages.
//!
//! On non-linux the backend is intentionally disabled.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use crossterm::ExecutableCommand;
use datatypes::status::DprGainInfo;
use ratatui::prelude::*;
use ratatui::widgets::*;

// ─── Domain types ─────────────────────────────────────────────────────────────

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

// ─── App state ────────────────────────────────────────────────────────────────

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

// ─── UI cursor ────────────────────────────────────────────────────────────────

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

// ─── CAN I/O ──────────────────────────────────────────────────────────────────

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

        std::thread::spawn(move || {
            let mut sock = sock_rx;
            loop {
                let frame = match sock.receive() {
                    Ok(f) => f,
                    Err(_) => {
                        std::thread::sleep(Duration::from_millis(1));
                        continue;
                    }
                };
                let id = match frame.id() {
                    embedded_can::Id::Standard(id) => id,
                    embedded_can::Id::Extended(e) => e.standard_id(),
                };
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
                let Some(frame) = socketcan::CanFdFrame::new(id, &buf[..len as usize]) else {
                    continue;
                };
                let _ = sock.transmit(&frame);
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

// ─── Rendering ────────────────────────────────────────────────────────────────

/// Top-level render: splits the area into two board panels plus status/help rows.
fn render(frame: &mut Frame, state: &AppState, cursor: &Cursor) {
    let area = frame.area();

    let outer = Layout::vertical([
        Constraint::Min(0),
        Constraint::Length(1),
        Constraint::Length(1),
    ])
    .split(area);

    let panels = Layout::horizontal([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(outer[0]);

    render_board(frame, panels[0], "FUEL", Board::Fuel, state, cursor);
    render_board(frame, panels[1], "OXIDIZER", Board::Oxidizer, state, cursor);

    frame.render_widget(
        Paragraph::new(state.status.as_str()).style(Style::default().fg(Color::Yellow)),
        outer[1],
    );

    let help = if cursor.editing {
        " [Enter] Send  [Esc] Cancel"
    } else {
        " [↑↓] Navigate  [←→] Switch board  [Enter] Edit  [Q] Quit"
    };
    frame.render_widget(
        Paragraph::new(help).style(Style::default().fg(Color::DarkGray)),
        outer[2],
    );
}

/// Renders a single board panel (bordered box with one row per [`Field`]).
fn render_board(
    frame: &mut Frame,
    area: Rect,
    title: &str,
    board: Board,
    state: &AppState,
    cursor: &Cursor,
) {
    let is_active = cursor.board == board;
    let border_style = if is_active {
        Style::default().fg(Color::Cyan)
    } else {
        Style::default().fg(Color::DarkGray)
    };

    let block = Block::bordered()
        .title(format!(" {title} DPR "))
        .border_style(border_style);
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let gains = match board {
        Board::Fuel => &state.fuel,
        Board::Oxidizer => &state.oxidizer,
    };

    let rows: Vec<Constraint> = Field::ALL.iter().map(|_| Constraint::Length(2)).collect();
    let cells = Layout::vertical(rows).split(inner);

    for (i, field) in Field::ALL.iter().enumerate() {
        let selected = is_active && cursor.field == i;
        let current_val = gains
            .as_ref()
            .map(|g| format!("{:.4}", field.from_gain(g)))
            .unwrap_or_else(|| "--".to_string());

        let row_style = if selected {
            Style::default().bg(Color::DarkGray)
        } else {
            Style::default()
        };

        let display = if selected && cursor.editing {
            format!(
                "{:<12} current: {:<12} > {}_",
                field.label(),
                current_val,
                cursor.input
            )
        } else {
            format!("{:<12} current: {:<12}", field.label(), current_val)
        };

        frame.render_widget(Paragraph::new(display).style(row_style), cells[i]);
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────────

/// Entry point: opens the CAN socket and runs the TUI event loop.
fn main() -> std::io::Result<()> {
    let interface = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "can0".to_string());

    let shared = Arc::new(Mutex::new(AppState::new()));
    let tx = spawn_can_threads(Arc::clone(&shared), &interface);

    enable_raw_mode()?;
    let mut stdout = std::io::stdout();
    stdout.execute(EnterAlternateScreen)?;
    let mut terminal = Terminal::new(ratatui::backend::CrosstermBackend::new(stdout))?;

    let mut cursor = Cursor::new();

    loop {
        {
            let st = shared.lock().expect("lock");
            terminal.draw(|f| render(f, &st, &cursor))?;
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
                                        cursor.current_field().label()
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
    terminal.backend_mut().execute(LeaveAlternateScreen)?;
    Ok(())
}
