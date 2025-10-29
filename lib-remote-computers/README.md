# hermes-lib-remote-computers
This repo is the baseline of the HERMES flight computer and HERMES remote filling station computer.

## Usage
See the our flight computer repo for proper usage.

However, for testing, we have the `nicollier` crate. Here are some instructions to run this for simulations purposes.

- Setup: you need Rust installed (<https://rustup.rs>), and the repository cloned with all submodules (run `git submodule init && git submodule update`).
- Run `cargo run --package nicollier` in one terminal.
  This will start the state machine in the idle state and advance the time in a slowed down fashion.
  It will log all data changes to CSV files and send position etc periodically over MAVlink (Port and protocol are configured in `configs/nicollier_flight.toml`).
- Run `cargo run --package dummy-controller` in another terminal, entering the port (default 5760).
- You should see a bunch of messages received in the controller.
- Typing things in the controller is possible, but gets hidden by new messages. Just be confident. 
  Try pasting `1 Idle` or `1 BAD` or `1 SelfDiagnostics`. This will attempt sending transition commands.
- In `logs/niollier*/` there should be csv logs (note there are currently 2 folder, will be fixed in #28).
- Kill with CTRL-C each.
