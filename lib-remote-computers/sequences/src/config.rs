//! Helpers for embedding a sequence DSL string in a mission config file.

use std::collections::HashSet;
use std::fmt::Display;
use std::str::FromStr;

use ariadne::{Label, Report, ReportKind, Source};
use lib_mission::state_machine::StateName;
use lib_mission::util::init::CONFIG;
use serde::{Deserialize, Deserializer};

use crate::{Command, parse_sequence_str};

/// Custom deserializer for inline sequences in TOML strings.
pub fn deserialize_sequence<'de, D, Actuator, Sensor>(
    deserializer: D,
) -> Result<Vec<Command<Actuator, Sensor, String>>, D::Error>
where
    D: Deserializer<'de>,
    Actuator: Clone + FromStr,
    <Actuator as FromStr>::Err: Display,
    Sensor: Clone + FromStr,
    <Sensor as FromStr>::Err: Display,
{
    let sequence_str = String::deserialize(deserializer)?;
    let sequence_span = 0..sequence_str.len();

    let errors = match parse_sequence_str::<Actuator, Sensor, String>(&sequence_str) {
        Ok(commands) => return Ok(commands),
        Err(errors) => errors,
    };

    let (config_path, _, _) = CONFIG.get().expect("Static should be initialized");
    let config_file_id = config_path.display().to_string();

    let mut report_builder =
        Report::build(ReportKind::Error, (&config_file_id, sequence_span.clone()))
            .with_message("unable to parse sequences");

    for err in errors {
        report_builder = report_builder.with_label(
            Label::new((
                &config_file_id,
                (sequence_span.start + err.span().start)..(sequence_span.start + err.span().end),
            ))
            .with_message(err.to_string()),
        );
    }

    let mut output = Vec::new();
    report_builder
        .finish()
        .write_for_stdout((&config_file_id, Source::from(&sequence_str)), &mut output)
        .expect("Should write bytes");

    Err(serde::de::Error::custom(
        String::from_utf8(output).expect("Should be valid UTF-8"),
    ))
}

/// Validate the sequence by ensuring that all referenced states
/// actually exist on the state machine.
pub fn validate_sequence<Actuator, Sensor>(
    sequence: Vec<Command<Actuator, Sensor, String>>,
    names: &HashSet<StateName>,
) -> lib_mission::Result<Vec<Command<Actuator, Sensor, StateName>>>
where
    Actuator: Clone,
    Sensor: Clone,
{
    sequence
        .into_iter()
        .map(|command| {
            let safe_command = match command {
                Command::SensorWatch {
                    name,
                    abort_state,
                    max,
                    min,
                    key,
                } => Command::SensorWatch {
                    name,
                    abort_state: StateName::try_new(&abort_state, names)?,
                    max,
                    min,
                    key,
                },
                Command::SensorDetect {
                    name,
                    value,
                    rising,
                    abort_state,
                    patience,
                    duration,
                } => Command::SensorDetect {
                    name,
                    value,
                    rising,
                    abort_state: StateName::try_new(&abort_state, names)?,
                    patience,
                    duration,
                },
                Command::ToState(state) => Command::ToState(StateName::try_new(&state, names)?),
                Command::Msg(msg) => Command::Msg(msg),
                Command::Warning(msg) => Command::Warning(msg),
                Command::Error(msg) => Command::Error(msg),
                Command::Wait(duration) => Command::Wait(duration),
                Command::ActuatorOpen(name) => Command::ActuatorOpen(name),
                Command::ActuatorClose(name) => Command::ActuatorClose(name),
                Command::DprActivate { name, setpoint } => Command::DprActivate { name, setpoint },
                Command::DprDeactivate(name) => Command::DprDeactivate(name),
                Command::SensorIgnore { name, key } => Command::SensorIgnore { name, key },
            };
            Ok(safe_command)
        })
        .collect::<Result<Vec<_>, _>>()
}
