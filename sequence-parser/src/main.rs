use std::{fs, path::PathBuf, process::exit};

use ariadne::{Label, Report, ReportKind, Source};
use chumsky::Parser as _;
use clap::Parser;
use colored::Colorize;
use sequence_parser::Command;
use sequence_parser::sequence;

#[derive(Parser)]
#[command(version, about = "Parser for `.seq` sequence files", long_about = None)]
struct Cli {
    /// Path to the sequence to parse.
    path: PathBuf,
    /// Available sensor names.
    #[arg(short, long, value_delimiter = ',')]
    sensors: Vec<String>,
    /// Available actuator names.
    #[arg(short, long, value_delimiter = ',')]
    actuators: Vec<String>,
    /// Available state machine states.
    #[arg(short = 'm', long, value_delimiter = ',')]
    states: Vec<String>,
}

fn main() {
    let cli = Cli::parse();

    let path = match cli.path.canonicalize() {
        Ok(path) => path,
        Err(err) => {
            eprintln!("[!!] unable to canonicalize the provided path: {err}");
            exit(1)
        }
    };

    let sequence_str = match fs::read_to_string(cli.path) {
        Ok(sequence_str) => sequence_str,
        Err(err) => {
            eprintln!("[!!] unable to read sequence file: {err}");
            exit(1)
        }
    };

    let (commands, errors) = sequence(&cli.actuators, &cli.sensors, &cli.states)
        .parse(&sequence_str)
        .into_output_errors();

    let has_errors = !errors.is_empty();

    let file_id = path.display().to_string();

    for error in errors {
        Report::build(
            ReportKind::Error,
            (file_id.clone(), error.span().into_range()),
        )
        .with_message(error.to_string())
        .with_label(
            Label::new((file_id.clone(), error.span().into_range()))
                .with_message("here is the problem"),
        )
        .finish()
        .print((file_id.clone(), Source::from(sequence_str.clone())))
        .expect("should be able to print");
    }

    if has_errors {
        // We don't want to print any parsed commands when we have errors. This is
        // because we use `Default::default()` when there are errors during argument
        // parsing.
        exit(1)
    }

    let commands = commands.unwrap_or_default();
    let width = commands.len().to_string().len();

    commands.iter().enumerate().for_each(|(idx, command)| {
        let command_fmt = match command {
            Command::Msg(_) | Command::Warning(_) | Command::Error(_) => {
                format!("{command:?}").blue()
            }
            Command::Wait(_) => format!("{command:?}").bright_black(),
            Command::ToState(_) => format!("{command:?}").bright_red(),
            Command::AcutatorOpen(_) | Command::ActuatorClose(_) => format!("{command:?}").yellow(),
            Command::SensorWatch { .. }
            | Command::SensorIgnore { .. }
            | Command::SensorDetect { .. } => format!("{command:?}").cyan(),
        };
        println!(
            "{} ❘ {command_fmt}",
            format!("{idx:>width$}").bright_black()
        );
    });
}

#[cfg(test)]
mod tests {
    use chumsky::Parser;
    use sequence_parser::sequence;

    #[test]
    fn test_firing3_seq() {
        let sequence_str = include_str!("../tests/firing3.seq");
        let (commands, errors) = sequence(
            &[
                String::from("IGN_PRG_VLV"),
                String::from("OSS_PRG_VLV"),
                String::from("FSS_PRG_VLV"),
                String::from("OSS_PRZ_DPR"),
                String::from("OSS_MNL_VLV"),
                String::from("FSS_PRZ_DPR"),
                String::from("FSS_MNL_VLV"),
                String::from("IGN_SPK_PLG"),
                String::from("IGN_OXD_VLV"),
                String::from("IGN_FUE_VLV"),
            ],
            &[String::from("IGN_CC_P"), String::from("ENG_CC_P1")],
            &[String::from("firing_abort")],
        )
        .parse(sequence_str)
        .into_output_errors();
        // The sequence should be valid.
        assert!(errors.is_empty());
        assert_eq!(commands.unwrap_or_default().len(), 49);
    }

    #[test]
    fn test_ignition_no_engine_seq() {
        let sequence_str = include_str!("../tests/ignition_no_engine.seq");
        let (_commands, errors) = sequence(&[], &[], &[])
            .parse(sequence_str)
            .into_output_errors();
        // The sequence should be invalid.
        assert_eq!(errors.len(), 7);
    }
}
