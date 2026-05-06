use std::{fs, path::PathBuf, process::exit};

use ariadne::{Label, Report, ReportKind, Source};
use chumsky::Parser as _;
use clap::Parser;
use colored::Colorize;
use sequences::sequence;

#[derive(Parser)]
#[command(version, about = "Parser for `.seq` sequence files", long_about = None)]
struct Cli {
    /// Path to the sequence to parse.
    path: PathBuf,
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

    let (commands, errors) = sequence().parse(&sequence_str).into_output_errors();

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

    let commands = commands.unwrap_or_default();
    let width = commands.len().to_string().len();

    commands.iter().enumerate().for_each(|(idx, command)| {
        let command_fmt = match command {
            sequences::Command::Msg(_)
            | sequences::Command::Warning(_)
            | sequences::Command::Error(_) => format!("{command:?}").blue(),
            sequences::Command::Wait(_) => format!("{command:?}").bright_black(),
            sequences::Command::ToState(_) => format!("{command:?}").bright_red(),
            sequences::Command::AcutatorOpen(_) | sequences::Command::ActuatorClose(_) => {
                format!("{command:?}").yellow()
            }
            sequences::Command::SensorWatch { .. }
            | sequences::Command::SensorIgnore { .. }
            | sequences::Command::SensorDetect { .. } => format!("{command:?}").cyan(),
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
    use sequences::sequence;

    #[test]
    fn test_firing3_seq() {
        let sequence_str = include_str!("../tests/firing3.seq");
        let (commands, errors) = sequence().parse(sequence_str).into_output_errors();
        // The sequence should be valid.
        assert!(errors.is_empty());
        assert_eq!(commands.unwrap_or_default().len(), 49);
    }

    #[test]
    fn test_ignition_no_engine_seq() {
        let sequence_str = include_str!("../tests/ignition_no_engine.seq");
        let (commands, errors) = sequence().parse(sequence_str).into_output_errors();
        // The sequence should be invalid.
        assert_eq!(errors.len(), 1);
        assert_eq!(commands.unwrap_or_default().len(), 0);
    }
}
