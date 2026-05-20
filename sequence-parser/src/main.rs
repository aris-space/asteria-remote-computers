use std::{fs, path::PathBuf, process::exit};

use ariadne::{Label, Report, ReportKind, Source};
use chumsky::error::Rich;
use clap::Parser;
use colored::Colorize;
use sequence_parser::{Command, parse_sequence_str};

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

    match parse_sequence_str::<String, String, String>(&sequence_str) {
        Ok(commands) => print_sequence_commands(&commands),
        Err(errors) => {
            let file_id = path.display().to_string();
            print_parser_errors(&file_id, &sequence_str, errors);
            exit(1)
        }
    }
}

fn print_sequence_commands(commands: &[Command<String, String, String>]) {
    let width = commands.len().to_string().len();

    commands.iter().enumerate().for_each(|(idx, command)| {
        let command_fmt = match command {
            Command::Msg(_) | Command::Warning(_) | Command::Error(_) => {
                format!("{command:?}").blue()
            }
            Command::Wait(_) => format!("{command:?}").bright_black(),
            Command::ToState(_) => format!("{command:?}").bright_red(),
            Command::ActuatorOpen(_)
            | Command::ActuatorClose(_)
            | Command::DprActivate { .. }
            | Command::DprDeactivate(_) => format!("{command:?}").yellow(),
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

fn print_parser_errors(file_id: &String, sequence_str: &String, errors: Vec<Rich<'_, char>>) {
    for error in errors {
        Report::build(ReportKind::Error, (&file_id, error.span().into_range()))
            .with_message(error.to_string())
            .with_label(
                Label::new((&file_id, error.span().into_range()))
                    .with_message("here is the problem"),
            )
            .finish()
            .print((&file_id, Source::from(sequence_str)))
            .expect("should be able to print");
    }
}

#[cfg(test)]
mod tests {
    use sequence_parser::parse_sequence_str;

    #[test]
    fn test_firing3_seq() {
        let sequence_str = include_str!("../tests/firing3.seq");
        let commands = parse_sequence_str::<String, String, String>(sequence_str)
            .expect("The sequence should be valid");
        assert_eq!(commands.len(), 49);
    }

    #[test]
    fn test_ignition_no_engine_seq() {
        let sequence_str = include_str!("../tests/ignition_no_engine.seq");
        let errors = parse_sequence_str::<String, String, String>(sequence_str)
            .expect_err("The sequence should be invalid");
        assert_eq!(errors.len(), 1);
    }
}
