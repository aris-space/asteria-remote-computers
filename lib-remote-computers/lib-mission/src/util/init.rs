//! Utilities to start a mission binary in a uniform way.

use std::path::{Path, PathBuf};

use fs4::fs_std::FileExt;
use sha1::Digest as _;
use tracing::level_filters::LevelFilter;
use tracing_subscriber::fmt::format::FmtSpan;
use tracing_subscriber::{EnvFilter, prelude::*};

use super::log::make_mission_log_folder;

/// Collects the important information from this build.
#[derive(Debug, serde::Serialize)]
pub struct BuildInfo {
    /// The name of the binary that was build. E.g. the cargo package the build script was run in.
    pub binary: &'static str,
    /// The full git commit hash that is checked out. "{unknown}" if not found. 40 chars.
    pub git_commit: &'static str,
    /// The tag or 7 digits of the hash of the git commit that is checked out. "{unknown}" if not found. As long as the tag.
    pub git_tag: &'static str,
    /// `true` if it cannot be ensured the code is exactly as `git_commit` suggests.
    pub git_dirty: bool,
    /// `true` if the this was built in release mode.
    pub is_release: bool,
    /// A comma separated list of features enabled for this build.
    pub features: &'static str,
    /// Unix timestamp in seconds when the build was compiled. u32 lasts until 2038.
    pub unix_timestamp: u32,
    /// Initials of the person who compiled the firmware, or `\0\0` if not set.
    pub author_initials: [u8; 2],
}

/// Collects important information about this execution.
#[derive(Debug, serde::Serialize)]
pub struct RunInfo {
    /// The system time in unix millies when the executable was started.
    pub time_started_unix_ms: u64,
    /// Information built into the binary about its version.
    pub build: BuildInfo,
    /// The filename of the config file.
    pub config_filename: String,
    /// The blob sha1 hash of the config file contents.
    pub config_hash: String,
}

const LOCKFILE_PATH: &str = "./flight_computer.lockfile";

/// Initialized the binary for a mission with tracing.
pub async fn init_binary<C: serde::de::DeserializeOwned + std::fmt::Debug>(
    build_info: BuildInfo,
    config_path: impl AsRef<Path>,
) -> Result<(C, PathBuf, RunInfo), Box<dyn std::error::Error>> {
    get_file_lock()?;
    // Idea: we could also refuse to start from a dirty repo (unless override env var is set)?

    #[allow(clippy::cast_possible_truncation)]
    let start_time = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("time moved backwards")
        .as_millis() as u64;

    // Parse and hash the config
    let config_content = tokio::fs::read_to_string(&config_path).await?;
    let config_hash = {
        let blob = format!("blob {}\0{config_content}", config_content.len());
        hex::encode(sha1::Sha1::digest(blob.as_bytes()))
    };
    let config_file: C = toml::from_str(&config_content)?;

    // Setup logging
    let log_name = format!(
        "{name}-{time}-{code_v}-{cfg_hash}",
        name = build_info.binary,
        code_v = &build_info.git_tag.replace('/', "-"),
        cfg_hash = &config_hash[..5],
        time = chrono::Local::now().format("%Y-%m-%dT%H-%M"),
    );
    let logs_folder = make_mission_log_folder(&log_name).await?;
    init_tracing(&logs_folder);

    // Collect info and write to file
    let run_info = RunInfo {
        time_started_unix_ms: start_time,
        build: build_info,
        config_filename: config_path
            .as_ref()
            .file_name()
            .ok_or("Weird config filename, refusing to start.")?
            .to_string_lossy()
            .into_owned(),
        config_hash,
    };
    {
        let mut path = PathBuf::from(&logs_folder);
        path.push("run_info.toml");
        let run_info_ser = toml::to_string_pretty(&run_info)?;
        tokio::fs::write(path, run_info_ser).await?;

        let mut path = PathBuf::from(&logs_folder);
        path.push("config.toml");
        tokio::fs::write(path, config_content).await?;
    }

    tracing::info!("run_info={run_info:#?}");
    tracing::debug!("config={config_file:#?}");

    Ok((config_file, logs_folder, run_info))
}

/// Sets up [`tracing`] to both print to STDOUT and logs to `logs/{mission_id}/trace.jsonl`.
fn init_tracing(logs_folder: &Path) {
    // Set up the formatter for logging to stdout
    let stdout_subscriber = tracing_subscriber::fmt::layer().with_filter(
        EnvFilter::builder()
            .with_default_directive(LevelFilter::TRACE.into())
            .from_env_lossy(),
    );

    // Set up the JSON subscriber for logging to a file
    let mut path = PathBuf::from(logs_folder);
    path.push("trace.jsonl");
    // TODO: GZ compression, but requires to be able to gracefully shut down.
    let file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
        .expect("Failed to open log file");
    let log_subscriber = tracing_subscriber::fmt::layer()
        .with_writer(file)
        .with_span_events(FmtSpan::NEW | FmtSpan::CLOSE)
        .json()
        .with_span_list(false);

    // Register both subscribers
    tracing_subscriber::registry()
        .with(stdout_subscriber)
        .with(log_subscriber)
        .init();

    // Example tracing events
    tracing::trace!("tracing initialized");
}

/// Locks the file `LOCKFILE_PATH` such that only one instance can run at a time.
///
/// Returns an error if this file can't be created and exits the program if the lock is already held.
fn get_file_lock() -> Result<(), Box<dyn std::error::Error>> {
    let lockfile = std::fs::File::create(LOCKFILE_PATH)?;
    if lockfile.try_lock_exclusive().is_err() {
        tracing::error!(
            "Another instance is already running. You must terminate it before starting a new one."
        );
        std::process::exit(1);
    }

    Ok(())
}
