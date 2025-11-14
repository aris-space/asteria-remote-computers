//! Utilities to output logs and timeseries to disk.

use std::path::{PathBuf};
use tokio::io;

pub const LOGS_PATH: &str = "./logs/";

/// Creates a folder in [`LOGS_PATH`] with this name. It should already be unique, i.e. contain a timestamp.
pub async fn make_mission_log_folder(unique_name: &str) -> io::Result<PathBuf> {
    let mut path = PathBuf::from(LOGS_PATH);
    path.push(unique_name);
    tokio::fs::create_dir_all(&path).await?;

    Ok(path)
}
