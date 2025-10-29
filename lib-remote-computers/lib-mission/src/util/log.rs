//! Utilities to output logs and timeseries to disk.

use std::path::{Path, PathBuf};

use csv_async::AsyncSerializer;
use tokio::fs::File;
use tokio::io;
use tokio::task::AbortHandle;
use tokio_util::compat::{Compat, TokioAsyncReadCompatExt};

use crate::reactive::{Watched, WatchedChannel, WatchedExt as _};
use crate::state_machine::StateMachine;
use crate::util::time::MissionTime;

pub const LOGS_PATH: &str = "./logs/";

/// This is used to write csv values asynchronously
pub type TimeseriesWriter = AsyncSerializer<Compat<File>>;
/// This can be used to write values to a csv file in a synchronous interface,
/// but the writing still happens non-blocking and async.
pub struct SyncTimeseriesWriter<T>(tokio::sync::mpsc::UnboundedSender<T>);

impl<T: Clone + Send + serde::Serialize + 'static> SyncTimeseriesWriter<T> {
    /// Create [`SyncTimeseriesWriter`] which can be used to write values to a csv file in the `log_folder`
    /// for the measurement `name` with the columns `headers`.
    ///
    /// Additionally a column will be generated which reports the timestamp by fetching `clock`.
    pub fn new(
        log_folder: &Path,
        name: &str,
        headers: &[&str],
        initial_value: T,
        clock: impl Watched<MissionTime> + Send + 'static,
    ) -> Self {
        let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
        // Do some ugly cloning because lifetimes.
        let log_folder = log_folder.to_path_buf();
        let name = name.to_string();
        let headers = headers.iter().map(|&x| String::from(x)).collect::<Vec<_>>();
        tokio::spawn(async move {
            let headers = headers.iter().map(String::as_str).collect::<Vec<_>>();
            let writer = make_timeseries_writer(&log_folder, &name, &headers)
                .await
                .inspect_err(|e| {
                    tracing::error!(cause = %e, "Unable to make sync timeseries writer");
                })?;
            let watched = WatchedChannel::new(rx, initial_value);

            spawn_watchable_logger(writer, watched, clock);
            io::Result::Ok(())
        });
        Self(tx)
    }

    pub fn write_record(&self, val: T) {
        let _ = self.0.send(val);
    }
}

/// Creates a folder in [`LOGS_PATH`] with this name. It should already be unique, i.e. contain a timestamp.
pub async fn make_mission_log_folder(unique_name: &str) -> io::Result<PathBuf> {
    let mut path = PathBuf::from(LOGS_PATH);
    path.push(unique_name);
    tokio::fs::create_dir_all(&path).await?;

    Ok(path)
}

/// Makes a csv writer for a specific timeseries.
///
/// This can then be used to serialize tuples of the timestamp and value, e.g. with [`spawn_watchable_logger`].
///
/// Due to CSV serializer rules, using a generic type cannot work with generating headers, thus they need to be provided.
/// Only include the headers of the watched type, without the timestamp.
/// <https://docs.rs/csv-async/latest/csv_async/struct.AsyncSerializer.html#structs>.
pub async fn make_timeseries_writer(
    log_folder: &Path,
    name: &str,
    headers: &[&str],
) -> io::Result<TimeseriesWriter> {
    let mut path = PathBuf::from(log_folder);
    path.push(format!("timeseries-{name}.csv"));
    let file = tokio::fs::File::create(path).await?;
    let mut writer = csv_async::AsyncWriterBuilder::new()
        .has_headers(false)
        .create_serializer(file.compat());
    writer.serialize(("timestamp [unix ms]", headers)).await?;
    Ok(writer)
}

/// Spawns a task which continuously reads the newest update from `value_watched` and writes it, including the timestamp, to `writer`.
pub fn spawn_watchable_logger<T: serde::Serialize + Clone + Send>(
    mut writer: TimeseriesWriter,
    mut watched_value: impl Watched<T> + Send + 'static,
    clock: impl Watched<MissionTime> + Send + 'static,
) -> AbortHandle {
    tokio::spawn(async move {
        loop {
            let val = watched_value.next().await;
            let _ = writer
                .serialize((clock.current().as_ms_timestamp(), val))
                .await;
        }
    })
    .abort_handle()
}

impl<I, O, P> StateMachine<I, O, P> {
    /// Spawn a thread to log the current state to csv.
    pub async fn start_logging_state(
        &self,
        folder: &Path,
        clock: impl Watched<MissionTime> + Send + 'static,
    ) -> io::Result<()> {
        let watched = self.watch_state_path();
        spawn_watchable_logger(
            make_timeseries_writer(folder, "current_state", &["index [int]", "state [string]"])
                .await?,
            watched.map(|p| (p.len() - 1, p.last().map(ToString::to_string))),
            clock,
        );
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reactive::IntoWatched;
    use std::time::{Duration, SystemTime, UNIX_EPOCH};
    use watchable::Watchable;

    async fn make_test_log_folder(name: &str) -> PathBuf {
        let unique_name = format!(
            "{name}-{time}",
            time = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .expect("time moved backwards")
                .as_millis()
        );
        make_mission_log_folder(&unique_name)
            .await
            .expect("should be able to create log folder")
    }

    #[tokio::test]
    async fn test_spawn_watchable_logger_struct() {
        #[derive(serde::Serialize, Clone)]
        struct Pos {
            x: f32,
            y: f32,
        }

        let pos = Watchable::new(Pos { x: 0.0, y: 1.0 });
        let time = Watchable::new(MissionTime::from(Duration::from_millis(1000)));

        let folder = make_test_log_folder("test_spawn_watchable_logger_struct").await;

        let task = spawn_watchable_logger(
            make_timeseries_writer(&folder, "position", &["x [m]", "y [m]"])
                .await
                .expect("should be able to create timeseries writer"),
            pos.watched(),
            time.watched(),
        );

        tokio::time::sleep(Duration::from_millis(1)).await;
        pos.replace(Pos { x: 2.0, y: 1.0 });
        time.replace(Duration::from_millis(1100).into());
        tokio::time::sleep(Duration::from_millis(1)).await;
        pos.replace(Pos { x: 3.0, y: 1.0 });
        time.replace(Duration::from_millis(1200).into());
        tokio::time::sleep(Duration::from_millis(1)).await;
        pos.replace(Pos { x: 4.0, y: 1.0 });
        time.replace(Duration::from_millis(1300).into());

        tokio::time::sleep(Duration::from_millis(20)).await;
        assert!(!task.is_finished(), "Task should still run");
        println!("MANUALLY ASSERT FILES EXIST");
    }

    #[tokio::test]
    async fn test_spawn_watchable_logger_scalar() {
        let altitude = Watchable::new(1.0_f32);
        let time = Watchable::new(MissionTime::from(Duration::from_millis(1000)));

        let folder = make_test_log_folder("test").await;

        let task = spawn_watchable_logger(
            make_timeseries_writer(&folder, "altitude", &["altitude [m.ü.M.]"])
                .await
                .expect("should be able to create timeseries writer"),
            altitude.watched(),
            time.watched(),
        );

        altitude.replace(2.0);
        time.replace(Duration::from_millis(1100).into());
        tokio::time::sleep(Duration::from_millis(1)).await;
        altitude.replace(3.0);
        time.replace(Duration::from_millis(1200).into());
        tokio::time::sleep(Duration::from_millis(1)).await;
        altitude.replace(4.0);
        time.replace(Duration::from_millis(1300).into());
        tokio::time::sleep(Duration::from_millis(1)).await;

        tokio::time::sleep(Duration::from_millis(20)).await;
        assert!(!task.is_finished(), "Task should still run");
    }

    /// Test the [`SyncTimeseriesWriter`]
    #[tokio::test]
    async fn test_sync_timeseries_writer() {
        #[derive(serde::Serialize, Clone)]
        struct Pos {
            x: f32,
            y: f32,
        }

        let time = Watchable::new(MissionTime::from(Duration::from_millis(1000)));

        let folder = make_test_log_folder("test_sync_timeseries_writer").await;

        let writer = SyncTimeseriesWriter::new(
            &folder,
            "position",
            &["x [m]", "y [m]"],
            Pos { x: 0.0, y: 1.0 },
            time.watched(),
        );

        /// Ensure it doesn't use async
        #[allow(clippy::items_after_statements)]
        fn write_some_non_async(writer: &SyncTimeseriesWriter<Pos>) {
            // Write some records
            writer.write_record(Pos { x: 2.0, y: 1.0 });
            writer.write_record(Pos { x: 3.0, y: 1.0 });
            writer.write_record(Pos { x: 4.0, y: 1.0 });
        }
        write_some_non_async(&writer);

        // Give some time for the async task to process the records
        tokio::time::sleep(Duration::from_millis(10)).await;

        // Verify the contents of the CSV file
        let mut path = PathBuf::from(&folder);
        path.push("timeseries-position.csv");
        let csv = tokio::fs::read_to_string(path)
            .await
            .expect("file should exist");
        assert_eq!(
            csv,
            r"timestamp [unix ms],x [m],y [m]
1000,0.0,1.0
1000,2.0,1.0
1000,3.0,1.0
1000,4.0,1.0
"
        );
    }
}
