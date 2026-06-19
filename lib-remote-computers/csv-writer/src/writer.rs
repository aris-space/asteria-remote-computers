use std::time::SystemTime;

use datapoint_information::datapoint::DatapointHolder;
use datapoint_information::influx::{DataPoint, to_datapoint};
use facet::Facet;
use tokio::sync::mpsc;

pub struct LoggerHandle<M> {
    channel: mpsc::Sender<(M, SystemTime)>,
}

impl<M> LoggerHandle<M> {
    pub fn log_message(&self, msg: M) {
        let _ = self.channel.try_send((msg, SystemTime::now()));
        // FIXME: handle dropped messages
    }
}

pub struct LoggerActor<M> {
    channel: mpsc::Receiver<(M, SystemTime)>,
    sender: mpsc::Sender<(M, SystemTime)>,
}

impl<M> LoggerActor<M>
where
    M: DatapointHolder + for<'a> Facet<'a>,
{
    async fn handle_single(&mut self) {
        let Some((msg, time)) = self.channel.recv().await else {
            // FIXME: break out of something
            return;
        };

        let Ok(mut datapoint) = to_datapoint(&msg) else {
            // FIXME: instead log as json
            return;
        };
        datapoint.timestamp_ns = Some(
            time.duration_since(SystemTime::UNIX_EPOCH)
                .expect("time moved backwards")
                .as_nanos()
                .try_into()
                .expect("is fine for the next dozen years"),
        );
    }

    fn make_row() {}
}
