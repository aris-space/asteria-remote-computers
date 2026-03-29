//! TODO: this can be removed once OutChannels are used

impl crate::output::PSSValves {
    pub fn spawn_output_drivers_fc(
        &self,
        _tx: &std::sync::mpsc::Sender<messages::Message>,
    ) -> [tokio::task::AbortHandle; 7] {
        todo!(
            "Remove this method, can be replaced with PSSValves::new_for_fc intializing OutChannels"
        )
    }
    pub fn spawn_output_drivers_rfs(
        &self,
        _tx: &std::sync::mpsc::Sender<messages::Message>,
    ) -> [tokio::task::AbortHandle; 7] {
        todo!(
            "Remove this method, can be replaced with PSSValves::new_for_rfs intializing OutChannels"
        )
    }
}
