use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum PSSMode {
    /// The default: The RFS is passive and doesn't command PSS valves.
    RFSPassive,
    /// The RFS takes over PSS control.
    RFSActive,
}

/// This state sets the mode with respect to the PSS control, then proceeds to `on_done`
///
/// This is used to take over control, revert this takeover, and for doing a proper handsoff.
#[derive(Debug)]
pub struct SetPSSMode {
    pub mode: PSSMode,
    pub on_done: StateName,
}

#[async_trait::async_trait]
impl StateLogic for SetPSSMode {
    type Input = super::Input;
    type Output = super::Output;
    type Parameter = super::Parameter;

    #[tracing::instrument(name = "states::SetPSSMode::run", skip_all, fields(?self, state = %param.current_state().0))]
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        match self.mode {
            PSSMode::RFSPassive => {
                tracing::info!("Setting the RFS to pss-passive mode");
                let prev = output.pss_valves.set_passive();
                tracing::trace!(prev, "previous passive state");
            }
            PSSMode::RFSActive => {
                tracing::info!("Setting the RFS to pss-active mode for current configuration!");
                output.pss_valves.set_active(&input.pss_valves).await;
            }
        }

        self.on_done.clone()
    }
}
