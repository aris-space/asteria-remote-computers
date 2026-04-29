// use lib_mission::state_machine::{AdditionalState, StateLogic, StateName};
// 
// #[derive(Debug, serde::Deserialize, serde::Serialize)]
// pub enum PSSMode {
//     /// The default: The FC is passive and doesn't command PSS valves.
//     FCPassive,
//     /// The FC takes over PSS control.
//     FCActive,
// }
// 
// /// This state sets the mode with respect to the PSS control, then proceeds to `on_done`
// ///
// /// This is used to hands off control to the RFS during, and for taking it over again afterwards.
// #[derive(Debug)]
// pub struct SetPSSMode {
//     pub mode: PSSMode,
//     pub on_done: StateName,
// }
// 
// #[async_trait::async_trait]
// impl StateLogic for SetPSSMode {
//     type Input = super::Input;
//     type Output = super::Output;
//     type Parameter = super::Parameter;
// 
//     #[tracing::instrument(name = "states::SetPSSMode::run", skip_all, fields(?self, state = %param.current_state().0))]
//     async fn run(
//         &self,
//         input: &Self::Input,
//         output: &Self::Output,
//         param: &AdditionalState<Self::Parameter>,
//     ) -> StateName {
//         match self.mode {
//             PSSMode::FCPassive => {
//                 tracing::info!("Setting the FC to pss-passive mode");
//                 let prev = output.valves.set_passive();
//                 tracing::trace!(prev, "previous passive state");
//             }
//             PSSMode::FCActive => {
//                 tracing::info!("Setting the FC to pss-active mode for current configuration!");
//                 output.valves.set_active(&input.pss_valves).await;
//             }
//         }
// 
//         self.on_done.clone()
//     }
// }
