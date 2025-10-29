use crate::state_machine::{AdditionalState, StateLogic, StateName};
use std::future::pending;
use std::marker::PhantomData;

/// This state will be pending forever and essentially block the state machine.
///
/// Use this to finish the mission.
#[derive(Debug)]
pub struct Shutdown<I, O, P>(PhantomData<(I, O, P)>);

impl<I, O, P> Default for Shutdown<I, O, P> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

#[async_trait::async_trait]
impl<I, O, P> StateLogic for Shutdown<I, O, P>
where
    I: Sync,
    O: Sync,
    P: Sync,
{
    type Input = I;
    type Output = O;
    type Parameter = P;

    /// pending forever, therefore this state will never exit
    #[tracing::instrument(name = "states::Shutdown::run", skip(self, _p), fields(state = %_p.current_state().0))]
    async fn run(
        &self,
        _: &Self::Input,
        _: &Self::Output,
        _p: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        tracing::info!(
            shutdown = true,
            "The state machine has reached a shutdown state and will not do anything again"
        );
        pending::<()>().await;
        unreachable!("Shutdown state should never exit");
    }
}
