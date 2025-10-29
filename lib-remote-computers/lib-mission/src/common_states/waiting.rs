use crate::state_machine::{AdditionalState, StateLogic, StateName};
use crate::transition::{GetTransitionCommand, ManualTransitions};
use std::marker::PhantomData;

/// This state will do nothing but wait for manual state transitions.
///
/// `I` must implement [`GetTransitionCommand`].
#[derive(Debug)]
pub struct Waiting<I, O, P> {
    transitions: ManualTransitions,
    _generic_capture: PhantomData<(I, O, P)>,
}

impl<I, O, P> Waiting<I, O, P> {
    pub fn new(transitions: ManualTransitions) -> Self {
        Self {
            transitions,
            _generic_capture: PhantomData,
        }
    }
}
#[async_trait::async_trait]
impl<I, O, P> StateLogic for Waiting<I, O, P>
where
    I: GetTransitionCommand + Sync,
    O: Sync,
    P: Sync,
{
    type Input = I;
    type Output = O;
    type Parameter = P;

    #[tracing::instrument(
        name = "states::Waiting::run",
        skip(self, input, param),
        fields(transitions = ?self.transitions, state = %param.current_state().0)
    )]
    async fn run(
        &self,
        input: &Self::Input,
        _: &Self::Output,
        param: &AdditionalState<Self::Parameter>,
    ) -> StateName {
        return self.transitions.next(input, param).await;
    }
}
