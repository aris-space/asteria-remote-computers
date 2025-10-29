use crate::state_machine::{BoxedState, StateMachine, StateName};
use crate::{Error, Result};
use std::convert::Infallible;
use watchable::Watchable;

impl<I, O, P> StateMachine<I, O, P> {
    /// Creates a future which runs the entire mission based on changes of the input.
    ///
    /// This currently never exists normally, but can return an error if some invariant was broken.
    #[tracing::instrument(name = "StateMachine::run", skip(self, input, output, param))]
    pub async fn run(self, input: &I, output: &O, param: P) -> Result<Infallible> {
        let mut state = self.get_state(&self.initial_state)?;
        let additional_state = AdditionalState::new(param, self.state_path.clone());

        loop {
            let next_state = state.run(input, output, &additional_state).await;
            tracing::info!(target = %next_state, index = additional_state.current_state().1 + 1, "state transition");
            additional_state.push_state_transition(next_state.clone());
            state = self.get_state(&next_state)?;
        }
    }

    /// A helper to lookup the state in the state map.
    /// Should always succeed if the state machine was constructed using [`StateMachine::new`].
    fn get_state(&self, name: &StateName) -> Result<&BoxedState<I, O, P>> {
        self.states.get(name).ok_or(Error::InvariantBroken(
            "`states` shall contain a value for every `StateName`".into(),
        ))
    }
}

/// All the internal state of the state machine.
pub struct AdditionalState<P> {
    /// The path through the state graph that has been taken.
    /// `state_path[0]` is always the initial state, and `state_path.last()` the current state.
    state_path: Watchable<Vec<StateName>>,
    /// Other parameters of the mission.
    pub parameters: P,
}

impl<P> AdditionalState<P> {
    /// Gets the state path by cloning it. Prefer [`Self::current_state`] if you only need the current state and length.
    pub fn clone_state_path(&self) -> Vec<StateName> {
        self.state_path.get()
    }

    /// Returns both the current state as well as the index in the state path,
    /// which corresponds to the number of transitions taken so far.
    pub fn current_state(&self) -> (StateName, usize) {
        let p = self.state_path.read();
        let current = p
            .last()
            .expect("state_path is initialized with one element and never shortened")
            .clone();
        (current, p.len() - 1)
    }

    /// Create the state and make sure we initialize the path with the inital element.
    fn new(parameters: P, state_path: Watchable<Vec<StateName>>) -> Self {
        debug_assert_eq!(
            state_path.get().len(),
            1,
            "initialize AdditionalState with non-singleton state path"
        );
        Self {
            state_path,
            parameters,
        }
    }

    /// Appends a state to the path.
    fn push_state_transition(&self, state_name: StateName) {
        self.state_path.write().push(state_name);
    }
}
