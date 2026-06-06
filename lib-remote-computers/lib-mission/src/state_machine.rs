use crate::{Error, Result};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

pub use crate::execution::AdditionalState;

/// The state machine for a mission.
pub struct StateMachine<I, O, P> {
    /// A reference to the initial state.
    pub(crate) initial_state: StateName,
    /// All the state logic blocks in the state machine.
    pub(crate) states: HashMap<StateName, BoxedState<I, O, P>>,

    /// The current position in the state graph: (path_index, current_state).
    /// `path_index` is the number of transitions taken so far (0 for initial state).
    /// *You must not write to this manually*! (it is exposed solely for broadcasting).
    pub current_state: watchable::Watchable<(u32, StateName)>,
}

impl<I, O, P> StateMachine<I, O, P> {
    /// Extracts the state names again after the fact.
    pub fn state_names(&self) -> HashSet<StateName> {
        self.states.keys().cloned().collect()
    }
}

/// The trait which the states of the state machine must implement.
#[async_trait::async_trait]
pub trait StateLogic {
    /// A struct with all the observable values. The "Input Tree".
    type Input;
    /// A struct with all the target output data. The "Output Tree".
    type Output;
    /// A struct with inter-state parameters.
    /// It is also used to track the state transition path.
    type Parameter;

    /// Performs the state logic to completion, then returns the next state.
    ///
    /// This must be able to be called multiple times without side-effects,
    /// therefore clone all mutable state or keep them in local variables.
    async fn run(
        &self,
        input: &Self::Input,
        output: &Self::Output,
        parameter: &AdditionalState<Self::Parameter>,
    ) -> StateName;
}

/// The type of a dynamically dispatched state logic instance.
pub type BoxedState<I, O, P = ()> = Box<dyn StateLogic<Input = I, Output = O, Parameter = P>>;

/// The names of states in a mission profile.
///
/// This is used to have a type-safer, but still dynamic way to reference the next state.
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct StateName(pub(crate) Arc<str>);

impl StateName {
    /// State names must have at most `MAX_LEN` bytes.
    /// This is necessary for Mavlink and really is almost no restriction.
    pub const MAX_LEN: usize = 30;
}

impl std::fmt::Display for StateName {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

impl StateName {
    /// Convert a string to a name if it exists.
    pub fn try_new(name: &str, names: &HashSet<StateName>) -> Result<Self> {
        names
            .get(name)
            .cloned()
            .ok_or(Error::UnknownStateName(name.into()))
    }

    /// Used to create the initial name set and for testing.
    /// *Do not use this in to construct the state machine*.
    pub fn new_unchecked(name: impl Into<Arc<str>>) -> Result<Self> {
        let name = name.into();
        if name.len() > Self::MAX_LEN {
            Err(Error::TooLongStateName(name))
        } else if name.contains('\0') {
            Err(Error::StateNameWithNullByte(name))
        } else {
            Ok(Self(name))
        }
    }
}

/// So that `HashSet::get` works.
impl std::borrow::Borrow<str> for StateName {
    fn borrow(&self) -> &str {
        &self.0
    }
}

#[cfg(test)]
mod tests {
    use crate::Error;

    use super::*;

    #[test]
    fn test_state_name_with_null_byte() {
        let name_with_null = "IDLE\0ARMED";
        let result = StateName::new_unchecked(name_with_null);

        assert!(matches!(result, Err(Error::StateNameWithNullByte(_))));
    }

    #[test]
    fn test_long_state() {
        let full_len = "A".repeat(StateName::MAX_LEN);
        StateName::new_unchecked(full_len).expect("should just about work");
    }

    #[test]
    fn too_long_state_name() {
        let too_long = "A".repeat(StateName::MAX_LEN + 1);
        let result = StateName::new_unchecked(too_long);
        assert!(matches!(result, Err(Error::TooLongStateName(_))));
    }
}
