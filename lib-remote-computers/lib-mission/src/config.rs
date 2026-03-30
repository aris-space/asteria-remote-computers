use crate::Result;
use crate::state_machine::{BoxedState, StateMachine};
use crate::state_machine::{StateLogic, StateName};
use crate::transition::ManualTransitions;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

/// This struct defines the schema of a mission profile configuration file.
///
/// A mission profile could be for example the flight mission, a drop test, or a recreation of an old flight.
/// The heart of the profile is the state machine, given by a map of state names to their configuration.
/// The actual states available are generic and given by `M`, a [`MakeState`] implementor.
///
/// # Example
/// ```toml
/// name = "Drogue Out"
///
/// initial_state = "Init"
///
/// [states."Init"]
/// type = "Initial"
/// manual_transitions = ["Control Loop"]
/// on_idk = "Some Other State"
/// threshold = 20.0
///
/// [states."Control Loop"]
/// type = "ControlLoop"
/// period_ms = 50
/// on_landing = "Awaiting Recovery"
///
/// [states."Some Other State"]
/// type = "ControlLoop"
/// period_ms = 10
/// on_landing = "Awaiting Recovery"
///
/// [states."Awaiting Recovery"]
/// type = "Shutdown"
/// ```
#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub struct MissionProfile<M> {
    /// The name of the mission, used only for logging.
    pub name: String,
    /// The state name, i.e. the key in the `states` map, of the initial state.
    pub initial_state: String,
    /// All states in the state machine.
    pub states: HashMap<String, MissionState<M>>,
}

/// A thing that can make a state.
///
/// This should be implemented by an enum which contains all possible states with their configuration.
pub trait MakeState {
    /// The actual state type. This should be `BoxedState<I, O, P>`.
    type State;

    /// Builds the state given additional information about the state machine.
    fn make(
        self,
        names: &HashSet<StateName>,
        transitions: ManualTransitions,
    ) -> Result<Self::State>;
}

/// Defines a state of the mission profile.
///
/// Contains some common information, and the kind which determines the logic block to use.
#[derive(Debug, Serialize, Deserialize)]
pub struct MissionState<M> {
    /// The names of states this state can be manually transitioned into.
    ///
    /// Defining the same target more than once isn't caught and the abort-ness is logically OR'ed. Don't do this.
    #[serde(default)]
    pub manual_transitions: Vec<SingleTransition>,
    /// The configuration of the state logic.
    ///
    /// This is deserialized flattened & should be tagged with a `type` field, i.e. all kind fields are fields of this map.
    #[serde(flatten)]
    pub kind: M,
}

impl<M, S> MissionState<M>
where
    M: MakeState<State = S>,
{
    /// Creates a state (`S: BoxedState` will be needed) from the common information and the kind.
    ///
    /// Returns the first error only.
    fn make(self, names: &HashSet<StateName>) -> Result<S> {
        // first, collect the transitions
        let mut all_transitions = vec![];
        let mut unvalidated_transitions = vec![];
        for s in self.manual_transitions {
            match s {
                SingleTransition::Default(target)
                | SingleTransition::Full {
                    target,
                    abort: false,
                    display: _,
                } => {
                    let s = StateName::try_new(&target, names)?;
                    all_transitions.push(s);
                }
                SingleTransition::Full {
                    target,
                    abort: true,
                    display: _,
                } => {
                    let s = StateName::try_new(&target, names)?;
                    all_transitions.push(s.clone());
                    unvalidated_transitions.push(s);
                }
            }
        }
        let transitions = ManualTransitions::new(unvalidated_transitions, all_transitions);

        // then let the generic inner do its thing
        self.kind.make(names, transitions)
    }
}

/// A deserializeable entry in the transitions array.
/// Can either be deserializaized from a String, which then corresponds to a normal transition,
/// or contain metadata, i.e. whether this is an abort transition.
#[derive(Debug, serde::Deserialize, serde::Serialize)]
#[serde(untagged)]
pub enum SingleTransition {
    /// A transition to the state named according to the inner value with default metadata, thus `abort = false`.
    Default(String),
    /// A full transition description.
    Full {
        /// The name of the target state.
        target: String,
        /// If true, then the index in the command isn't validated, i.e. doesn't have to match.
        /// This is used for abort scenarios, where we want to abort to this state independently of whether the state machine has moved on.
        /// It also signifies to the ground station that this is the transition to take in case something seems wrong.
        #[serde(default)]
        abort: bool,
        /// An optional, non-functional name of this transition. This is purely for the ground station.
        display: Option<String>,
    },
}

impl<I, O, P> StateMachine<I, O, P> {
    /// Builds and validates the state machine from the config.
    pub fn new<M>(config: MissionProfile<M>) -> Result<Self>
    where
        M: MakeState<State = BoxedState<I, O, P>>,
    {
        let state_names = config
            .states
            .keys()
            .map(|s| StateName::new_unchecked(s.as_str()))
            .collect::<Result<HashSet<StateName>>>()?;

        let states = config
            .states
            .into_iter()
            .map(|(name, config)| {
                let name =
                    StateName::try_new(&name, &state_names).expect("generated above from same set");
                let logic = config.make(&state_names)?;
                Ok((name, logic))
            })
            .collect::<Result<HashMap<StateName, BoxedState<I, O, P>>>>()?;

        let initial_state = StateName::try_new(&config.initial_state, &state_names)?;

        Ok(Self {
            current_state: watchable::Watchable::new((0, initial_state.clone())),
            initial_state,
            states,
        })
    }
}

/// A helper trait which is implemented for `T: StateLogic`, to allow easy boxing and type erasure of a state logic block.
pub trait BoxableState<I, O, P> {
    /// Type erases and boxes `self`.
    fn into_box(self) -> BoxedState<I, O, P>;
}

impl<T, I, O, P> BoxableState<I, O, P> for T
where
    T: StateLogic<Input = I, Output = O, Parameter = P> + 'static,
{
    fn into_box(self) -> BoxedState<I, O, P> {
        Box::new(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Logic<I, O, P> {
        _phantom: std::marker::PhantomData<(I, O, P)>,
    }

    #[async_trait::async_trait]
    impl<I, O, P> StateLogic for Logic<I, O, P>
    where
        I: Sync,
        O: Sync,
        P: Sync,
    {
        type Input = I;
        type Output = O;
        type Parameter = P;

        async fn run(
            &self,
            _: &Self::Input,
            _: &Self::Output,
            _: &crate::execution::AdditionalState<Self::Parameter>,
        ) -> StateName {
            unreachable!("Will never be executed");
        }
    }

    #[derive(Debug, serde::Serialize, serde::Deserialize)]
    #[serde(tag = "type")]
    enum MissionStateKind {
        A,
        B,
        C { x: f32, y: u8 },
    }

    impl super::MakeState for MissionStateKind {
        type State = crate::state_machine::BoxedState<(), (), ()>;

        fn make(
            self,
            names: &HashSet<StateName>,
            transitions: ManualTransitions,
        ) -> crate::Result<Self::State> {
            let s1 = StateName::try_new("1", names).expect("defined in this scenario");
            let s2 = StateName::try_new("2", names).expect("defined in this scenario");
            let s3 = StateName::try_new("3", names).expect("defined in this scenario");
            match self {
                MissionStateKind::A => {
                    assert!(transitions.is_allowed_transition(&s1));
                    assert!(!transitions.is_allowed_unvalidated_transition(&s1));
                    assert!(transitions.is_allowed_transition(&s2));
                    assert!(!transitions.is_allowed_unvalidated_transition(&s2));
                    assert!(transitions.is_allowed_transition(&s3));
                    assert!(transitions.is_allowed_unvalidated_transition(&s3));
                }
                MissionStateKind::B => {
                    assert!(!transitions.is_allowed_transition(&s1));
                    assert!(!transitions.is_allowed_unvalidated_transition(&s1));
                    assert!(!transitions.is_allowed_transition(&s2));
                    assert!(!transitions.is_allowed_unvalidated_transition(&s2));
                    assert!(transitions.is_allowed_transition(&s3));
                    assert!(transitions.is_allowed_unvalidated_transition(&s3));
                }
                #[allow(clippy::float_cmp)]
                MissionStateKind::C { x, y } => {
                    assert!(!transitions.is_allowed_transition(&s1));
                    assert!(!transitions.is_allowed_unvalidated_transition(&s1));
                    assert!(!transitions.is_allowed_transition(&s2));
                    assert!(!transitions.is_allowed_unvalidated_transition(&s2));
                    assert!(!transitions.is_allowed_transition(&s3));
                    assert!(!transitions.is_allowed_unvalidated_transition(&s3));
                    assert_eq!(x, 1.125);
                    assert_eq!(y, 8);
                }
            }
            Ok(Logic {
                _phantom: std::marker::PhantomData,
            }
            .into_box())
        }
    }

    #[test]
    fn test_abort_transition() {
        let cfg = r#"
            name = "Abort"
            initial_state = "1"

            [states."1"]
            type = "A"
            manual_transitions = ["1", { target = "2", abort = false }, { target = "3", abort = true } ]

            [states."2"]
            type = "B"
            manual_transitions = [{ target = "3", abort = true }, { target = "3", abort = false }]

            [states."3"]
            type = "C"
            x = 1.125
            y = 8
        "#;

        let x =
            toml::from_str::<MissionProfile<MissionStateKind>>(cfg).expect("config should be good");
        // this will trigger the asserts above
        let _sm = StateMachine::new(x).expect("config should be semantically good as well");
    }

    #[test]
    fn test_named_transitions() {
        let cfg = r#"
            name = "Abort"
            initial_state = "1"

            [states."1"]
            type = "A"
            manual_transitions = ["1", { target = "2", display = "1-2", abort = false }, { target = "3", abort = true } ]

            [states."2"]
            type = "B"
            manual_transitions = [{ target = "3", abort = true }, { target = "3", display = "2 to 3" }]

            [states."3"]
            type = "C"
            x = 1.125
            y = 8
        "#;

        let x =
            toml::from_str::<MissionProfile<MissionStateKind>>(cfg).expect("config should be good");
        // this will trigger the asserts above
        let _sm = StateMachine::new(x).expect("config should be semantically good as well");
    }
}
