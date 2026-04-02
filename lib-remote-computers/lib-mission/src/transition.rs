//! This module provides the implementation state logic to listen for manual transition commands.
//!
//! The idea is that a transition command specifies a target state and optionally a precondition
//! in terms of the index of the transition in the transitions path.
//! If the preconition is specified, it is called a `validated` transitions and it will be used
//! for planned mandatory transitions, to ensure consistency.
//! If it isn't, these `unvalidated` transitions can be used to abort to an abort scenario,
//! independently of whether some other transitions have triggered in the meantime.

use datatypes::status::ManualStateTransition;

use crate::reactive::{IntoWatched, Watched};
use crate::state_machine::{AdditionalState, StateName};

/// Either the command is `None`, wheen there is no command,
/// or `Some((precondition, target))`, as explained in [`crate::transition`].
pub type Command = Option<(Option<usize>, StateName)>;

/// This trait can watch the uplink command.
///
/// Either the command is `None`, wheen there is no command,
/// or `Some((precondition, target))`, as explained in [`crate::transition`].
pub trait GetTransitionCommand {
    /// Implementations must ensure this Watched is "reset", i.e. the next read will instantly return.
    fn transition_cmd_watcher(&self) -> impl IntoWatched<ManualStateTransition> + Send;
}

#[derive(Debug, Default)]
pub struct ManualTransitions {
    /// The targets of transitions that can be done without a precondition.
    unvalidated_transitions: Vec<StateName>,
    /// The targets of transitions that can be done with a precondition (including the unvalidated ones).
    all_transitions: Vec<StateName>,
}

impl ManualTransitions {
    /// Create a new transitions object in testing. During config parsing, this will be called by the library.
    pub fn new(unvalidated_transitions: Vec<StateName>, all_transitions: Vec<StateName>) -> Self {
        ManualTransitions {
            unvalidated_transitions,
            all_transitions,
        }
    }

    /// Returns true if the this object contains configuration to allow transitions (at all) to `target`.
    pub fn is_allowed_transition(&self, target: &StateName) -> bool {
        self.all_transitions.contains(target)
    }

    /// Returns true only if the objects contains configuration to allow *unvalidated* transitions to `target`.
    pub fn is_allowed_unvalidated_transition(&self, target: &StateName) -> bool {
        self.unvalidated_transitions.contains(target)
    }

    /// Will wait for a manual transitions command and return the transition when it occurs.
    ///
    /// This should be used in the `select` loop.
    #[tracing::instrument(level = "TRACE", name = "ManualTransitions::next", skip(cmd, path), fields(current_state = ?path.current_state()))]
    pub async fn next<P>(
        &self,
        cmd: &impl GetTransitionCommand,
        path: &AdditionalState<P>,
    ) -> StateName {
        let mut watcher = cmd.transition_cmd_watcher().watched();

        let next_index = (path.current_state().1 + 1) as usize;

        loop {
            let ManualStateTransition {
                transition_index,
                state,
            } = watcher.next().await;
            let Ok(target) = StateName::try_from_array(&state, &path.states) else {
                tracing::warn!("received invalid state name in command");
                continue;
            };
            let index = if transition_index == 0 {
                None
            } else {
                Some(transition_index as usize)
            };

            match index {
                None => {
                    if self.is_allowed_unvalidated_transition(&target) {
                        tracing::info!(%target, "Manual unvalidated transition");
                        return target;
                    }
                    tracing::warn!("attempted unvalidated transition to unconfigured state");
                }
                Some(idx) => {
                    if self.is_allowed_transition(&target) && idx == next_index {
                        tracing::info!(%target, "Manual validated transition");
                        return target;
                    } else if idx == next_index {
                        tracing::warn!("attempted validated transition to unconfigured state");
                    } else if idx >= next_index {
                        tracing::warn!("attempted validated transition with a future index");
                    } else {
                        // ignore old transitions
                    }
                }
            }
        }
    }
}
