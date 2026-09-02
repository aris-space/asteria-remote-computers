//! This module serves to abstract the fine grained control of actuators over telemetry.

use std::collections::{BTreeMap, BTreeSet};
use std::str::FromStr;

use datatypes::actuator::Settable;

use crate::reactive::{IntoWatched, Watched as _};

/// A valid representation of the mavlink command sent.
/// This is generic over `ActuatorName`, so users can have their own careful set of actuators.
#[derive(Debug, Clone, PartialEq)]
pub struct Command<ActuatorName> {
    /// In which state this command should work in.
    /// Is equal to the amount of transitions performed so far.
    pub in_state_index: usize,
    /// The actuator to be changed.
    pub actuator: ActuatorName,
    /// The target value of the actuator.
    /// Depends on the actuator, but for valves, this is the openness,
    /// 0 meaning fully closed, 1 meaning fully open. Binary valves only allow exactly these two values.
    pub value: f32,
}

#[derive(Debug)]
pub struct ActuatorControl<ActuatorName> {
    /// The actuators that are allowed to be activated with this instance.
    allowed_actuators: BTreeSet<ActuatorName>,
}

impl<ActuatorName> ActuatorControl<ActuatorName> {
    /// Create a new instance that allows the given actuators to be controlled.
    pub fn new(allowed_actuators: impl Into<BTreeSet<ActuatorName>>) -> Self {
        Self {
            allowed_actuators: allowed_actuators.into(),
        }
    }
}

impl<ActuatorName> ActuatorControl<ActuatorName>
where
    ActuatorName: FromStr + Ord,
{
    /// Parses the actuator names and returns an instance to be used for all passed actuators.
    /// Returns an error if any cannot be parsed, thus are not in the valid domain.
    pub fn from_strings<'a>(
        allowed_actuators: impl IntoIterator<Item = &'a str>,
    ) -> Result<Self, ActuatorName::Err> {
        let allowed_actuators = allowed_actuators
            .into_iter()
            .map(|s| ActuatorName::from_str(s))
            .collect::<Result<BTreeSet<ActuatorName>, _>>()?;
        Ok(Self::new(allowed_actuators))
    }
}

/// A type which can't be constructed and as such a function which returns this cannot return.
#[derive(Debug)]
pub enum NeverReturns {}

impl<ActuatorName> ActuatorControl<ActuatorName>
where
    ActuatorName: std::fmt::Debug + Clone + Send + Ord,
{
    /// Acts upon commands in an infinite loop.
    /// This is cancellation safe in a sense that each command will either be executed or not.
    pub async fn act(
        &self,
        currect_state_index: usize,
        cmd: impl IntoWatched<Option<Command<ActuatorName>>> + Send,
        actuators: BTreeMap<ActuatorName, Box<dyn Settable + Send>>,
    ) -> NeverReturns {
        self.act_optional(Some(currect_state_index), true, cmd, actuators)
            .await
    }

    /// Acts upon commands in an infinite loop.
    /// This is cancellation safe in a sense that each command will either be executed or not.
    ///
    /// Unlike [`Self::act`], this can allow commands in every state, which is only really useful for global actuators.
    #[tracing::instrument(level = "TRACE", skip_all, fields(?self, currect_state_index = currect_state_index))]
    pub async fn act_optional(
        &self,
        currect_state_index: Option<usize>,
        warn_unknown: bool,
        cmd: impl IntoWatched<Option<Command<ActuatorName>>> + Send,
        mut actuators: BTreeMap<ActuatorName, Box<dyn Settable + Send>>,
    ) -> NeverReturns {
        let mut watcher = cmd.watched();

        loop {
            let cmd = watcher.next().await;
            if let Some(cmd) = cmd {
                self.act_on(cmd, currect_state_index, warn_unknown, &mut actuators);
            }
        }
    }

    /// Acts upon a single actuator command by setting the corresponding `Settable` in `actuators` if allowed.
    /// Otherwise logs warnings.
    #[tracing::instrument(level = "TRACE", skip_all, fields(command = ?cmd))]
    fn act_on(
        &self,
        cmd: Command<ActuatorName>,
        currect_state_index: Option<usize>,
        warn_unknown: bool,
        actuators: &mut BTreeMap<ActuatorName, Box<dyn Settable + Send>>,
    ) {
        let Command {
            in_state_index,
            actuator,
            value,
        } = cmd;
        if currect_state_index.is_some_and(|currect_state| in_state_index != currect_state) {
            tracing::warn!("received old command");
            return;
        }
        if !self.allowed_actuators.contains(&actuator) {
            if warn_unknown {
                tracing::warn!("received command for unconfigured actuator");
            }
            return;
        }
        let Some(settable) = actuators.get_mut(&actuator) else {
            tracing::error!("allowed actuator not in the acutators list");
            return;
        };
        debug_assert!(
            currect_state_index.is_none_or(|c| c == in_state_index)
                && self.allowed_actuators.contains(&actuator),
            "asserted by control flow above"
        );

        if settable.set_value(value).is_ok() {
            tracing::info!("set actuator {actuator:?} to {value}");
        } else {
            tracing::warn!("out of range value for the actuator");
        }
    }
}
