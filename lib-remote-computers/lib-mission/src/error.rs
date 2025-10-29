use std::sync::Arc;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("State `{0}` isn't defined")]
    UnknownStateName(Arc<str>),
    #[error("State `{0}` has more than the allowed 30 bytes")]
    TooLongStateName(Arc<str>),
    #[error("State `{0}` contains a null byte")]
    StateNameWithNullByte(Arc<str>),
    #[error("Received state (~`{0}`) is invalid utf-8")]
    StateNameInvalidUTF8(String),
    #[error("Invariant `{0}` was broken")]
    InvariantBroken(Arc<str>),
    #[error("Config is malformed: {0}")]
    ConfigMalformed(Box<dyn std::error::Error>),
}

impl Error {
    pub fn malformed_config(e: impl Into<Box<dyn std::error::Error>>) -> Self {
        Self::ConfigMalformed(e.into())
    }
}
