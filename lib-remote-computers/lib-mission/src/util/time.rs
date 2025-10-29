use std::time::Duration;

/// A unix timestamp.
#[derive(Default, Debug, Clone, Copy, PartialOrd, Ord, PartialEq, Eq)]
pub struct MissionTime(Duration);

impl MissionTime {
    pub fn as_ms_timestamp(&self) -> u64 {
        self.0
            .as_millis()
            .try_into()
            .expect("this will take millions of years")
    }
}

impl From<MissionTime> for Duration {
    fn from(value: MissionTime) -> Self {
        value.0
    }
}

impl From<Duration> for MissionTime {
    fn from(value: Duration) -> Self {
        Self(value)
    }
}

impl std::ops::Add<Duration> for MissionTime {
    type Output = MissionTime;

    fn add(self, rhs: Duration) -> Self::Output {
        MissionTime(self.0 + rhs)
    }
}

impl std::ops::AddAssign<Duration> for MissionTime {
    fn add_assign(&mut self, rhs: Duration) {
        self.0 += rhs;
    }
}
