use fugit::HertzU64;

use crate::clock::{Clock, Duration, Instant};

pub const PID_PERIOD: Duration = Duration::from_rate(HertzU64::Hz(8000));

pub fn next_pid_time() -> Instant {
    let now = Clock::now();
    // Round down to
    let remainder = Duration::from_ticks(now.ticks() % PID_PERIOD.ticks());
    now + PID_PERIOD - remainder
}

pub fn next_pid_ticks() -> impl Iterator<Item = Instant> {
    core::iter::successors(Some(next_pid_time()), |prev| Some(*prev + PID_PERIOD))
}
