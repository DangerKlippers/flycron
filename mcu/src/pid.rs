use crate::commands::CommandContext;
use anchor::*;
use core::mem::transmute_copy;
use fugit::HertzU64;

use crate::clock::{Clock, Duration, Instant};

pub const PID_PERIOD: Duration = Duration::from_rate(HertzU64::Hz(8000));

pub fn next_pid_time() -> Instant {
    let now = Clock::now();
    // Round down to
    let remainder_ticks = now.ticks() % PID_PERIOD.ticks();
    // In theory, remainder_ticks would be zero if we ask for next time exactly on the target time.
    // In practice, we'd really already have missed the target in that case, so we'll just ignore
    // this case to save a few instructions on the comparison
    now + PID_PERIOD - Duration::from_ticks(remainder_ticks)
}

pub fn next_pid_times() -> impl Iterator<Item = Instant> {
    core::iter::successors(Some(next_pid_time()), |prev| {
        let mut next = *prev;
        loop {
            next += PID_PERIOD;
            if next >= Clock::now() {
                return Some(next);
            }
        }
    })
}

#[derive(Default)]
pub struct PidGains {
    pub p: f32,
    pub p_max: f32,
    pub i: f32,
    pub i_max: f32,
    pub d: f32,
    pub d_max: f32,
}

#[klipper_command]
pub fn pid_set_gains(
    context: &mut CommandContext,
    p: u32,
    p_max: u32,
    i: u32,
    i_max: u32,
    d: i32,
    d_max: u32,
) {
    let gains = PidGains {
        p: unsafe { transmute_copy(&p) },
        p_max: unsafe { transmute_copy(&p_max) },
        i: unsafe { transmute_copy(&i) },
        i_max: unsafe { transmute_copy(&i_max) },
        d: unsafe { transmute_copy(&d) },
        d_max: unsafe { transmute_copy(&d_max) },
    };
    context.interfaces.pid_gains.signal(gains);
}

#[klipper_command]
pub fn pid_set_setpoint(context: &mut CommandContext, setpoint: u32) {
    let value: f32 = unsafe { transmute_copy(&setpoint) };
    context
        .interfaces
        .pid_setpoint
        .store(value, portable_atomic::Ordering::SeqCst);
}
#[klipper_command]
pub fn pid_set_enable(context: &mut CommandContext, enable: bool) {
    defmt::info!("Setting pid enable to {}", enable);
    let value: bool = unsafe { transmute_copy(&enable) };
    context
        .interfaces
        .pid_set_enable
        .store(value, portable_atomic::Ordering::SeqCst);
}
