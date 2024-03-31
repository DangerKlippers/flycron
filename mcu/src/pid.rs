use crate::{commands::CommandContext, dshot::ThrottleCommand};
use anchor::*;
use core::mem::transmute_copy;
use fugit::HertzU64;

use crate::clock::{Clock, Duration, Instant};
use control_law::PidGains;

pub const PID_RATE: u64 = 1000;
pub const PID_PERIOD: Duration = Duration::from_rate(HertzU64::Hz(PID_RATE));

pub fn next_pid_time() -> Instant {
    let now = Clock::now();
    // Round down to
    let remainder_ticks = now.ticks() % PID_PERIOD.ticks();
    // In theory, remainder_ticks would be zero if we ask for next time exactly on the target time.
    // In practice, we'd really already have missed the target in that case, so we'll just ignore
    // this case to save a few instructions on the comparison
    now + PID_PERIOD - Duration::from_ticks(remainder_ticks)
}

#[derive(Debug, defmt::Format)]
pub struct PidTimeIterator {
    next: Instant,
}

impl PidTimeIterator {
    pub fn new() -> PidTimeIterator {
        Self {
            next: next_pid_time(),
        }
    }

    pub fn advance(&mut self) -> Instant {
        self.next += PID_PERIOD;
        self.next
    }

    pub fn next(&mut self) -> Instant {
        loop {
            if self.next >= Clock::now() {
                return self.next;
            } else {
                self.advance();
            }
        }
    }
}

pub async fn pid_loop_task(cx: crate::app::pid_loop::Context<'_>) {
    let mut controller = control_law::Controller::new(400.0);
    let mut ticks = PidTimeIterator::new();
    let mut last_pos = 0u32;
    loop {
        let next_time = ticks.next();
        Clock::delay_until(next_time).await;
        if !cx
            .shared
            .pid_set_enable
            .load(portable_atomic::Ordering::Relaxed)
        {
            cx.shared.dshot_throttle.signal(ThrottleCommand::MotorsOff);
            continue;
        }
        if cx.shared.pid_gains.signaled() {
            // NOTE: Will return immediately because we checked for signal
            let gains = cx.shared.pid_gains.wait().await;
            controller.update_gains(0, &gains);
        }
        if cx.shared.filter_coefs.signaled() {
            let (a, b) = cx.shared.filter_coefs.wait().await;
            controller.update_filters(0, a, b);
        }

        let current_position = cx.shared.encoder.count();
        cx.shared
            .last_measured_position
            .store(current_position, portable_atomic::Ordering::SeqCst);
        // TODO: Replace with sampling from stepper emulation at the current(or next?) time step
        //
        let (c0, c1, c2) = cx.shared.target_queue.get_for_control(next_time);
        let target_position = if let Some((_, v0)) = c0 {
            last_pos = v0;
            v0
        } else {
            last_pos
        } as i32;
        cx.shared
            .last_commanded_position
            .store(target_position, portable_atomic::Ordering::SeqCst);

        let v0 = match (c0, c1) {
            (Some((t0, p0)), Some((t1, p1))) => {
                (p1.wrapping_sub(p0) as f32) / ((t1 - t0).ticks() as f32)
            }
            _ => 0.0,
        };
        let v1 = match (c1, c2) {
            (Some((t1, p1)), Some((t2, p2))) => {
                (p2.wrapping_sub(p1) as f32) / ((t2 - t1).ticks() as f32)
            }
            _ => 0.0,
        };
        let a0 = match (v0, v1, c1, c2) {
            (v0, v1, Some((t1, _)), Some((t2, _))) => (v1 - v0) / ((t2 - t1).ticks() as f32),
            _ => 0.0,
        };

        let throttle = controller
            .update(
                target_position,
                v0,
                a0,
                current_position,
                1.0 / (PID_RATE as f32),
            )
            .output;

        let scaled_throttle = throttle.clamp(0.0, 1.0) * ThrottleCommand::MAX as f32;
        // defmt::info!("Throttle: {}", scaled_throttle as u16);
        cx.shared
            .dshot_throttle
            .signal(ThrottleCommand::Throttle(scaled_throttle as u16));
    }
}

#[klipper_command]
pub fn pid_set_gains(
    context: &mut CommandContext,
    limit: u32,
    p: u32,
    p_max: u32,
    i: u32,
    i_max: u32,
    d: i32,
    d_max: u32,
) {
    let gains = PidGains {
        limit: unsafe { transmute_copy(&limit) },
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
    context
        .interfaces
        .pid_setpoint
        .store(setpoint as i32, portable_atomic::Ordering::SeqCst);
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

#[cfg(test)]
mod test {}
