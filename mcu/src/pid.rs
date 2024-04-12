use crate::{
    clock::{Clock, Duration, Instant, CLOCK_FREQ},
    commands::CommandContext,
    dshot::ThrottleCommand,
};
use anchor::*;
use control_law::{Controller, PidGains};
use core::mem::transmute_copy;
use fugit::HertzU64;

pub enum ModelParam {
    Mass(f32),
}

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
    let mut controller = Controller::new(400.0);
    let mut ticks = PidTimeIterator::new();
    loop {
        let next_time = ticks.next();

        Clock::delay_until(next_time).await;
        while let Ok((target, gains)) = cx.shared.pid_gains.try_receive() {
            defmt::info!("update gains {} {}", target, gains);
            controller.update_gains(target as usize, &gains);
        }
        while let Ok((target, a, b)) = cx.shared.filter_coefs.try_receive() {
            defmt::info!("update filter {} {} {}", target, a, b);
            controller.update_filters(target as usize, a, b);
        }
        while let Ok((target, rising, falling)) = cx.shared.slew_rate_limits.try_receive() {
            defmt::info!("update slew rate {} {} {}", target, rising, falling);
            controller.update_slew_rate(target as usize, rising, falling);
        }
        while let Ok(param) = cx.shared.model_params.try_receive() {
            match param {
                ModelParam::Mass(mass) => {
                    defmt::info!("update mass {}", mass);
                    controller.set_mass(mass);
                }
            }
        }

        if cx.shared.encoder_override.signaled() {
            let target = cx.shared.encoder_override.wait().await;
            cx.shared.encoder.set_value(target);
        }

        let current_position = cx.shared.encoder.count();
        cx.shared
            .last_measured_position
            .store(current_position, portable_atomic::Ordering::SeqCst);

        let (target_position, c1, c2) = cx.shared.target_queue.get_for_control(next_time);
        cx.shared
            .last_commanded_position
            .store(target_position, portable_atomic::Ordering::SeqCst);

        let v0 = match c1 {
            Some((t1, p1)) => {
                (((p1 as i32) - (target_position)) as f32)
                    / ((t1 - next_time).ticks() as f32 / (CLOCK_FREQ as f32))
            }
            _ => 0.0,
        };
        let v1 = match (c1, c2) {
            (Some((t1, p1)), Some((t2, p2))) => {
                (((p2 as i32) - (p1 as i32)) as f32)
                    / ((t2 - t1).ticks() as f32 / (CLOCK_FREQ as f32))
            }
            _ => 0.0,
        };
        let a0 = match (v0, v1, c1, c2) {
            (v0, v1, Some((t1, _)), Some((t2, _))) => {
                (v1 - v0) / ((t2 - t1).ticks() as f32 / (CLOCK_FREQ as f32))
            }
            _ => 0.0,
        };

        let output = controller.update(
            target_position,
            v0,
            a0,
            current_position,
            1.0 / (PID_RATE as f32),
        );
        let throttle = output.output;

        if !cx
            .shared
            .pid_set_enable
            .load(portable_atomic::Ordering::Relaxed)
        {
            cx.shared.dshot_throttle.signal(ThrottleCommand::MotorsOff);
            cx.shared
                .last_throttle
                .store(0.0, portable_atomic::Ordering::SeqCst);
            continue;
        }

        let scaled_throttle = throttle.clamp(
            cx.shared
                .throttle_limits
                .0
                .load(portable_atomic::Ordering::SeqCst),
            cx.shared
                .throttle_limits
                .1
                .load(portable_atomic::Ordering::SeqCst),
        );
        cx.shared
            .last_throttle
            .store(scaled_throttle, portable_atomic::Ordering::SeqCst);
        cx.shared.dshot_throttle.signal(ThrottleCommand::Throttle(
            (scaled_throttle * ThrottleCommand::MAX as f32) as u16,
        ));
    }
}

#[klipper_command]
pub fn pid_set_gains(
    context: &mut CommandContext,
    target: u32,
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
    let _ = context.interfaces.pid_gains.try_send((target as u8, gains));
}

#[klipper_command]
pub fn pid_set_coefs(context: &mut CommandContext, target: u32, alpha: u32, beta: u32) {
    let _ = context.interfaces.filter_coefs.try_send((
        target as u8,
        unsafe { transmute_copy(&alpha) },
        unsafe { transmute_copy(&beta) },
    ));
}

#[klipper_command]
pub fn pid_set_slew_limits(
    context: &mut CommandContext,
    target: u32,
    limit_rising: u32,
    limit_falling: u32,
) {
    let _ = context.interfaces.slew_rate_limits.try_send((
        target as u8,
        unsafe { transmute_copy(&limit_rising) },
        unsafe { transmute_copy(&limit_falling) },
    ));
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

#[klipper_command]
pub fn pid_set_mass(context: &mut CommandContext, mass_grams: u32) {
    let _ = context
        .interfaces
        .model_params
        .try_send(ModelParam::Mass(unsafe { transmute_copy(&mass_grams) }));
}

#[klipper_command]
pub fn pid_set_throttle_limits(context: &mut CommandContext, min: u32, max: u32) {
    let min: f32 = unsafe { transmute_copy(&min) };
    let max: f32 = unsafe { transmute_copy(&max) };
    context
        .interfaces
        .throttle_limits
        .0
        .store(min, portable_atomic::Ordering::SeqCst);
    context
        .interfaces
        .throttle_limits
        .1
        .store(max, portable_atomic::Ordering::SeqCst);
}

#[klipper_command]
pub fn pid_get_dump(context: &mut CommandContext) {
    let throttle = unsafe {
        transmute_copy(
            &context
                .interfaces
                .pid_last_throttle
                .load(portable_atomic::Ordering::SeqCst),
        )
    };
    klipper_reply!(pid_dump, throttle: u32 = throttle);
}
