use core::cell::RefCell;

use crate::{
    clock::{Clock, Instant},
    commands::CommandContext,
    pid::{self, PidTimeIterator},
};
use anchor::*;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex as BlockingMutex};
use heapless::Deque;
use rtic::Mutex;
use stepperemu::Direction;

pub type EmulatedStepper = stepperemu::EmulatedStepper<PidTimeIterator>;

impl stepperemu::PidTimeIterator for pid::PidTimeIterator {
    fn next(&mut self) -> stepperemu::Instant {
        stepperemu::Instant::from_ticks(self.next().ticks() as u32)
    }

    fn advance(&mut self) -> stepperemu::Instant {
        stepperemu::Instant::from_ticks(self.advance().ticks() as u32)
    }
}

struct StepperCallbacks<'a>(&'a TargetQueue);

impl<'a> stepperemu::Callbacks for StepperCallbacks<'a> {
    fn can_append(&self) -> bool {
        self.0.can_append()
    }

    fn append(&mut self, time: stepperemu::Instant, value: u32) {
        self.0.append(time.ticks(), value);
    }

    fn update_last(&mut self, time: stepperemu::Instant, value: u32) {
        self.0.update_last(time.ticks(), value);
    }
}

pub const TARGET_QUEUE_DEPTH: usize = 2000;

pub struct TargetQueueInner {
    queue: Deque<(Instant, u32), TARGET_QUEUE_DEPTH>,
    last_value: u32,
    value_offset: u32,
}

pub struct TargetQueue {
    inner: BlockingMutex<CriticalSectionRawMutex, RefCell<TargetQueueInner>>,
}

impl TargetQueue {
    pub fn new() -> TargetQueue {
        Self {
            inner: BlockingMutex::new(RefCell::new(TargetQueueInner {
                queue: Deque::new(),
                last_value: 0,
                value_offset: 0,
            })),
        }
    }

    fn can_append(&self) -> bool {
        !self.inner.lock(|q| q.borrow().queue.is_full())
    }

    fn append(&self, time: u32, value: u32) {
        self.inner.lock(|q| {
            let mut q = q.borrow_mut();

            let time = Clock::clock32_to_clock64(time);
            q.queue.push_back((time, value)).unwrap();
            q.last_value = value;
        });
    }

    fn update_last(&self, _time: u32, value: u32) {
        self.inner.lock(|q| {
            let mut q = q.borrow_mut();
            if let Some((_, v)) = q.queue.back_mut() {
                *v = value;
            }
            q.last_value = value;
        });
    }

    fn set_offset(&self, target: i32) {
        self.inner.lock(|q| {
            let mut inner = q.borrow_mut();
            let offset = (target as u32) - (inner.last_value + inner.value_offset);
            inner.value_offset = offset;
        });
    }

    pub fn get_for_control(
        &self,
        time: Instant,
    ) -> (i32, Option<(Instant, i32)>, Option<(Instant, i32)>) {
        self.inner.lock(|q| {
            let mut inner = q.borrow_mut();
            let last = (inner.last_value + inner.value_offset) as i32;
            let q = &mut inner.queue;

            // Remove from front such that the next item will be read now

            while let Some((t, _)) = q.front() {
                if *t >= time {
                    break;
                }
                q.pop_front();
            }
            if q.is_empty() {
                return (last, None, None);
            }
            let mut iter = q.iter();
            let v0 = iter.next().copied();
            let v0 = match v0 {
                Some((t0, v0)) if t0 == time => v0,
                _ => return (last, None, None),
            };
            let v1 = iter.next().copied();
            let v2 = iter.next().copied();
            (
                (v0 + inner.value_offset) as i32,
                v1.map(|(t, v)| (t, (v + inner.value_offset) as i32)),
                v2.map(|(t, v)| (t, (v + inner.value_offset) as i32)),
            )
        })
    }
}

pub fn process_moves(cx: &mut crate::app::stepper_move_processor::Context) {
    cx.shared.command_state.lock(|cs| {
        let mut cb = StepperCallbacks(cx.shared.target_queue);
        cs.stepper.advance(&mut cb);
    });
}

#[klipper_command]
pub fn config_stepper(
    context: &mut CommandContext,
    oid: u8,
    _step_pin: u8,
    _dir_pin: u8,
    _invert_step: u8,
    _step_pulse_ticks: u32,
) {
    context.state.stepper_oid = Some(oid);
}

#[klipper_command]
pub fn queue_step(context: &mut CommandContext, oid: u8, interval: u32, count: u16, add: i16) {
    if context.state.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    context.state.stepper.queue_move(interval, count, add);
}

#[klipper_command]
pub fn set_next_step_dir(context: &mut CommandContext, oid: u8, dir: u8) {
    if context.state.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    context.state.stepper.set_next_dir(if dir == 1 {
        Direction::Forward
    } else {
        Direction::Backward
    });
}

#[klipper_command]
pub fn reset_step_clock(context: &mut CommandContext, oid: u8, clock: u32) {
    if context.state.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    context
        .state
        .stepper
        .reset_clock(stepperemu::Instant::from_ticks(clock));
}

#[klipper_command]
pub fn stepper_get_position(context: &mut CommandContext, oid: u8) {
    if context.state.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    let pos = context
        .interfaces
        .pid_last_measured_position
        .load(portable_atomic::Ordering::SeqCst);
    klipper_reply!(stepper_position, oid: u8, pos: i32)
}

#[klipper_command]
pub fn stepper_get_commanded_position(context: &mut CommandContext, oid: u8) {
    if context.state.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    let pos = context
        .interfaces
        .pid_last_commanded_position
        .load(portable_atomic::Ordering::SeqCst);
    klipper_reply!(stepper_commanded_position, oid: u8, pos: i32)
}

#[klipper_command]
pub fn stepper_set_bias(context: &mut CommandContext, target: u32) {
    context.interfaces.target_queue.set_offset(target as i32);
}

#[klipper_command]
pub fn stepper_stop_on_trigger(context: &mut CommandContext, oid: u8, _trsync_oid: u8) {
    if context.state.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    klipper_shutdown!("trsync not supported", Clock::clock32());
}

#[klipper_command]
pub fn config_digital_out(
    context: &mut CommandContext,
    oid: u8,
    pin: u8,
    _value: u8,
    _default_value: u8,
    _max_duration: u32,
) {
    if pin != Pins::Enable.into() {
        return;
    }
    context.state.stepper_enable_oid = Some(oid);
}

#[klipper_command]
pub fn queue_digital_out(context: &mut CommandContext, oid: u8, _clock: u32, on_ticks: u32) {
    if Some(oid) != context.state.stepper_enable_oid {
        return;
    }
    let enable = on_ticks != 0;
    if !enable {
        context.state.stepper.reset_target(0);
    }
    context
        .interfaces
        .pid_set_enable
        .store(enable, portable_atomic::Ordering::SeqCst);
}

#[klipper_command]
pub fn update_digital_out(context: &mut CommandContext, oid: u8, value: u8) {
    if Some(oid) != context.state.stepper_enable_oid {
        return;
    }
    let enable = value != 0;
    if !enable {
        context.state.stepper.reset_target(0);
    }
    context
        .interfaces
        .pid_set_enable
        .store(enable, portable_atomic::Ordering::SeqCst);
}

klipper_enumeration! {
    #[klipper_enumeration(name = "pin", rename_all="snake_case")]
    enum Pins {
        Step,
        Dir,
        Enable,
        Endstop,
    }
}
