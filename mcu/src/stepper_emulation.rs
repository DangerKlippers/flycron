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
        self.next()
    }

    fn advance(&mut self) -> stepperemu::Instant {
        self.advance()
    }
}

struct StepperCallbacks<'a>(&'a TargetQueue);

impl<'a> stepperemu::Callbacks for StepperCallbacks<'a> {
    fn can_append(&self) -> bool {
        self.0.can_append()
    }

    fn append(&mut self, time: Instant, value: u32) {
        self.0.append(time, value);
    }

    fn update_last(&mut self, time: Instant, value: u32) {
        self.0.update_last(time, value);
    }
}

pub const TARGET_QUEUE_DEPTH: usize = 2000;

pub struct TargetQueueInner(Deque<(Instant, u32), TARGET_QUEUE_DEPTH>);

pub struct TargetQueue {
    inner: BlockingMutex<CriticalSectionRawMutex, RefCell<TargetQueueInner>>,
}

impl TargetQueue {
    pub fn new() -> TargetQueue {
        Self {
            inner: BlockingMutex::new(RefCell::new(TargetQueueInner(Deque::new()))),
        }
    }

    fn can_append(&self) -> bool {
        !self.inner.lock(|q| q.borrow().0.is_full())
    }

    fn append(&self, time: Instant, value: u32) {
        self.inner
            .lock(|q| q.borrow_mut().0.push_back((time, value)).unwrap());
    }

    fn update_last(&self, _time: Instant, value: u32) {
        self.inner.lock(|q| {
            if let Some((_, v)) = q.borrow_mut().0.back_mut() {
                *v = value;
            }
        });
    }

    pub fn get_for_control(
        &self,
        time: Instant,
    ) -> (
        Option<(Instant, u32)>,
        Option<(Instant, u32)>,
        Option<(Instant, u32)>,
    ) {
        self.inner.lock(|q| {
            let q = &mut q.borrow_mut().0;

            // Remove from front such that the next item will be read now

            while let Some((t, _)) = q.front() {
                if *t >= time {
                    break;
                }
                q.pop_front();
            }
            if q.is_empty() {
                return (None, None, None);
            }
            let mut iter = q.iter();
            let v0 = iter.next().copied();
            match v0 {
                Some((t0, _)) if t0 == time => {}
                _ => return (None, None, None),
            }
            let v1 = iter.next().copied();
            let v2 = iter.next().copied();
            (v0, v1, v2)
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
    context.state.stepper.set_next_dir(if dir == 0 {
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
        .reset_clock(Clock::clock32_to_clock64(clock));
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
pub fn stepper_stop_on_trigger(context: &mut CommandContext, oid: u8, _trsync_oid: u8) {
    if context.state.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    klipper_shutdown!("trsync not supported", Clock::clock32());
}

#[klipper_command]
pub fn config_digital_out(
    _context: &mut CommandContext,
    _oid: u8,
    _pin: u8,
    _value: u8,
    _default_value: u8,
    _max_duration: u32,
) {
    // TODO:
}

#[klipper_command]
pub fn queue_digital_out(_context: &mut CommandContext, _oid: u8, _clock: u32, _on_ticks: u32) {
    // TODO:
}

#[klipper_command]
pub fn update_digital_out(_context: &mut CommandContext, _oid: u8, _value: u8) {
    // TODO:
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
