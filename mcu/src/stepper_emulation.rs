use crate::{
    clock::Clock,
    commands::CommandContext,
    pid::{self, PidTimeIterator},
};
use anchor::*;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex as BlockingMutex};
use rtic::Mutex;
use stepperemu::Direction;

pub type EmulatedStepper = stepperemu::EmulatedStepper<PidTimeIterator>;

impl stepperemu::PidTimeIterator for pid::PidTimeIterator {
    fn next(&mut self) -> u32 {
        self.next().ticks() as u32
    }

    fn advance(&mut self) -> u32 {
        self.advance().ticks() as u32
    }
}

pub const TARGET_QUEUE_DEPTH: usize = 2000;

pub struct MutexWrapper;

impl stepperemu::Mutex for MutexWrapper {
    type Inner<T> = BlockingMutex<CriticalSectionRawMutex, T>;

    fn new<T>(val: T) -> Self::Inner<T> {
        BlockingMutex::new(val)
    }

    fn lock<T, R>(inner: &Self::Inner<T>, f: impl FnOnce(&T) -> R) -> R {
        inner.lock(f)
    }
}

pub type TargetQueue = stepperemu::TargetQueue<MutexWrapper, TARGET_QUEUE_DEPTH>;

pub fn process_moves(cx: &mut crate::app::stepper_move_processor::Context) {
    cx.shared.command_state.lock(|cs| {
        cs.stepper.advance(&mut cx.shared.target_queue);
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
    context.state.stepper.reset_clock(clock);
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
