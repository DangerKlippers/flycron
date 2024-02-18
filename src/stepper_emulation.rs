use crate::commands::CommandState;
use crate::movequeue::{ MoveQueue, MoveNode };
use anchor::*;

struct StepperFlags;

impl StepperFlags {
    const LAST_DIR: u8 = 0;
    const NEXT_DIR: u8 = 1;
    const INVERT_STEP: u8 = 2;
    const NEEDS_RESET: u8 = 3;
}

pub struct StepperMove {
    node: MoveNode,
    interval: u32,
    add: i16,
    count: u16,
}

impl StepperMove {
    pub fn setup() -> Self {
        Self {
            node: MoveNode::setup(),
            interval: 0,
            add: 0,
            count: 0,
        }
    }
}

pub struct Stepper {
    oid: u8,
    //time: Time,
    interval: u32,
    //add: i16,
    count: u16,
    //next_step_time: u32,
    //step_pulse_ticks: u32,
    position: u32,
    mq: MoveQueue,
    //stop_signal: TrsyncSignal,
    flags: u8,
}

#[klipper_command]
pub fn config_stepper(
    _context: &mut CommandState,
    _oid: u8,
    _step_pin: u8,
    _dir_pin: u8,
    _invert_step: u8,
    _step_pulse_ticks: u32
) {
    let stepper: &mut Stepper = _context.stepper.as_mut().unwrap();
    stepper.oid = _oid;
    stepper.position = 0;
    stepper.mq = MoveQueue::setup();
}

#[klipper_command]
pub fn queue_step(_context: &mut CommandState, _oid: u8, _interval: u32, _count: u16, _add: i16) {
    let stepper: &mut Stepper = _context.stepper.as_mut().unwrap();

    if _count == 0 {
        // shutdown ?
    }

    let stepper_move = StepperMove {
        node: MoveNode {},
        interval: _interval,
        count: _count,
        add: _add,
    };

    // flags stuff

    if stepper.count > 0 {
        stepper.mq.push(stepper_move.node);
    }
}

#[klipper_command]
pub fn set_next_step_dir(_context: &mut CommandState, _oid: u8, _dir: u8) {
    let stepper: &mut Stepper = _context.stepper.as_mut().unwrap();
    stepper.flags =
        (stepper.flags & !StepperFlags::NEXT_DIR) |
        (if _dir != 0 { StepperFlags::NEXT_DIR } else { 0 });
}

#[klipper_command]
pub fn reset_step_clock(_context: &mut CommandState, _oid: u8, _clock: u8) {
    let stepper: &mut Stepper = _context.stepper.as_mut().unwrap();
    // stepper.next_step_time = _clock;
    // stepper.time.waketime = _clock;
    stepper.flags &= !StepperFlags::NEEDS_RESET;
}

#[klipper_command]
pub fn stepper_get_position(_context: &mut CommandState, _oid: u8) {
    klipper_reply!(
        stepper_position,
        oid: u8 = _context.stepper.as_ref().unwrap().oid,
        pos: u32 = _context.stepper.as_ref().unwrap().position
    );
}

#[klipper_command]
pub fn stepper_stop_on_trigger(_context: &mut CommandState, _oid: u8, _trsync_oid: u8) {
    // TODO:
}

#[klipper_command]
pub fn config_digital_out(
    _context: &mut CommandState,
    _oid: u8,
    _pin: u8,
    _value: u8,
    _default_value: u8,
    _max_duration: u32
) {
    // TODO:
}

#[klipper_command]
pub fn queue_digital_out(_context: &mut CommandState, _oid: u8, _clock: u8, _on_ticks: u32) {
    // TODO:
}

#[klipper_command]
pub fn update_digital_out(_context: &mut CommandState, _oid: u8, _value: u8) {
    // TODO:
}
