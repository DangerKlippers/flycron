use crate::commands::CommandState;
use anchor::*;

#[klipper_command]
pub fn config_stepper(
    _context: &mut CommandState,
    _oid: u8,
    _step_pin: u8,
    _dir_pin: u8,
    _invert_step: u8,
    _step_pulse_ticks: u32,
) {
    // TODO:
}

#[klipper_command]
pub fn queue_step(_context: &mut CommandState, _oid: u8, _interval: u32, _count: u16, _add: i16) {
    // TODO:
}

#[klipper_command]
pub fn set_next_step_dir(_context: &mut CommandState, _oid: u8, _dir: u8) {
    // TODO:
}

#[klipper_command]
pub fn reset_step_clock(_context: &mut CommandState, _oid: u8, _clock: u8) {
    // TODO:
}

#[klipper_command]
pub fn stepper_get_position(_context: &mut CommandState, _oid: u8) {
    // TODO:
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
    _max_duration: u32,
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
