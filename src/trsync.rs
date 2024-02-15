use crate::commands::CommandState;
use anchor::*;

#[klipper_command]
pub fn config_trsync(_context: &mut CommandState, _oid: u8) {
    // TODO:
}

#[klipper_command]
pub fn trsync_start(
    _context: &mut CommandState,
    _oid: u8,
    _report_clock: u32,
    _report_ticks: u32,
    _expire_reason: u8,
) {
    // TODO:
}

#[klipper_command]
pub fn trsync_set_timeout(_context: &mut CommandState, _oid: u8, _clock: u32) {
    // TODO:
}

#[klipper_command]
pub fn trsync_trigger(_context: &mut CommandState, _oid: u8, _reason: u8) {
    // TODO:
}
