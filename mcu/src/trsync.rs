use crate::commands::CommandContext;
use anchor::*;

#[klipper_command]
pub fn config_trsync(_context: &mut CommandContext, _oid: u8) {
    // TODO:
}

#[klipper_command]
pub fn trsync_start(
    _context: &mut CommandContext,
    _oid: u8,
    _report_clock: u32,
    _report_ticks: u32,
    _expire_reason: u8,
) {
    // TODO:
}

#[klipper_command]
pub fn trsync_set_timeout(_context: &mut CommandContext, _oid: u8, _clock: u32) {
    // TODO:
}

#[klipper_command]
pub fn trsync_trigger(_context: &mut CommandContext, _oid: u8, _reason: u8) {
    // TODO:
}

#[klipper_command]
pub fn trsync_state(_context: &mut CommandContext, _oid: u8, _can_trigger: u8, _trigger_reason: u8, _clock: u32) {
    // TODO:
}
