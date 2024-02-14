use crate::clock::Clock;
use anchor::*;

pub struct CommandState {
    config_crc: Option<u32>,
}

impl CommandState {
    pub fn init() -> Self {
        Self { config_crc: None }
    }
}

#[klipper_command]
pub fn get_uptime() {
    let c: u64 = Clock::now().ticks();
    klipper_reply!(
        uptime,
        high: u32 = (c >> 32) as u32,
        clock: u32 = (c & 0xFFFF_FFFF) as u32
    );
}

#[klipper_command]
pub fn get_clock() {
    let c = Clock::clock32();
    klipper_reply!(
        clock,
        clock: u32 = c
    );
}

#[klipper_command]
pub fn emergency_stop() {}

#[klipper_command]
pub fn get_config(context: &CommandState) {
    let crc = context.config_crc;
    klipper_reply!(
        config,
        is_config: bool = crc.is_some(),
        crc: u32 = crc.unwrap_or(0),
        is_shutdown: bool = false,
        move_count: u16 = 0
    );
}

#[klipper_command]
pub fn config_reset(context: &mut CommandState) {
    context.config_crc = None;
}

#[klipper_command]
pub fn finalize_config(context: &mut CommandState, crc: u32) {
    context.config_crc = Some(crc);
}

#[klipper_command]
pub fn allocate_oids(_count: u8) {}

#[klipper_command]
pub fn debug_nop() {}

#[klipper_constant]
const MCU: &str = "beacon";

#[klipper_constant]
const STATS_SUMSQ_BASE: u32 = 256;
