use crate::clock::Clock;
use anchor::*;
use control_law::PidGains;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

pub struct CommandState {
    pub config_crc: Option<u32>,
}

impl CommandState {
    pub fn init() -> Self {
        Self { config_crc: None }
    }
}

pub struct CommandInterfaces<'ctx> {
    pub pid_gains: &'ctx Signal<CriticalSectionRawMutex, PidGains>,
    pub pid_setpoint: &'ctx portable_atomic::AtomicI32,
    pub pid_set_enable: &'ctx portable_atomic::AtomicBool,
}

pub struct CommandContext<'ctx> {
    pub state: &'ctx mut CommandState,
    pub interfaces: CommandInterfaces<'ctx>,
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
pub fn get_config(context: &CommandContext) {
    let crc = context.state.config_crc;
    klipper_reply!(
        config,
        is_config: bool = crc.is_some(),
        crc: u32 = crc.unwrap_or(0),
        is_shutdown: bool = false,
        move_count: u16 = 0
    );
}

#[klipper_command]
pub fn config_reset(context: &mut CommandContext) {
    context.state.config_crc = None;
}

#[klipper_command]
pub fn finalize_config(context: &mut CommandContext, crc: u32) {
    context.state.config_crc = Some(crc);
}

#[klipper_command]
pub fn allocate_oids(_count: u8) {}

#[klipper_command]
pub fn debug_nop() {}

#[klipper_constant]
const MCU: &str = "flycron";

#[klipper_constant]
const STATS_SUMSQ_BASE: u32 = 256;
