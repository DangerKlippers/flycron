use crate::{
    clock::Clock,
    pid::{ModelParam, PidTimeIterator},
    stepper_emulation::{EmulatedStepper, TargetQueue},
};
use anchor::*;
use control_law::PidGains;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};

pub struct CommandState {
    pub config_crc: Option<u32>,

    pub stepper_oid: Option<u8>,
    pub stepper: EmulatedStepper,
    pub stepper_enable_oid: Option<u8>,
}

impl CommandState {
    pub fn init() -> Self {
        Self {
            config_crc: None,

            stepper_oid: None,
            stepper: EmulatedStepper::new(PidTimeIterator::new()),
            stepper_enable_oid: None,
        }
    }
}

pub struct CommandInterfaces<'ctx> {
    pub encoder_override: &'ctx Signal<CriticalSectionRawMutex, i32>,
    pub pid_gains: &'ctx Channel<CriticalSectionRawMutex, (u8, PidGains), 2>,
    pub filter_coefs: &'ctx Channel<CriticalSectionRawMutex, (u8, f32, f32), 2>,
    pub slew_rate_limits: &'ctx Channel<CriticalSectionRawMutex, (u8, f32, f32), 2>,
    pub throttle_limits: &'ctx (portable_atomic::AtomicF32, portable_atomic::AtomicF32),
    pub throttle_force: &'ctx portable_atomic::AtomicF32,
    pub pid_set_enable: &'ctx portable_atomic::AtomicBool,
    pub pid_last_measured_position: &'ctx portable_atomic::AtomicI32,
    pub pid_last_commanded_position: &'ctx portable_atomic::AtomicI32,
    pub pid_last_throttle: &'ctx portable_atomic::AtomicF32,
    pub target_queue: &'ctx TargetQueue,
    pub model_params: &'ctx Channel<CriticalSectionRawMutex, ModelParam, 2>,
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
pub fn emergency_stop(context: &mut CommandContext) {
    klipper_shutdown!("Emergency stop", Clock::clock32());
    config_reset(context);
    context
        .interfaces
        .pid_set_enable
        .store(false, portable_atomic::Ordering::SeqCst);
}

#[klipper_command]
pub fn get_config(context: &CommandContext) {
    defmt::info!("get cfg");
    let crc = context.state.config_crc;
    klipper_reply!(
        config,
        is_config: bool = crc.is_some(),
        crc: u32 = crc.unwrap_or(0),
        is_shutdown: bool = false,
        move_count: u16 = context.state.stepper.move_count() as u16
    );
}

#[klipper_command]
pub fn config_reset(context: &mut CommandContext) {
    defmt::info!("config reset");
    context.state.config_crc = None;
    context.state.stepper_oid = None;
    context.state.stepper = EmulatedStepper::new(PidTimeIterator::new());
}

#[klipper_command]
pub fn finalize_config(context: &mut CommandContext, crc: u32) {
    context.state.config_crc = Some(crc);
}

#[klipper_command]
pub fn allocate_oids(_count: u8) {}

#[klipper_command]
pub fn debug_nop() {}

#[klipper_command]
pub fn reset() {
    cortex_m::peripheral::SCB::sys_reset();
}

#[klipper_constant]
const MCU: &str = "flycron";

#[klipper_constant]
const STATS_SUMSQ_BASE: u32 = 256;
