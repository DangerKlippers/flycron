use anchor::*;
use core::sync::atomic::AtomicI32;

use crate::{
    commands::CommandContext,
    hal::{
        prelude::*,
        qei::{Instance, Qei},
    },
};

pub struct Encoder<TIM: Instance> {
    qei: Qei<TIM>,
    bias: AtomicI32,
}

impl<TIM: Instance<Width = u32>> Encoder<TIM> {
    pub fn new(qei: Qei<TIM>) -> Self {
        Self {
            qei,
            bias: AtomicI32::new(0),
        }
    }

    fn raw_count(&self) -> i32 {
        let raw: u32 = self.qei.count();
        if raw > i32::MAX as u32 {
            -(u32::MAX as i32 - raw as i32)
        } else {
            raw as i32
        }
    }

    pub fn count(&self) -> i32 {
        let raw = self.raw_count();
        raw + self.bias.load(portable_atomic::Ordering::Relaxed)
    }

    pub fn set_value(&self, target: i32) {
        let cur = self.raw_count();
        let bias = target - cur;
        defmt::info!("set value {} {}", target, bias);
        self.bias.store(bias, portable_atomic::Ordering::Relaxed);
    }
}

#[klipper_command]
pub fn encoder_set_position(context: &CommandContext, value: u32) {
    let target: i32 = value as i32;
    context.interfaces.encoder_override.signal(target);
}
