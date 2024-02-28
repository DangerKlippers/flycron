use crate::hal::{
    prelude::*,
    qei::{Instance, Qei},
};

pub struct Encoder<TIM: Instance>(Qei<TIM>);

impl<TIM: Instance<Width = u32>> Encoder<TIM> {
    pub fn new(qei: Qei<TIM>) -> Self {
        Self(qei)
    }

    pub fn count(&self) -> i32 {
        let raw: u32 = self.0.count();
        if raw > i32::MAX as u32 {
            -(u32::MAX as i32 - raw as i32)
        } else {
            raw as i32
        }
    }
}
