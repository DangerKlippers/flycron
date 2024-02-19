use core::mem;
use fugit::HertzU32;
use stm32f4xx_hal::{
    dma::{self, DMAError},
    gpio::PushPull,
    pac::Interrupt,
    pac::{tim3, DMA1, RCC, TIM3},
    prelude::*,
    timer::{self, CPin},
    ClearFlags, ReadFlags,
};

use crate::hal::rcc::{Enable, Reset};

#[derive(defmt::Format, Copy, Clone, Eq, PartialEq, Ord, PartialOrd)]
pub enum ThrottleCommand {
    Disarmed,
    Throttle(u16),
}

impl ThrottleCommand {
    pub const MIN: u16 = 0;
    pub const MAX: u16 = 800;
}

impl From<ThrottleCommand> for u16 {
    fn from(value: ThrottleCommand) -> Self {
        match value {
            ThrottleCommand::Disarmed => 0,
            ThrottleCommand::Throttle(v) => 48 + v.clamp(0, Self::MAX),
        }
    }
}

#[derive(defmt::Format, Copy, Clone, Eq, PartialEq, Ord, PartialOrd)]
pub enum DShotSpeed {
    Speed150kHz,
    Speed300kHz,
    Speed600kHz,
    Speed1200kHz,
}

impl From<DShotSpeed> for HertzU32 {
    fn from(value: DShotSpeed) -> Self {
        match value {
            DShotSpeed::Speed150kHz => HertzU32::kHz(150),
            DShotSpeed::Speed300kHz => HertzU32::kHz(300),
            DShotSpeed::Speed600kHz => HertzU32::kHz(600),
            DShotSpeed::Speed1200kHz => HertzU32::kHz(1200),
        }
    }
}

pub const DMA_BUFFER_LEN: usize = DSHOT_BUFFER_LEN + 2;
const DSHOT_BUFFER_LEN: usize = 16;
const DSHOT_BIT_LEN: u32 = 20;
const DSHOT_BIT_0: u16 = 8;
const DSHOT_BIT_1: u16 = 16;

type DshotDmaTransfer = dma::Transfer<
    dma::StreamX<DMA1, 4>,
    5,
    timer::CCR<TIM3, 0>,
    dma::MemoryToPeripheral,
    &'static mut [u16; 18],
>;

pub struct Dshot {
    dma_transfer: DshotDmaTransfer,
    idle_buffer: Option<&'static mut [u16; DMA_BUFFER_LEN]>,
}

impl Dshot {
    pub fn new(
        dma1: DMA1,
        timer: TIM3,
        output_pin: impl Into<<TIM3 as CPin<0>>::Ch<PushPull>>,
        clk: HertzU32,
        speed: DShotSpeed,
    ) -> Dshot {
        unsafe {
            let rcc = &(*RCC::ptr());
            TIM3::enable(rcc);
            TIM3::reset(rcc);
        }

        let dma1_streams = dma::StreamsTuple::new(dma1);

        timer.ccmr1_output().modify(|_, w| {
            w.oc1pe().set_bit();
            w.oc1fe().set_bit();
            w.oc1m().pwm_mode1()
        });

        let speed: HertzU32 = speed.into();
        let psc = clk.to_Hz() / speed.to_Hz() / DSHOT_BIT_LEN;
        timer.psc.write(|w| w.psc().bits((psc - 1) as u16));
        timer.arr.write(|w| w.arr().variant(DSHOT_BIT_LEN as u16));

        timer.cr1.write(|w| {
            w.cms().variant(tim3::cr1::CMS_A::EdgeAligned);
            w.dir().clear_bit();
            w.opm().clear_bit();
            w.arpe().set_bit();
            w.urs().clear_bit();
            w.cen().set_bit()
        });
        timer.ccer.modify(|_, w| w.cc1e().set_bit());
        timer.dier.modify(|_, w| w.cc1de().enabled());

        let ccr1_tim3 = timer::CCR::<TIM3, 0>(unsafe { mem::transmute_copy(&timer) });
        let dma_config = get_dshot_dma_cfg();
        let dma_transfer = dma::Transfer::init_memory_to_peripheral(
            dma1_streams.4,
            ccr1_tim3,
            cortex_m::singleton!(: [u16; DMA_BUFFER_LEN] = [0; DMA_BUFFER_LEN]).unwrap(),
            None,
            dma_config,
        );
        let idle_buffer = cortex_m::singleton!(: [u16; DMA_BUFFER_LEN] = [0; DMA_BUFFER_LEN]);

        let _pin = output_pin.into();

        unsafe {
            cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_STREAM4);
        }

        Dshot {
            dma_transfer,
            idle_buffer,
        }
    }

    pub fn send_throttle(&mut self, value: u16) {
        let mut packet = encode_dshot_packet(value, false);
        let buffer = self.idle_buffer.take().unwrap();
        for slot in buffer.iter_mut().take(DSHOT_BUFFER_LEN) {
            *slot = match packet & 0x8000 {
                0 => DSHOT_BIT_0,
                _ => DSHOT_BIT_1,
            };
            packet <<= 1;
        }
        buffer[DSHOT_BUFFER_LEN] = 0;
        buffer[DSHOT_BUFFER_LEN + 1] = 0;
        let ret = self.dma_transfer.next_transfer(buffer);
        match ret {
            // Some error occured, maybe buffer was still in flight. We'll just ignore it though,
            // as the client will ask us to send again later. We do need to save our buffer again
            // however.
            Err(DMAError::NotReady(buf)) => self.idle_buffer = Some(buf),
            Err(DMAError::Overrun(buf)) => self.idle_buffer = Some(buf),
            Err(DMAError::SmallBuffer(buf)) => self.idle_buffer = Some(buf),
            // Replaced in-flight buffer, save the one that we got back
            Ok((buffer, _)) => self.idle_buffer = Some(buffer),
        }
    }

    pub fn on_interrupt(&mut self) -> bool {
        let flags = self.dma_transfer.flags();
        self.dma_transfer.clear_all_flags();
        flags.is_transfer_complete() || flags.is_transfer_error()
    }
}
fn get_dshot_dma_cfg() -> dma::config::DmaConfig {
    dma::config::DmaConfig::default()
        .transfer_complete_interrupt(true)
        .transfer_error_interrupt(false)
        .half_transfer_interrupt(false)
        .fifo_enable(true)
        .fifo_threshold(dma::config::FifoThreshold::QuarterFull)
        .peripheral_burst(dma::config::BurstMode::NoBurst)
        .peripheral_increment(false)
        .memory_burst(dma::config::BurstMode::NoBurst)
        .memory_increment(true)
        .priority(dma::config::Priority::High)
}
fn encode_dshot_packet(mut packet: u16, enable_tel: bool) -> u16 {
    packet = (packet << 1) | enable_tel as u16;
    let csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0xf;
    (packet << 4) | csum
}
