

use stm32f4xx_hal::dma;
use stm32f4xx_hal::pac;
use stm32f4xx_hal::timer;
use stm32f4xx_hal::bb;

pub const DSHOT_150_MHZ: u32 = 3;
pub const DSHOT_300_MHZ: u32 = 6;
pub const DSHOT_600_MHZ: u32 = 12;
pub const DSHOT_1200_MHZ: u32 = 24;

pub const DMA_BUFFER_LEN: usize = DSHOT_BUFFER_LEN + 2;
const DSHOT_BUFFER_LEN: usize = 16;
const DSHOT_BIT_LEN: u32 = 20;
const DSHOT_BIT_0: u16 = 8;
const DSHOT_BIT_1: u16 = 16;

struct Dshot{
    dma_transfer: dma::Transfer<dma::StreamX<pac::DMA1, 4>, 5, timer::CCR<pac::TIM3, 0>, dma::MemoryToPeripheral, &'static mut [u16; 18]>,
    throttle: u16,
    buffer: Option<&'static mut [u16; DMA_BUFFER_LEN]>,
}

impl Dshot {
    fn new(device: pac::Peripherals) -> Dshot{
        let dma1 = device.DMA1;
        let dma1_streams = dma::StreamsTuple::new(dma1);
        let tim3 = device.TIM3;
        let ccr1_tim3 = timer::CCR::<pac::TIM3, 0>(tim3);
        let dma_config = get_dshot_dma_cfg();
        let dma_transfer: dma::Transfer<dma::StreamX<pac::DMA1, 4>, 5, timer::CCR<pac::TIM3, 0>, dma::MemoryToPeripheral, &mut [u16; 18]> = dma::Transfer::init_memory_to_peripheral(
            dma1_streams.4,
            ccr1_tim3,
            cortex_m::singleton!(: [u16; DMA_BUFFER_LEN] = [0; DMA_BUFFER_LEN]).unwrap(),
            None,
            dma_config,
        );
        let buffer = Some(cortex_m::singleton!(: [u16; DMA_BUFFER_LEN] = [0; DMA_BUFFER_LEN]).unwrap());
        let throttle = 0;
        Dshot {
            dma_transfer,
            throttle,
            buffer,
        }
    }   
    fn set_throttle(&mut self, throttle: u16) {
        self.throttle = throttle;
    }
    fn transmit_frame(&mut self) {
        let buffer = self.buffer.take().unwrap();
        let mut packet = encode_dshot_packet(self.throttle, false);
        for n in 0..DSHOT_BUFFER_LEN {
            buffer[n] = match packet & 0x8000 {
                0 => DSHOT_BIT_0,
                _ => DSHOT_BIT_1,
            };

            packet <<= 1;
        }
        let buffer = self.dma_transfer.next_transfer(buffer).unwrap();
        self.buffer = Some(buffer.0);
    }
    fn start(&mut self) {
        //(tim3, apb1enr, apb1rstr, 1u8, pclk1, ppre1),
        let bit = 1u8;
        unsafe {
            let rcc = &(*pac::RCC::ptr());
            bb::set(&rcc.apb1rstr, bit);
            bb::set(&rcc.apb1rstr, bit);
            bb::clear(&rcc.apb1rstr, bit);
        }
        self.tim.ccmr1_output()
            .modify(|_, w| w.oc1pe().set_bit().oc1m().pwm_mode1());
        self.tim.ccmr1_output()
            .modify(|_, w| w.oc2pe().set_bit().oc2m().pwm_mode1());
        self.tim.cr1.modify(|_, w| w.arpe().set_bit());
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
