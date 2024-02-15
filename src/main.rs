#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

mod chipid;
mod clock;
mod commands;
mod detail;
mod stepper_emulation;
mod trsync;
mod usb;

use stm32f4xx_hal as hal;

#[rtic::app(
    device = crate::hal::pac,
    dispatchers = [I2C1_EV, I2C3_EV]
)]
mod app {
    use core::mem::MaybeUninit;

    use crate::{clock::Clock, create_stm32_tim2_monotonic_token, hal::prelude::*, usb::*};
    use bbqueue::BBBuffer;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice,
        command_state: crate::commands::CommandState,
    }

    #[local]
    struct Local {}

    #[init(local = [
        usb_allocator: MaybeUninit<UsbBusAllocator> = MaybeUninit::uninit(),
        usb_ep_memory: [u32; 1024] = [0; 1024],
        usb_tx_queue: BBBuffer<USB_OUTPUT_BUFFER_SIZE> = BBBuffer::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        // Following needed to let RTT attach during sleep
        cx.device.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });
        cx.device.RCC.ahb1enr.modify(|_, w| w.dma1en().enabled());

        // Clock setup
        let rcc = cx.device.RCC.constrain();
        let hse = 25.MHz();
        let sysclk = 96.MHz();
        let clocks = rcc
            .cfgr
            .use_hse(hse)
            .sysclk(sysclk)
            .require_pll48clk()
            .freeze();

        let gpioa = cx.device.GPIOA.split();

        // USB setup
        let usb_allocator = cx.local.usb_allocator.write(UsbBus::new(
            USB {
                usb_global: cx.device.OTG_FS_GLOBAL,
                usb_device: cx.device.OTG_FS_DEVICE,
                usb_pwrclk: cx.device.OTG_FS_PWRCLK,
                pin_dm: gpioa.pa11.into(),
                pin_dp: gpioa.pa12.into(),
                hclk: clocks.hclk(),
            },
            cx.local.usb_ep_memory,
        ));
        let usb_dev = UsbDevice::init(usb_allocator, cx.local.usb_tx_queue);
        usb_send_pump::spawn().ok();

        // Timebase setup
        let token = create_stm32_tim2_monotonic_token!();
        Clock::start(cx.device.TIM2, token);

        let command_state = crate::commands::CommandState::init();

        (
            Shared {
                usb_dev,
                command_state,
            },
            Local {},
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    #[task(priority = 2, shared = [usb_dev])]
    async fn usb_send_pump(mut cx: usb_send_pump::Context) {
        loop {
            UsbDevice::tx_wait().await;
            cx.shared.usb_dev.lock(|usb_dev| usb_dev.pump_write());
        }
    }

    #[task(binds = OTG_FS, priority = 2, shared = [usb_dev, command_state])]
    fn irq_usb(mut cx: irq_usb::Context) {
        cx.shared.usb_dev.lock(|usb_dev| {
            if usb_dev.on_interrupt() {
                cx.shared
                    .command_state
                    .lock(|cs| usb_dev.handle_commands(cs));
                usb_dev.pump_write();
            }
        });
    }
}

use anchor::klipper_config_generate;
klipper_config_generate!(
  transport = crate::usb::TRANSPORT_OUTPUT: crate::usb::BufferTransportOutput,
  context = &'ctx mut crate::commands::CommandState,
);
