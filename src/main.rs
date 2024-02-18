#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

mod chipid;
mod clock;
mod commands;
mod detail;
mod dshot;
mod stepper_emulation;
mod trsync;
mod usb;

use stm32f4xx_hal as hal;

#[rtic::app(
    device = crate::hal::pac,
    dispatchers = [I2C1_EV, I2C1_ER, I2C2_EV, I2C2_ER, I2C3_EV, I2C3_ER],
)]
mod app {
    use crate::{
        clock::{Clock, Duration},
        create_stm32_tim2_monotonic_token,
        dshot::{DShotSpeed, Dshot},
        hal::{prelude::*, qei::Qei},
        usb::*,
    };
    use bbqueue::BBBuffer;
    use core::mem::MaybeUninit;
    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

    #[shared]
    struct Shared {
        usb_dev: UsbDevice,
        command_state: crate::commands::CommandState,
        encoder: Qei<stm32f4xx_hal::pac::TIM5>,
        dshot: Dshot,
        dshot_complete: Signal<CriticalSectionRawMutex, ()>,
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
        let gpiob = cx.device.GPIOB.split();

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

        let encoder = Qei::new(
            cx.device.TIM5,
            (
                gpioa.pa0.into_pull_up_input(),
                gpioa.pa1.into_pull_up_input(),
            ),
        );

        // encoder_read::spawn().ok();

        //(tim3, apb1enr, apb1rstr, 1u8, pclk1, ppre1),
        let out = gpiob.pb4.into_push_pull_output();
        let dshot = crate::dshot::Dshot::new(
            cx.device.DMA1,
            cx.device.TIM3,
            out,
            clocks.timclk1(),
            DShotSpeed::Speed1200kHz,
        );

        dshot_loop::spawn().ok();
        (
            Shared {
                usb_dev,
                command_state,
                encoder,
                dshot,
                dshot_complete: Signal::new(),
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

    #[task(priority = 8, shared = [dshot, &dshot_complete])]
    async fn dshot_loop(mut cx: dshot_loop::Context) {
        let mut cnt = 0;
        let mut last_report = Clock::now();
        loop {
            cx.shared
                .dshot
                .lock(|dshot| dshot.send_throttle(48 + cnt % 2000));
            cx.shared.dshot_complete.wait().await;
            cnt += 1;

            let now = Clock::now();
            if now > last_report + crate::clock::Duration::millis(1000) {
                last_report = now;
                defmt::info!("COUNT {}", cnt);
                cnt = 0;
            }
        }
    }

    #[task(priority = 1, shared = [&encoder])]
    async fn encoder_read(cx: encoder_read::Context) {
        loop {
            Clock::delay(Duration::millis(10)).await;
            let count = cx.shared.encoder.count();
            defmt::info!("Count: {}", count);
        }
    }

    #[task(binds = DMA1_STREAM4, priority = 9, shared = [dshot, &dshot_complete])]
    fn dshot_dma_finish(mut cx: dshot_dma_finish::Context) {
        if cx.shared.dshot.lock(|dshot| dshot.on_interrupt()) {
            cx.shared.dshot_complete.signal(());
        }
    }
}

use anchor::klipper_config_generate;
klipper_config_generate!(
  transport = crate::usb::TRANSPORT_OUTPUT: crate::usb::BufferTransportOutput,
  context = &'ctx mut crate::commands::CommandState,
);
