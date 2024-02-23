#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

mod chipid;
mod clock;
mod commands;
mod detail;
mod dshot;
mod pid;
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
        commands::{CommandContext, CommandInterfaces},
        create_stm32_tim2_monotonic_token,
        dshot::{DShotSpeed, Dshot, ThrottleCommand},
        hal::{prelude::*, qei::Qei},
        pid::{next_pid_time, PidGains},
        usb::*,
    };
    use bbqueue::BBBuffer;
    use core::mem::{discriminant, MaybeUninit};
    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
    use micromath::F32Ext;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice,
        command_state: crate::commands::CommandState,
        encoder: Qei<stm32f4xx_hal::pac::TIM5>,

        dshot_throttle: Signal<CriticalSectionRawMutex, ThrottleCommand>,
        dshot: Dshot,
        dshot_complete: Signal<CriticalSectionRawMutex, ()>,

        pid_gains: Signal<CriticalSectionRawMutex, PidGains>,
        pid_setpoint: portable_atomic::AtomicF32,
        pid_set_enable: portable_atomic::AtomicBool,
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

        let out = gpiob.pb4.into_push_pull_output();
        let dshot = crate::dshot::Dshot::new(
            cx.device.DMA1,
            cx.device.TIM3,
            out,
            clocks.timclk1(),
            DShotSpeed::Speed150kHz,
        );

        dshot_loop::spawn().ok();
        pid_loop::spawn().ok();

        (
            Shared {
                usb_dev,
                command_state,
                encoder,

                dshot_throttle: Signal::new(),
                dshot,
                dshot_complete: Signal::new(),

                pid_gains: Signal::new(),
                pid_setpoint: portable_atomic::AtomicF32::new(5000.0),
                pid_set_enable: portable_atomic::AtomicBool::new(true),
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

    #[task(binds = OTG_FS, priority = 2, shared = [usb_dev, command_state, &pid_gains, &pid_setpoint, &pid_set_enable])]
    fn irq_usb(mut cx: irq_usb::Context) {
        cx.shared.usb_dev.lock(|usb_dev| {
            if usb_dev.on_interrupt() {
                cx.shared.command_state.lock(|cs| {
                    usb_dev.handle_commands(CommandContext {
                        state: cs,
                        interfaces: CommandInterfaces {
                            pid_gains: cx.shared.pid_gains,
                            pid_setpoint: cx.shared.pid_setpoint,
                            pid_set_enable: cx.shared.pid_set_enable,
                        },
                    })
                });
                usb_dev.pump_write();
            }
        });
    }

    #[task(priority = 8, shared = [dshot, &dshot_complete, &dshot_throttle])]
    async fn dshot_loop(mut cx: dshot_loop::Context) {
        let deadline = Clock::now() + Duration::millis(3000);
        defmt::info!("Arming");
        while Clock::now() < deadline {
            cx.shared
                .dshot
                .lock(|dshot| dshot.send_throttle(ThrottleCommand::Throttle(0).into()));
            Clock::delay(Duration::millis(1)).await;
        }
        defmt::info!("Armed, entering throttle loop");
        let mut cnt = 0;
        let mut last_report = Clock::now();
        loop {
            let throttle = cx.shared.dshot_throttle.wait().await;
            cx.shared
                .dshot
                .lock(|dshot| dshot.send_throttle(throttle.into()));
            cx.shared.dshot_complete.wait().await;
            cnt += 1;

            let now = Clock::now();
            if now >= last_report + crate::clock::Duration::millis(1000) {
                last_report = now;
                defmt::info!("COUNT {}, LAST VAL {}", cnt, throttle);
                cnt = 0;
            }
        }
    }

    #[task(binds = DMA1_STREAM4, priority = 9, shared = [dshot, &dshot_complete])]
    fn dshot_dma_finish(mut cx: dshot_dma_finish::Context) {
        if cx.shared.dshot.lock(|dshot| dshot.on_interrupt()) {
            cx.shared.dshot_complete.signal(());
        }
    }

    #[task(priority = 7, shared = [&encoder, &dshot_throttle, &pid_gains, &pid_setpoint, &pid_set_enable])]
    async fn pid_loop(cx: pid_loop::Context) {
        let mut controller = ::pid::Pid::<f32>::new(0.0f32, 1.0);
        controller.p(0.12, 0.12);
        controller.i(0.12, 0.12);
        loop {
            let next_time = next_pid_time();
            Clock::delay_until(next_time).await;
            if !cx
                .shared
                .pid_set_enable
                .load(portable_atomic::Ordering::Relaxed)
            {
                cx.shared.dshot_throttle.signal(ThrottleCommand::MotorsOff);
                continue;
            }
            if cx.shared.pid_gains.signaled() {
                // NOTE: Will return immediately because we checked for signal
                let gains = cx.shared.pid_gains.wait().await;
                controller
                    .p(gains.p, gains.p_max)
                    .i(gains.i, gains.i_max)
                    .d(gains.d, gains.d_max);
            }

            // TODO: Add scaling to stepper units or something?
            let current_position;
            let raw_pulse_count = cx.shared.encoder.count() as i32;
            if raw_pulse_count > i32::MAX {
                current_position = -(u32::MAX as i32 - raw_pulse_count) as f32;
            } else {
                current_position = raw_pulse_count as f32;
            }
            // defmt::info!("Raw pulse count: {}", raw_pulse_count);
            // TODO: Replace with sampling from stepper emulation at the current(or next?) time step
            let target_position = cx
                .shared
                .pid_setpoint
                .load(portable_atomic::Ordering::Relaxed);

            controller.setpoint(target_position);
            let throttle = controller.next_control_output(current_position).output;
            let mut actual_throttle: f32;
            let max_thrust = 408.0;
            let target_thrust = throttle * max_thrust;

            let a = 427.0;
            let b = -22.1;
            let c = 3.56 - target_thrust;
            let discriminant: f32 = b * b - 4.0 * a * c;
            if discriminant > 0.0 {
                let sqrt_discriminant = discriminant.sqrt();
                let x1 = (-b + sqrt_discriminant) / (2.0 * a);
                let x2 = (-b - sqrt_discriminant) / (2.0 * a);
                actual_throttle = x1.max(x2); // take the positive one
            } else if discriminant == 0.0 {
                actual_throttle = -b / (2.0 * a);
            } else {
                actual_throttle = 0.0;
            }
            if throttle < 0.0 {
                actual_throttle = 0.0;
            }

            let scaled_throttle = actual_throttle.clamp(0.0, 1.0) * ThrottleCommand::MAX as f32;
            // defmt::info!("Throttle: {}", scaled_throttle as u16);
            cx.shared
                .dshot_throttle
                .signal(ThrottleCommand::Throttle(scaled_throttle as u16));
        }
    }
}

use anchor::klipper_config_generate;
klipper_config_generate!(
  transport = crate::usb::TRANSPORT_OUTPUT: crate::usb::BufferTransportOutput,
  context = crate::commands::CommandContext<'ctx>,
);
