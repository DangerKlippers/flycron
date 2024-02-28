use crate::hal::{
    pac::{tim2, Interrupt, NVIC_PRIO_BITS, RCC, TIM2},
    rcc::{Enable, Reset},
};
use anchor::klipper_constant;
use core::future::Future;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::NVIC;
use rtic_monotonics::{InterruptToken, Monotonic, TimeoutError, TimerQueue};

static HALF_PERIOD_COUNTER: AtomicU32 = AtomicU32::new(0);

#[klipper_constant]
const CLOCK_FREQ: u32 = 96_000_000;

pub struct Clock;

impl Clock {
    pub fn start(tim2: TIM2, _interrupt_token: impl InterruptToken<Self>) {
        unsafe {
            let rcc = &(*RCC::ptr());
            TIM2::enable(rcc);
            TIM2::reset(rcc);
        }

        tim2.cr1.modify(|_, w| w.cen().set_bit());
        tim2.psc.write(|w| w.psc().variant(0));
        tim2.ccr2()
            .write(|w| w.ccr().variant(u32::MAX - (u32::MAX - 1)));
        tim2.dier.modify(|_, w| {
            w.uie().set_bit(); // Full period interrupt
            w.cc2ie().set_bit() // Half period interrupt
        });
        tim2.egr.write(|w| w.ug().set_bit());
        tim2.sr.modify(|_, w| w.uif().clear_bit());

        HALF_PERIOD_COUNTER.store(0, Ordering::SeqCst);
        TIMER_QUEUE.initialize(Self);

        tim2.cr1.modify(|_, w| w.cen().set_bit());
        unsafe {
            let mut nvic: NVIC = core::mem::transmute(());
            nvic.set_priority(Interrupt::TIM2, (1 << NVIC_PRIO_BITS) - 1);
            NVIC::unmask(Interrupt::TIM2);
        }
    }

    pub fn clock32() -> u32 {
        Self::register().cnt.read().cnt().bits()
    }

    fn register() -> &'static tim2::RegisterBlock {
        unsafe { TIM2::ptr().as_ref().unwrap() }
    }

    pub fn now() -> <Self as Monotonic>::Instant {
        <Self as Monotonic>::now()
    }

    #[doc(hidden)]
    pub fn __tq() -> &'static TimerQueue<Self> {
        &TIMER_QUEUE
    }

    /// Timeout at a specific time.
    #[inline]
    #[allow(dead_code)]
    pub async fn timeout_at<F: Future>(
        instant: <Self as Monotonic>::Instant,
        future: F,
    ) -> Result<F::Output, TimeoutError> {
        TIMER_QUEUE.timeout_at(instant, future).await
    }

    /// Timeout after a specific duration.
    #[inline]
    #[allow(dead_code)]
    pub async fn timeout_after<F: Future>(
        duration: <Self as Monotonic>::Duration,
        future: F,
    ) -> Result<F::Output, TimeoutError> {
        TIMER_QUEUE.timeout_after(duration, future).await
    }

    /// Delay for some duration of time.
    #[inline]
    #[allow(dead_code)]
    pub async fn delay(duration: <Self as Monotonic>::Duration) {
        TIMER_QUEUE.delay(duration).await;
    }

    /// Delay to some specific time instant.
    #[inline]
    #[allow(dead_code)]
    pub async fn delay_until(instant: <Self as Monotonic>::Instant) {
        TIMER_QUEUE.delay_until(instant).await;
    }
}

pub type Instant = fugit::TimerInstantU64<CLOCK_FREQ>;
pub type Duration = fugit::TimerDurationU64<CLOCK_FREQ>;

static TIMER_QUEUE: TimerQueue<Clock> = TimerQueue::new();

impl Monotonic for Clock {
    type Instant = Instant;
    type Duration = Duration;

    const ZERO: Self::Instant = Self::Instant::from_ticks(0);
    const TICK_PERIOD: Self::Duration = Self::Duration::from_ticks(1);

    fn now() -> Self::Instant {
        Self::Instant::from_ticks(rtic_time::half_period_counter::calculate_now(
            || HALF_PERIOD_COUNTER.load(Ordering::Relaxed),
            Self::clock32,
        ))
    }

    fn set_compare(instant: Self::Instant) {
        let reg = Self::register();

        let max = u32::MAX as u64;

        // Check for overflow, if overflow we will just wait for the overflow interrupt to trigger
        // anyway.
        let target = match instant.checked_duration_since(Self::now()) {
            Some(x) if x.ticks() <= max => instant.duration_since_epoch().ticks() as u32,
            _ => 0,
        };

        reg.ccr1().write(|w| w.ccr().variant(target));
    }

    fn clear_compare_flag() {
        Self::register().sr.modify(|_, w| w.cc1if().clear_bit());
    }

    fn pend_interrupt() {
        NVIC::pend(Interrupt::TIM2)
    }

    fn enable_timer() {
        Self::register().dier.modify(|_, w| w.cc1ie().set_bit());
    }

    fn disable_timer() {
        Self::register().dier.modify(|_, w| w.cc1ie().clear_bit());
    }

    fn on_interrupt() {
        let reg = Self::register();
        let sr = &reg.sr;

        if sr.read().uif().bit_is_set() {
            HALF_PERIOD_COUNTER.fetch_add(1, Ordering::SeqCst);
            sr.modify(|_, w| w.uif().clear_bit());
        }
        if sr.read().cc2if().bit_is_set() {
            HALF_PERIOD_COUNTER.fetch_add(1, Ordering::SeqCst);
            sr.modify(|_, w| w.cc2if().clear_bit());
        }
    }
}

#[macro_export]
macro_rules! create_stm32_tim2_monotonic_token {
    () => {{
        #[no_mangle]
        #[allow(non_snake_case)]
        unsafe extern "C" fn TIM2() {
            $crate::clock::Clock::__tq().on_monotonic_interrupt();
        }

        pub struct Tim2Token;

        unsafe impl rtic_monotonics::InterruptToken<$crate::clock::Clock> for Tim2Token {}

        Tim2Token
    }};
}
