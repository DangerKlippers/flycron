use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _; // global logger

use panic_probe as _;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", { COUNT.fetch_add(1, Ordering::Relaxed) });

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
#[allow(dead_code)]
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
