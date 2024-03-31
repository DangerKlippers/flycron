use core::{
    arch::asm,
    ptr,
    sync::atomic::{AtomicUsize, Ordering},
};
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
const BOOTLOADER_FLAG_ADDR: u32 = 0x2000_6ffc; // 0 and max get clobbered at init
const BOOTLOADER_FLAG_MAGIC: u32 = 0xf026_69ef;
const BOOTLOADER_REBOOT_MAGIC: u32 = 0xfeeb_beef;
const BOOTLOADER_ST_ADDR: u32 = 0x1fff_0000;

pub fn reset_to_bootloader() -> ! {
    unsafe {
        ptr::write(BOOTLOADER_FLAG_ADDR as *mut u32, BOOTLOADER_FLAG_MAGIC);
        asm!("nop"); // write needs time to hit ram
    }
    cortex_m::peripheral::SCB::sys_reset();
}

pub fn bootloader_check() {
    unsafe {
        if ptr::read(BOOTLOADER_FLAG_ADDR as *const u32) == BOOTLOADER_FLAG_MAGIC {
            ptr::write(BOOTLOADER_FLAG_ADDR as *mut u32, 0);
            let initial_sp = ptr::read(BOOTLOADER_ST_ADDR as *const u32);
            let start_addr = ptr::read((BOOTLOADER_ST_ADDR + 4) as *const u32);
            asm!("mov sp, {0}\nbx {1}", in(reg) initial_sp, in(reg) start_addr);
        }
        // always chip reset once on boot to fix bootloader re-enumeration
        if ptr::read(BOOTLOADER_FLAG_ADDR as *const u32) != BOOTLOADER_REBOOT_MAGIC {
            ptr::write(BOOTLOADER_FLAG_ADDR as *mut u32, BOOTLOADER_REBOOT_MAGIC);
            asm!("nop");
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}

#[cfg(all(not(feature = "crystal-25mhz"), not(feature = "crystal-8mhz")))]
compile_error!("Must specify crystal-25mhz or crystal-8mhz feature!");
