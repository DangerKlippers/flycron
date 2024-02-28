use crate::hal;

pub fn get_chipid() -> &'static str {
    static mut CHIP_ID: [u8; 32] = [0; 32];

    fn hexchar(n: u8) -> u8 {
        if n >= 0xA {
            n + 0x41 - 0xA
        } else {
            n + 0x30
        }
    }

    unsafe {
        let uid: &[u8; 12] = core::mem::transmute(hal::signature::Uid::get());
        if CHIP_ID[0] == 0 {
            for (i, b) in uid.iter().enumerate() {
                CHIP_ID[i * 2] = hexchar((b >> 4) & 0xF);
                CHIP_ID[i * 2 + 1] = hexchar(b & 0xF);
            }
        }
        core::str::from_utf8_unchecked(&*core::ptr::addr_of!(CHIP_ID))
    }
}
