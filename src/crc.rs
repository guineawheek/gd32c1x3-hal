
use core::{mem::MaybeUninit, ptr::copy_nonoverlapping};

use crate::pac::{Crc, Rcu};

use crate::rcu::Enable;

pub trait CrcExc {
    fn constrain(self) -> Crc32;
}

///
/// ethernet-compatible crc32.
/// 
/// this is comparable to the peripheral you get on stm32fxxx devices.
pub struct Crc32 {
    pub(crate) regs: Crc
}

pub trait CrcExt {
    fn constrain(self) -> Crc32;
}

impl CrcExt for Crc {
    fn constrain(self) -> Crc32 {
        let rcu = {unsafe{&(*Rcu::ptr())}};
        Crc::enable(rcu);
        Crc32 { regs: self }
    }
}

impl Crc32 {

    /// resets the crc32 peripheral to its initial state.
    pub fn init(&mut self) {
        self.regs.ctl().modify(|_, w| w.rst().set_bit());
    }

    pub fn update(&mut self, data: &[u32]) -> u32 {
        for word in data {
            self.regs.data().write(|w| w.data().bits(*word));
        }
        self.regs.data().read().bits()
    }

    /// calculates a CRC for the slice.
    /// 
    /// this uses crc32/mpeg2 like most stm32-like crc32 peripherals.
    /// if the input slice is not 4-byte aligned it will be implicitly zero-padded at the end to the next 4-byte boundary.
    pub fn update_bytes(&mut self, data: &[u8]) -> u32 {

        let chunks = data.chunks_exact(4);
        let remainder = chunks.remainder();

        // For each full chunk of four bytes...
        chunks.for_each(|chunk| unsafe {
            // Create an uninitialized scratch buffer. We make it uninitialized
            // to avoid re-zeroing this data inside of the loop.
            let mut scratch: MaybeUninit<[u8; 4]> = MaybeUninit::uninit();

            // Copy the (potentially unaligned) bytes from the input chunk to
            // our scratch bytes. We cast the `scratch` buffer from a `*mut [u8; 4]`
            // to a `*mut u8`.
            let src: *const u8 = chunk.as_ptr();
            let dst: *mut u8 = scratch.as_mut_ptr().cast::<u8>();
            copy_nonoverlapping(src, dst, 4);

            // Mark the scratch bytes as initialized, and then convert it to a
            // native-endian u32. Feed this into the CRC peripheral
            self.regs.data().write(|w| w.bits(u32::from_be_bytes(scratch.assume_init())));
        });
        // If we had a non-multiple of four bytes...
        if !remainder.is_empty() {
            // Create a zero-filled scratch buffer, and copy the data in
            let mut scratch = [0u8; 4];

            // NOTE: We are on a little-endian processor. This means that copying
            // the 0..len range fills the LEAST significant bytes, leaving the
            // MOST significant bytes as zeroes
            scratch[..remainder.len()].copy_from_slice(remainder);
            self.regs.data().write(|w| unsafe {w.bits(u32::from_be_bytes(scratch))});
        }

        self.regs.data().read().bits()

    }

    pub fn release(self) -> Crc {
        self.regs
    }
}