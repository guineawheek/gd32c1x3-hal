//! Flash memory

use crate::pac::{fmc, Fmc};
use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};
/// Extension trait to constrain the FLASH peripheral
pub trait FMCExt {
    /// Constrains the FLASH peripheral to play nicely with the other abstractions
    fn constrain(self) -> Parts;
}

impl FMCExt for Fmc {
    fn constrain(self) -> Parts {
        Parts { ws: WS { _0: () }, flash : Flash { _0: () } }
    }
}

/// Constrained FLASH peripheral
pub struct Parts {
    /// Opaque ACR register
    pub ws: WS,
    pub flash: Flash
}

/// Opaque ACR register
pub struct WS {
    _0: (),
}

impl WS {
    pub(crate) fn ws(&mut self) -> &fmc::Ws {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { (*Fmc::ptr()).ws() }
    }
}

pub struct Flash {
    _0: (),

}

impl Flash {
    const FLASH_BASE : u32 = 0x0800_0000;
    const FLASH_SIZE : usize = 0x20000;
    const MAX_ADDR : u32 = Flash::FLASH_BASE + (Flash::FLASH_SIZE as u32) - 1;
    const READ_SIZE: usize = 0x1;
    const WRITE_SIZE: usize = 0x4;
    const ERASE_SIZE: usize = 1024;

    fn unlock(&mut self) {
        let fmc: &fmc::RegisterBlock = unsafe { &(*Fmc::ptr()) };
        if fmc.ctl().read().lk().bit_is_set() {
            fmc.key().write(|w| unsafe{w.bits(0x45670123)});
            fmc.key().write(|w| unsafe{w.bits(0xCDEF89AB)});
        }
    }

    fn lock(&mut self) {
        let fmc: &fmc::RegisterBlock = unsafe { &(*Fmc::ptr()) };
        if fmc.ctl().read().lk().bit_is_clear() {
            fmc.ctl().modify(|_, w| w.lk().set_bit());
        }
    }

    fn program_word(&mut self, offset : u32, word : u32) {
        let fmc: &fmc::RegisterBlock = unsafe { &(*Fmc::ptr()) };
        while fmc.stat().read().busy().bit_is_set() {}
        fmc.ctl().modify(|_, w| w.pg().set_bit());
        let write_ptr = unsafe { core::mem::transmute::<usize,*mut u32>((Flash::FLASH_BASE + offset) as usize) };
        unsafe { core::ptr::write_volatile(write_ptr, word); }
        while fmc.stat().read().busy().bit_is_set() {}
        fmc.ctl().modify(|_, w|w.pg().clear_bit());
    }

    fn program_dword(&mut self, offset : u32, dword : u64) {
        let fmc: &fmc::RegisterBlock = unsafe { &(*Fmc::ptr()) };
        while fmc.stat().read().busy().bit_is_set() {}
        fmc.ws().modify(|_, w| w.pgw().set_bit());
        fmc.ctl().modify(|_, w| w.pg().set_bit());
        let write_ptr_l = unsafe { core::mem::transmute::<usize, *mut u32>((Flash::FLASH_BASE + offset) as usize) };
        let write_ptr_h = unsafe { core::mem::transmute::<usize, *mut u32>((Flash::FLASH_BASE + offset + 4) as usize) };
        let (word0, word1) = ((dword & 0xffffffff) as u32, (dword >> 32) as u32);
        unsafe { core::ptr::write_volatile(write_ptr_l, word0 ); }
        unsafe { core::ptr::write_volatile(write_ptr_h, word1 ); }
        while fmc.stat().read().busy().bit_is_set() {}
        fmc.ctl().modify(|_, w|w.pg().clear_bit());
        fmc.ws().modify(|_, w| w.pgw().clear_bit());
    }

    fn erase_page(&mut self, offset : u32) {
        let fmc: &fmc::RegisterBlock = unsafe { &(*Fmc::ptr()) };
        while fmc.stat().read().busy().bit_is_set() {}
        let erase_addr = Flash::FLASH_BASE + offset;

        fmc.ctl().modify(|_, w| w.per().set_bit());
        fmc.addr().write(|w| w.addr().bits(erase_addr));
        fmc.ctl().modify(|_, w| w.start().set_bit());
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        while fmc.stat().read().busy().bit_is_set() {}
        fmc.ctl().modify(|_, w| w.per().clear_bit());

    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FlashError {
    WriteProtected,
    ProgramError,
    OutOfBounds,
    NotAligned,
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            FlashError::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            FlashError::NotAligned => NorFlashErrorKind::NotAligned,
            FlashError::WriteProtected => NorFlashErrorKind::Other,
            FlashError::ProgramError => NorFlashErrorKind::Other,
        }
    }
}

impl ErrorType for Flash {
    type Error = FlashError;
}

impl ReadNorFlash for Flash {
    const READ_SIZE: usize = Flash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let mut addr = Flash::FLASH_BASE + offset;
        if addr > Flash::MAX_ADDR || (addr + bytes.len() as u32) > (Flash::MAX_ADDR + 1) {
            return Err(FlashError::OutOfBounds);
        }

        // this actually produces Reasonable Assembly somehow.
        // we use read_volatile because eliding these calls can lead to Odd Behavior when dealing with pointers/offsets
        // derived from consts/statics that happen to live in flash; the compiler may assume that they are immutable
        // even if the underlying data is mutated.

        let mut biter = bytes.chunks_exact_mut(4);
        for b4 in &mut biter {
            let bbuf = unsafe { core::ptr::read_volatile(addr as *const u32) }.to_ne_bytes();
            b4[0] = bbuf[0];
            b4[1] = bbuf[1];
            b4[2] = bbuf[2];
            b4[3] = bbuf[3];
            addr += 4;
        }
        for b in biter.into_remainder() {
            *b = unsafe { core::ptr::read_volatile(addr as *const u8) };
            addr += 1;
        }

        Ok(())
    }

    fn capacity(&self) -> usize {
        131072
    }
}

impl NorFlash for Flash
{
    const WRITE_SIZE: usize = Flash::WRITE_SIZE;
    const ERASE_SIZE: usize = Flash::ERASE_SIZE;


    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from > Flash::MAX_ADDR {
            return Err(Self::Error::OutOfBounds);
        }

        if to > (Flash::MAX_ADDR + 1) {
            return Err(Self::Error::OutOfBounds);
        }

        if from % Self::ERASE_SIZE as u32 != 0 || to % Self::ERASE_SIZE as u32 != 0 {
            return Err(Self::Error::NotAligned);
        }
        self.unlock();

        let range = (from / Self::ERASE_SIZE as u32)..(to / Self::ERASE_SIZE as u32);
        for page in range {
            self.erase_page(page * 1024);
        }
        self.lock();
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if bytes.len() % Self::WRITE_SIZE != 0 {
            return Err(Self::Error::NotAligned);
        }

        if offset as usize % Self::WRITE_SIZE != 0 {
            return Err(Self::Error::NotAligned);
        }

        if (offset as usize) + bytes.len() > Flash::FLASH_SIZE {
            return Err(Self::Error::OutOfBounds);
        }

        self.unlock();

        let mut byte_chunks = bytes.chunks_exact(8);
        let mut i = 0u32;
        for b in byte_chunks.by_ref() {
            self.program_dword(offset + i, u64::from_ne_bytes(b.try_into().unwrap()));
            i += 8;
        }

        // just so happens we checked if it's either 4 or 8-byte aligned.
        let r = byte_chunks.remainder();
        if r.len() == 4 {
            self.program_word(offset + i, u32::from_ne_bytes(r.try_into().unwrap()));
        }
        self.lock();
        Ok(())
    }
}

impl embedded_storage_async::nor_flash::NorFlash for Flash {

    // while theoretically possible to async wait on the fmc.stat() register combined with the fmc interrupt,
    // it's unknown if it's worth doing.
    // so for now we just provide the sync impls.

    const WRITE_SIZE: usize = Flash::WRITE_SIZE;
    const ERASE_SIZE: usize = Flash::ERASE_SIZE;


    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        embedded_storage::nor_flash::NorFlash::erase(self, from, to)

    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        embedded_storage::nor_flash::NorFlash::write(self, offset, bytes)
    }

}

impl embedded_storage_async::nor_flash::ReadNorFlash for Flash {
    const READ_SIZE: usize = Flash::READ_SIZE;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        embedded_storage::nor_flash::ReadNorFlash::read(self, offset, bytes)
    }

    fn capacity(&self) -> usize {
        131072
    }
}
