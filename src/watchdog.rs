//! Watchdog peripherals
//! 
//! (yoinked from gd32vf103xx-hal)

use fugit::ExtU32;

use crate::{
    hal::watchdog::{Watchdog, WatchdogEnable},
    pac::{Dbg, Fwdgt},
    time::MilliSeconds,
};

/// Wraps the Free Watchdog Timer (FWDGT) peripheral
pub struct FreeWatchdog {
    fwdgt: Fwdgt,
}

const IRC40K_KHZ: u32 = 40;

const MAX_PR: u8 = 8;
const MAX_RL: u16 = 0xFFF;

const CMD_ACCESS: u16 = 0x5555;
const CMD_RELOAD: u16 = 0xAAAA;
const CMD_START: u16 = 0xCCCC;

impl FreeWatchdog {
    /// Wrap and start the watchdog
    pub fn new(fwdgt: Fwdgt) -> Self {
        FreeWatchdog { fwdgt }
    }

    /// Free watchdog stopped when core is halted
    pub fn stop_on_debug(&self, dbg: &Dbg, stop: bool) {
        dbg.ctl0().modify(|_, w| w.fwdgt_hold().bit(stop));
    }

    fn setup(&self, timeout_ms: u32) {
        let mut pr = 0;
        while pr < MAX_PR && Self::timeout_period(pr, MAX_RL) < timeout_ms {
            pr += 1;
        }

        let max_period = Self::timeout_period(pr, MAX_RL);
        let max_rl = u32::from(MAX_RL);
        let rl = (timeout_ms * max_rl / max_period).min(max_rl) as u16;

        self.access_registers(|fwdgt| {
            fwdgt.psc().modify(|_, w| w.psc().bits(pr));
            fwdgt.rld().modify(|_, w| w.rld().bits(rl));
        });
    }

    fn is_pr_updating(&self) -> bool {
        self.fwdgt.stat().read().pud().bit()
    }

    /// Returns the interval in ms
    pub fn interval(&self) -> MilliSeconds {
        while self.is_pr_updating() {}

        let pr = self.fwdgt.psc().read().psc().bits();
        let rl = self.fwdgt.rld().read().rld().bits();
        let ms = Self::timeout_period(pr, rl);

        ms.millis()
    }

    /// pr: Prescaler divider bits, rl: reload value
    ///
    /// Returns ms
    const fn timeout_period(pr: u8, rl: u16) -> u32 {
        let divider: u32 = match pr {
            0b000 => 4,
            0b001 => 8,
            0b010 => 16,
            0b011 => 32,
            0b100 => 64,
            0b101 => 128,
            0b110 => 256,
            0b111 => 256,
            _ => panic!("Invalid FWDGT prescaler divider"),
        };
        ((rl as u32) + 1) * divider / IRC40K_KHZ
    }

    fn access_registers<A, F: FnMut(&Fwdgt) -> A>(&self, mut f: F) -> A {
        // Unprotect write access to registers
        self.fwdgt
            .ctl()
            .write(|w| unsafe { w.cmd().bits(CMD_ACCESS) });
        let a = f(&self.fwdgt);

        // Protect again
        self.fwdgt
            .ctl()
            .write(|w| unsafe { w.cmd().bits(CMD_RELOAD) });
        a
    }
}

impl WatchdogEnable for FreeWatchdog {
    type Time = MilliSeconds;

    fn start<T: Into<Self::Time>>(&mut self, period: T) {
        self.setup(period.into().to_millis());

        self.fwdgt.ctl().write(|w| unsafe { w.cmd().bits(CMD_START) });
    }
}

impl Watchdog for FreeWatchdog {
    fn feed(&mut self) {
        self.fwdgt
            .ctl()
            .write(|w| unsafe { w.cmd().bits(CMD_RELOAD) });
    }
}