//! Inter-Integrated Circuit (I2C) bus

// This document describes a correct i2c implementation and is what
// parts of this code is based on
// https://www.st.com/content/ccc/resource/technical/document/application_note/5d/ae/a3/6f/08/69/4e/9b/CD00209826.pdf/files/CD00209826.pdf/jcr:content/translations/en.CD00209826.pdf

use crate::afio::PCF0;
use crate::gpio::{self, Alternate, OpenDrain};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::rcu::{BusClock, Clocks, Enable, Reset};
use crate::time::{kHz, Hertz};
use core::ops::Deref;
use crate::pac::{self, I2c0, I2c1, Rcu};

pub mod blocking;
pub use blocking::BlockingI2c;

/// I2C error
#[derive(Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// No ack received
    Acknowledge,
    /// Overrun/underrun
    Overrun,
    // Pec, // SMBUS mode only
    Timeout,
    // Alert, // SMBUS mode only
}

#[derive(Debug, Eq, PartialEq)]
pub enum DutyCycle {
    Ratio2to1,
    Ratio16to9,
}

#[derive(Debug, PartialEq, Eq)]
pub enum Mode {
    Standard {
        frequency: Hertz,
    },
    Fast {
        frequency: Hertz,
        duty_cycle: DutyCycle,
    },
}

impl Mode {
    pub fn standard(frequency: Hertz) -> Self {
        Mode::Standard { frequency }
    }

    pub fn fast(frequency: Hertz, duty_cycle: DutyCycle) -> Self {
        Mode::Fast {
            frequency,
            duty_cycle,
        }
    }

    pub fn get_frequency(&self) -> Hertz {
        match *self {
            Mode::Standard { frequency } => frequency,
            Mode::Fast { frequency, .. } => frequency,
        }
    }
}

impl From<Hertz> for Mode {
    fn from(frequency: Hertz) -> Self {
        if frequency <= kHz(100) {
            Self::Standard { frequency }
        } else {
            Self::Fast {
                frequency,
                duty_cycle: DutyCycle::Ratio2to1,
            }
        }
    }
}

/// Helper trait to ensure that the correct I2C pins are used for the corresponding interface
pub trait Pins<I2C> {
    const REMAP: bool;
}

impl Pins<I2c0>
    for (
        gpio::PB6<Alternate<OpenDrain>>,
        gpio::PB7<Alternate<OpenDrain>>,
    )
{
    const REMAP: bool = false;
}

impl Pins<I2c0>
    for (
        gpio::PB8<Alternate<OpenDrain>>,
        gpio::PB9<Alternate<OpenDrain>>,
    )
{
    const REMAP: bool = true;
}

impl Pins<I2c1>
    for (
        gpio::PB10<Alternate<OpenDrain>>,
        gpio::PB11<Alternate<OpenDrain>>,
    )
{
    const REMAP: bool = false;
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
    mode: Mode,
    pclk1: Hertz,
}

pub trait Instance:
    crate::Sealed + Deref<Target = pac::i2c0::RegisterBlock> + Enable + Reset + BusClock
{
}

impl Instance for I2c0 {}
impl Instance for I2c1 {}

impl<PINS> I2c<I2c0, PINS> {
    /// Creates a generic I2C1 object on pins PB6 and PB7 or PB8 and PB9 (if remapped)
    pub fn i2c0<M: Into<Mode>>(
        i2c: I2c0,
        pins: PINS,
        pcf0: &mut PCF0,
        mode: M,
        clocks: Clocks,
    ) -> Self
    where
        PINS: Pins<I2c0>,
    {
        pcf0.modify_pcf0(|_, w| w.i2c0_remap().bit(PINS::REMAP));
        I2c::<I2c0, _>::configure(i2c, pins, mode, clocks)
    }
}

impl<PINS> I2c<I2c1, PINS> {
    /// Creates a generic I2C2 object on pins PB10 and PB11 using the embedded-hal `BlockingI2c` trait.
    pub fn i2c1<M: Into<Mode>>(i2c: I2c1, pins: PINS, mode: M, clocks: Clocks) -> Self
    where
        PINS: Pins<I2c1>,
    {
        I2c::<I2c1, _>::configure(i2c, pins, mode, clocks)
    }
}

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Instance,
{
    /// Configures the I2C peripheral to work in master mode
    fn configure<M: Into<Mode>>(i2c: I2C, pins: PINS, mode: M, clocks: Clocks) -> Self {
        let mode = mode.into();
        let rcu = unsafe { &(*Rcu::ptr()) };
        I2C::enable(rcu);
        I2C::reset(rcu);

        let pclk1 = I2C::clock(&clocks);

        assert!(mode.get_frequency() <= kHz(400));

        let mut i2c = I2c {
            i2c,
            pins,
            mode,
            pclk1,
        };
        i2c.init();
        i2c
    }
}

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Instance,
{
    /// Initializes I2C. Configures the `I2C_TRISE`, `I2C_CRX`, and `I2C_CCR` registers
    /// according to the system frequency and I2C mode.
    fn init(&mut self) {
        let freq = self.mode.get_frequency();
        let pclk1_mhz = self.pclk1.to_MHz() as u16;

        self.i2c
            .ctl1()
            .write(|w| unsafe { w.i2cclk().bits(pclk1_mhz as u8) });
        self.i2c.ctl0().write(|w| w.i2cen().clear_bit());

        match self.mode {
            Mode::Standard { .. } => {
                self.i2c
                    .rt()
                    .write(|w| unsafe { w.risetime().bits((pclk1_mhz + 1) as u8) });
                self.i2c
                    .ckcfg()
                    .write(|w| w.clkc().bits(((self.pclk1 / (freq * 2)) as u16).max(4)));
            }
            Mode::Fast { ref duty_cycle, .. } => {
                self.i2c
                    .rt()
                    .write(|w| unsafe { w.risetime().bits((pclk1_mhz * 300 / 1000 + 1) as u8) });

                self.i2c.ckcfg().write(|w| {
                    let (freq, duty) = match duty_cycle {
                        DutyCycle::Ratio2to1 => (((self.pclk1 / (freq * 3)) as u16).max(1), false),
                        DutyCycle::Ratio16to9 => (((self.pclk1 / (freq * 25)) as u16).max(1), true),
                    };

                    w.clkc().bits(freq).dtcy().bit(duty).fast().set_bit()
                });
            }
        };

        self.i2c.ctl0().modify(|_, w| w.i2cen().set_bit());
    }

    /// Perform an I2C software reset
    fn reset(&mut self) {
        self.i2c
            .ctl0()
            .write(|w| w.i2cen().set_bit().sreset().set_bit());
        self.i2c.ctl0().reset();
        self.init();
    }

    /// Generate START condition
    fn send_start(&mut self) {
        self.i2c.ctl0().modify(|_, w| w.start().set_bit());
    }

    /// Sends the (7-Bit) address on the I2C bus. The 8th bit on the bus is set
    /// depending on wether it is a read or write transfer.
    fn send_addr(&self, addr: u8, read: bool) {
        self.i2c
            .data()
            .write(|w| w.trb().bits(addr << 1 | (u8::from(read))));
    }

    /// Generate STOP condition
    fn send_stop(&self) {
        self.i2c.ctl0().modify(|_, w| w.stop().set_bit());
    }

    /// Releases the I2C peripheral and associated pins
    pub fn release(self) -> (I2C, PINS) {
        (self.i2c, self.pins)
    }
}
