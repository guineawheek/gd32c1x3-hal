//! This module allows Timer peripherals to be configured as pwm input.
//! In this mode, the timer sample a squared signal to find it's frequency and duty cycle.

use core::marker::PhantomData;
use core::mem;

use crate::pac::Dbg;
use crate::pac::{Timer0, Timer1, Timer2, Timer3, Timer4};

use crate::afio::PCF0;
use crate::gpio::{self, Input};
use crate::rcu::{BusTimerClock, Clocks};
use crate::time::Hertz;
use crate::timer::Timer;

pub trait Pins<REMAP> {}

use super::pins::{sealed::Remap, CPin};

impl<TIM, REMAP, P1, P2, MODE1, MODE2> Pins<REMAP> for (P1, P2)
where
    REMAP: Remap<Periph = TIM>,
    P1: CPin<REMAP, 0> + gpio::PinExt<Mode = Input<MODE1>>,
    P2: CPin<REMAP, 1> + gpio::PinExt<Mode = Input<MODE2>>,
{
}

/// PWM Input
pub struct PwmInput<TIM, REMAP, PINS> {
    _timer: PhantomData<TIM>,
    _remap: PhantomData<REMAP>,
    _pins: PhantomData<PINS>,
}

/// How the data is read from the timer
pub enum ReadMode {
    /// Return the latest captured data
    Instant,
    /// Wait for one period of the signal before computing the frequency and the duty cycle
    /// The microcontroller will be halted for at most two period of the input signal.
    WaitForNextCapture,
}

/// The error returned when reading a frequency from a timer
#[derive(Debug)]
pub enum Error {
    /// The signal frequency is too low to be sampled by the current timer configuration
    FrequencyTooLow,
}

/// Which frequency the timer will try to sample
pub enum Configuration {
    /// In this mode an algorithm calculates the optimal value for the autoreload register and the
    /// prescaler register in order to be able to sample a wide range of frequency, at the expense
    /// of resolution.
    ///
    /// The minimum frequency that can be sampled is 20% the provided frequency.
    ///
    /// Use this mode if you do not know what to choose.
    Frequency(Hertz),

    /// In this mode an algorithm calculates the optimal value for the autoreload register and the
    /// prescaler register in order to sample the duty cycle with a high resolution.
    /// This will limit the frequency range where the timer can operate.
    ///
    /// The minimum frequency that can be sampled is 90% the provided frequency
    DutyCycle(Hertz),

    /// In this mode an algorithm calculates the optimal value for the autoreload register and the
    /// prescaler register in order to be able to sample signal with a frequency higher than the
    /// provided value : there is no margin for lower frequencies.
    RawFrequency(Hertz),

    /// In this mode, the provided arr and presc are directly programmed in the register.
    RawValues { arr: u16, presc: u16 },
}
impl Timer<Timer0> {
    pub fn pwm_input<REMAP, PINS>(
        mut self,
        pins: PINS,
        pcf0: &mut PCF0,
        dbg: &mut Dbg,
        mode: Configuration,
    ) -> PwmInput<Timer0, REMAP, PINS>
    where
        REMAP: Remap<Periph = Timer0>,
        PINS: Pins<REMAP>,
    {
        REMAP::remap(pcf0);
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim0(tim, pins, clk, mode)
    }
}
impl Timer<Timer1> {
    pub fn pwm_input<REMAP, PINS>(
        mut self,
        pins: PINS,
        pcf0: &mut PCF0,
        dbg: &mut Dbg,
        mode: Configuration,
    ) -> PwmInput<Timer1, REMAP, PINS>
    where
        REMAP: Remap<Periph = Timer1>,
        PINS: Pins<REMAP>,
    {
        REMAP::remap(pcf0);
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim1(tim, pins, clk, mode)
    }
}

impl Timer<Timer2> {
    pub fn pwm_input<REMAP, PINS>(
        mut self,
        pins: PINS,
        pcf0: &mut PCF0,
        dbg: &mut Dbg,
        mode: Configuration,
    ) -> PwmInput<Timer2, REMAP, PINS>
    where
        REMAP: Remap<Periph = Timer2>,
        PINS: Pins<REMAP>,
    {
        REMAP::remap(pcf0);
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim2(tim, pins, clk, mode)
    }
}

impl Timer<Timer3> {
    pub fn pwm_input<REMAP, PINS>(
        mut self,
        pins: PINS,
        pcf0: &mut PCF0,
        dbg: &mut Dbg,
        mode: Configuration,
    ) -> PwmInput<Timer3, REMAP, PINS>
    where
        REMAP: Remap<Periph = Timer3>,
        PINS: Pins<REMAP>,
    {
        REMAP::remap(pcf0);
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim3(tim, pins, clk, mode)
    }
}

impl Timer<Timer4> {
    pub fn pwm_input<REMAP, PINS>(
        mut self,
        pins: PINS,
        pcf0: &mut PCF0,
        dbg: &mut Dbg,
        mode: Configuration,
    ) -> PwmInput<Timer4, REMAP, PINS>
    where
        REMAP: Remap<Periph = Timer4>,
        PINS: Pins<REMAP>,
    {
        REMAP::remap(pcf0);
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim4(tim, pins, clk, mode)
    }
}

/// Courtesy of @TeXitoi (https://github.com/stm32-rs/stm32f1xx-hal/pull/10#discussion_r259535503)
fn compute_arr_presc(freq: u32, clock: u32) -> (u16, u16) {
    if freq == 0 {
        return (core::u16::MAX, core::u16::MAX);
    }
    let presc = clock / freq.saturating_mul(core::u16::MAX as u32 + 1);
    let arr = clock / freq.saturating_mul(presc + 1);
    (core::cmp::max(1, arr as u16), presc as u16)
}
macro_rules! hal {
    ($($TIMX:ident: ($timX:ident),)+) => {
        $(
            fn $timX<REMAP, PINS>(
                tim: $TIMX,
                _pins: PINS,
                clk: Hertz,
                mode : Configuration,
            ) -> PwmInput<$TIMX, REMAP, PINS>
            where
                REMAP: Remap<Periph = $TIMX>,
                PINS: Pins<REMAP>,
            {
                use Configuration::*;
                // Disable capture on both channels during setting
                // (for Channel X bit is CCXE)
                tim.chctl2().modify(|_,w| w.ch0en().clear_bit().ch1en().clear_bit()
                                       .ch0p().clear_bit().ch1p().set_bit());

                // Define the direction of the channel (input/output)
                // and the used input
                tim.chctl0_input().modify( |_,w| w.ch0ms().ci0().ch1ms().ci0());

                tim.dmainten().write(|w| w.ch0ie().set_bit());

                // Configure slave mode control register
                // Selects the trigger input to be used to synchronize the counter
                // 101: Filtered Timer Input 1 (TI1FP1)
                // ---------------------------------------
                // Slave Mode Selection :
                //  100: Reset Mode - Rising edge of the selected trigger input (TRGI)
                //  reinitializes the counter and generates an update of the registers.
                tim.smcfg().modify( |_,w| unsafe {w.trgs().bits(0b101).smc().bits(0b100)});

                match mode {
                    Frequency(f)  => {
                        let freq = f.raw();
                        let max_freq = if freq > 5 {freq/5} else {1};
                        let (arr,presc) = compute_arr_presc(max_freq, clk.raw());
                        tim.car().write(|w| w.car().bits(arr));
                        tim.psc().write(|w| w.psc().bits(presc));
                    },
                    DutyCycle(f) => {
                        let freq = f.raw();
                        let max_freq = if freq > 2 {freq/2 + freq/4 + freq/8} else {1};
                        let (arr,presc) = compute_arr_presc(max_freq, clk.raw());
                        tim.car().write(|w| w.car().bits(arr));
                        tim.psc().write(|w| w.psc().bits(presc));
                    },
                    RawFrequency(f) => {
                        let freq = f.raw();
                        let (arr,presc) = compute_arr_presc(freq, clk.raw());
                        tim.car().write(|w| w.car().bits(arr));
                        tim.psc().write(|w| w.psc().bits(presc));
                    }
                    RawValues{arr, presc} => {
                        tim.car().write(|w| w.car().bits(arr));
                        tim.psc().write(|w| w.psc().bits(presc));
                    }
                }

                // Enable Capture on both channels
                tim.chctl2().modify(|_,w| w.ch0en().set_bit().ch1en().set_bit());

                tim.ctl0().modify(|_,w| w.cen().set_bit());
                unsafe { mem::MaybeUninit::uninit().assume_init() }
            }

            impl<REMAP, PINS> PwmInput<$TIMX, REMAP, PINS>
            where
                REMAP: Remap<Periph = $TIMX>,
                PINS: Pins<REMAP>,
            {
                /// Return the frequency sampled by the timer
                pub fn read_frequency(&self, mode : ReadMode, clocks : &Clocks) -> Result<Hertz,Error> {
                    if let ReadMode::WaitForNextCapture = mode {
                        self.wait_for_capture();
                    }

                    let presc = unsafe { (*$TIMX::ptr()).psc().read().bits() as u16};
                    let ccr1 = unsafe { (*$TIMX::ptr()).ch0cv().read().bits() as u16};

                    // Formulas :
                    //
                    // F_timer = F_pclk / (PSC+1)*(ARR+1)
                    // Frac_arr = (CCR1+1)/(ARR+1)
                    // F_signal = F_timer/Frac_arr
                    // <=> F_signal = [(F_plck)/((PSC+1)*(ARR+1))] * [(ARR+1)/(CCR1+1)]
                    // <=> F_signal = F_pclk / ((PSC+1)*(CCR1+1))
                    //
                    // Where :
                    // * PSC is the prescaler register
                    // * ARR is the auto-reload register
                    // * F_timer is the number of time per second where the timer overflow under normal
                    // condition
                    //
                    if ccr1 == 0 {
                        Err(Error::FrequencyTooLow)
                    } else {
                        let clk = <$TIMX>::timer_clock(&clocks);
                        Ok(clk/((presc+1) as u32*(ccr1 + 1)as u32))
                    }
                }

                /// Return the duty in the form of a fraction : (duty_cycle/period)
                pub fn read_duty(&self, mode : ReadMode) -> Result<(u16,u16),Error> {
                    if let ReadMode::WaitForNextCapture = mode {
                        self.wait_for_capture();
                    }

                    // Formulas :
                    // Duty_cycle = (CCR2+1)/(CCR1+1)
                    let ccr1 = unsafe { (*$TIMX::ptr()).ch0cv().read().bits() as u16};
                    let ccr2 = unsafe { (*$TIMX::ptr()).ch1cv().read().bits() as u16};
                    if ccr1 == 0 {
                        Err(Error::FrequencyTooLow)
                    } else {
                        Ok((ccr2,ccr1))
                    }
                }

                /// Wait until the timer has captured a period
                fn wait_for_capture(&self) {
                    unsafe { (*$TIMX::ptr()).intf().write(|w| w.upif().clear_bit().ch0if().clear_bit().ch0of().clear_bit())};
                    while unsafe { (*$TIMX::ptr()).intf().read().ch0if().bit_is_clear()} {}
                }
            }
        )+
    }
}
hal! {
    Timer0: (tim0),
}

hal! {
    Timer1: (tim1),
}

hal! {
    Timer2: (tim2),
    Timer3: (tim3),
}

hal! {
    Timer4: (tim4),
}
