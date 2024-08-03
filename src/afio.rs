//! # Alternate Function I/Os

use crate::pac::{afio, Afio, Rcu};

use crate::rcu::{Enable, Reset};

use crate::gpio::{
    Debugger, Floating, Input, PA15, {PB3, PB4},
};

pub trait AfioExt {
    fn constrain(self) -> Parts;
}

impl AfioExt for Afio {
    fn constrain(self) -> Parts {
        let rcu = unsafe { &(*Rcu::ptr()) };
        Afio::enable(rcu);
        Afio::reset(rcu);

        Parts {
            ec: EC { _0: () },
            pcf0: PCF0 {
                _0: (),
                debug_state: DebugState::FullyEnabled,
            },
            extiss0: EXTISS0 { _0: () },
            extiss1: EXTISS1 { _0: () },
            extiss2: EXTISS2 { _0: () },
            extiss3: EXTISS3 { _0: () },
            pcf1: PCF1 { _0: () },
        }
    }
}

/// HAL wrapper around the AFIO registers
///
/// Aquired by calling [constrain](trait.AfioExt.html#constrain) on the [AFIO
/// registers](../pac/struct.AFIO.html)
///
/// ```rust
/// let p = pac::Peripherals::take().unwrap();
/// let mut rcc = p.RCC.constrain();
/// let mut afio = p.AFIO.constrain();
pub struct Parts {
    pub ec: EC,
    pub pcf0: PCF0,
    pub extiss0: EXTISS0,
    pub extiss1: EXTISS1,
    pub extiss2: EXTISS2,
    pub extiss3: EXTISS3,
    pub pcf1: PCF1,
}

pub struct EC {
    _0: (),
}

impl EC {
    pub fn ec(&mut self) -> &afio::Ec {
        unsafe { (*Afio::ptr()).ec() }
    }
}

pub enum DebugState {
    FullyEnabled,
    JtagNoTrstEnabled,
    SwdEnabled,
    DebugDisabled
}

/// AF remap and debug I/O configuration register (MAPR)
///
/// Aquired through the [Parts](struct.Parts.html) struct.
///
/// ```rust
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcc = dp.RCC.constrain();
/// let mut afio = dp.AFIO.constrain();
/// function_using_mapr(&mut afio.mapr);
/// ```
pub struct PCF0 {
    _0: (),
    debug_state: DebugState,
}

impl PCF0 {
    fn mapr(&mut self) -> &afio::Pcf0 {
        unsafe { (*Afio::ptr()).pcf0() }
    }

    pub fn modify_pcf0<F>(&mut self, mod_fn: F)
    where
        F: for<'w> FnOnce(&afio::pcf0::R, &'w mut afio::pcf0::W) -> &'w mut afio::pcf0::W,
    {
        let debug_bits = match self.debug_state {
            DebugState::FullyEnabled => 0b000,
            DebugState::JtagNoTrstEnabled => 0b001,
            DebugState::SwdEnabled => 0b010,
            DebugState::DebugDisabled => 0b100
        };
        self.mapr()
            .modify(unsafe { |r, w| mod_fn(r, w).swj_cfg().bits(debug_bits) });
    }

    /// Disables the JTAG to free up pa15, pb3 and pb4 for normal use
    #[allow(clippy::redundant_field_names, clippy::type_complexity)]
    pub fn disable_jtag(
        &mut self,
        pa15: PA15<Debugger>,
        pb3: PB3<Debugger>,
        pb4: PB4<Debugger>,
    ) -> (
        PA15<Input<Floating>>,
        PB3<Input<Floating>>,
        PB4<Input<Floating>>,
    ) {
        self.debug_state = DebugState::SwdEnabled;
        // Avoid duplicating swj_cfg write code
        self.modify_pcf0(|_, w| w);

        // NOTE(unsafe) The pins are now in the good state.
        unsafe { (pa15.activate(), pb3.activate(), pb4.activate()) }
    }

}

pub struct EXTISS0 {
    _0: (),
}

impl EXTISS0 {
    pub fn extiss0(&mut self) -> &afio::Extiss0 {
        unsafe { (*Afio::ptr()).extiss0() }
    }
}

pub struct EXTISS1 {
    _0: (),
}

impl EXTISS1 {
    pub fn extiss1(&mut self) -> &afio::Extiss1 {
        unsafe { &(*Afio::ptr()).extiss1() }
    }
}

pub struct EXTISS2 {
    _0: (),
}

impl EXTISS2 {
    pub fn extiss2(&mut self) -> &afio::Extiss2 {
        unsafe { (*Afio::ptr()).extiss2() }
    }
}

pub struct EXTISS3 {
    _0: (),
}

impl EXTISS3 {
    pub fn extiss3(&mut self) -> &afio::Extiss3 {
        unsafe { (*Afio::ptr()).extiss3() }
    }
}

pub struct PCF1 {
    _0: (),
}

impl PCF1 {
    pub fn pcf1(&mut self) -> &afio::Pcf1 {
        unsafe { (*Afio::ptr()).pcf1() }
    }
}
