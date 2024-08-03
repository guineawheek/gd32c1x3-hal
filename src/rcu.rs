//! # Reset & Control Clock
#![allow(missing_docs)]
use crate::pac::{rcu, Pmu, Rcu as PacRcu};

use crate::fmc::WS;
use crate::time::MHz;
use fugit::{HertzU32 as Hertz, RateExtU32};

use crate::backup_domain::BackupDomain;

mod enable;

/// Extension trait that constrains the `RCU` peripheral
pub trait RcuExt {
    /// Constrains the `RCU` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcu;
}

impl RcuExt for PacRcu {
    fn constrain(self) -> Rcu {
        Rcu {
            cfgr: CFGR {
                hse: None,
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
                adcclk: None,
            },
            bkp: BKP { _0: () },
        }
    }
}

/// Constrained RCU peripheral
///
/// Aquired by calling the [constrain](../trait.RcuExt.html#tymethod.constrain) method
/// on the Rcu struct from the `PAC`
///
/// ```rust
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcu = dp.RCU.constrain();
/// ```
pub struct Rcu {
    pub cfgr: CFGR,
    pub bkp: BKP,
}

/// AMBA High-performance Bus (AHB) registers
pub struct AHB {
    _0: (),
}

impl AHB {
    fn en(rcu: &rcu::RegisterBlock) -> &rcu::Ahben {
        rcu.ahben()
    }
}

/// Advanced Peripheral Bus 1 (APB1) registers
pub struct APB1 {
    _0: (),
}

impl APB1 {
    fn en(rcu: &rcu::RegisterBlock) -> &rcu::Apb1en {
        rcu.apb1en()
    }

    fn rst(rcu: &rcu::RegisterBlock) -> &rcu::Apb1rst {
        rcu.apb1rst()
    }
}

impl APB1 {
    /// Set power interface clock (PWREN) bit in RCU_APB1ENR
    pub fn set_pwren() {
        let rcu = unsafe { &*PacRcu::ptr() };
        Pmu::enable(rcu);
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

impl APB2 {
    fn en(rcu: &rcu::RegisterBlock) -> &rcu::Apb2en {
        rcu.apb2en()
    }

    fn rst(rcu: &rcu::RegisterBlock) -> &rcu::Apb2rst {
        rcu.apb2rst()
    }
}

const HSI: u32 = 8_000_000; // Hz

/// Clock configuration register (CFGR)
///
/// Used to configure the frequencies of the clocks present in the processor.
///
/// After setting all frequencies, call the [freeze](#method.freeze) function to
/// apply the configuration.
///
/// **NOTE**: Currently, it is not guaranteed that the exact frequencies selected will be
/// used, only frequencies close to it.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct CFGR {
    hse: Option<u32>,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
    adcclk: Option<u32>,
}

impl CFGR {
    /// Uses HSE (external oscillator) instead of HSI (internal RC oscillator) as the clock source.
    /// Will result in a hang if an external oscillator is not connected or it fails to start.
    /// The frequency specified must be the frequency of the external oscillator
    #[inline(always)]
    pub fn use_hse(mut self, freq: Hertz) -> Self {
        self.hse = Some(freq.raw());
        self
    }

    /// Sets the desired frequency for the HCLK clock
    #[inline(always)]
    pub fn hclk(mut self, freq: Hertz) -> Self {
        self.hclk = Some(freq.raw());
        self
    }

    /// Sets the desired frequency for the PCKL1 clock
    #[inline(always)]
    pub fn pclk1(mut self, freq: Hertz) -> Self {
        self.pclk1 = Some(freq.raw());
        self
    }

    /// Sets the desired frequency for the PCLK2 clock
    #[inline(always)]
    pub fn pclk2(mut self, freq: Hertz) -> Self {
        self.pclk2 = Some(freq.raw());
        self
    }

    /// Sets the desired frequency for the SYSCLK clock
    #[inline(always)]
    pub fn sysclk(mut self, freq: Hertz) -> Self {
        self.sysclk = Some(freq.raw());
        self
    }

    /// Sets the desired frequency for the ADCCLK clock
    #[inline(always)]
    pub fn adcclk(mut self, freq: Hertz) -> Self {
        self.adcclk = Some(freq.raw());
        self
    }

    /// Applies the clock configuration and returns a `Clocks` struct that signifies that the
    /// clocks are frozen, and contains the frequencies used. After this function is called,
    /// the clocks can not change
    ///
    /// Usage:
    ///
    /// ```rust
    /// let dp = pac::Peripherals::take().unwrap();
    /// let mut flash = dp.FLASH.constrain();
    /// let mut rcu = dp.RCU.constrain();
    /// let clocks = rcu.cfgr.freeze(&mut flash.acr);
    /// ```

    #[inline(always)]
    pub fn freeze(self, ws: &mut WS) -> Clocks {
        let cfg = Config::from_cfgr(self);
        Self::_freeze_with_config(cfg, ws)
    }

    #[inline(always)]
    pub fn freeze_with_config(self, cfg: Config, ws: &mut WS) -> Clocks {
        Self::_freeze_with_config(cfg, ws)
    }

    #[allow(unused_variables)]
    fn _freeze_with_config(cfg: Config, ws: &mut WS) -> Clocks {
        let clocks = cfg.get_clocks();
        // adjust flash wait states
        unsafe {
            ws.ws().write(|w| {
                w.wscnt().bits(if clocks.sysclk <= MHz(30) {
                    0b000
                } else if clocks.sysclk <= MHz(60) {
                    0b001
                } else if clocks.sysclk <= MHz(90) {
                    0b010
                } else if clocks.sysclk <= MHz(120) {
                    0b011
                } else {
                    0b011
                })
            })
        }

        let rcu = unsafe { &*PacRcu::ptr() };

        if cfg.hse.is_some() {
            // enable HSE and wait for it to be ready

            rcu.ctl0().modify(|_, w| w.hxtalen().set_bit());

            while rcu.ctl0().read().hxtalstb().bit_is_clear() {}
        }
        let prediv = match cfg.prediv {
            Some(div) => div,
            None => 0,
        };

        if let Some(pllmul_bits) = cfg.pllmul {
            // enable PLL and wait for it to be ready
            let pllmul_bits_3_0 = pllmul_bits & 0xF;
            let pllmul_bits_5_4 = (pllmul_bits & 0x30) >> 4;
            #[allow(unused_unsafe)]
            rcu.cfg0().modify(|_, w| unsafe {
                w.pllmf()
                    .bits(pllmul_bits_3_0)
                    .pllmf_msb()
                    .bits(pllmul_bits_5_4)
                    .pllsel()
                    .bit(cfg.hse.is_some())
            });

            rcu.cfg1().modify(|_,w| unsafe { w.predv0().bits(prediv) });
            rcu.ctl0().modify(|_, w| w.pllen().set_bit());

            while rcu.ctl0().read().pllstb().bit_is_clear() {}
        }
        rcu.cfg1().modify(|_,w| {
            w.adcpsc_3().bit(((cfg.adcpre as u8 >> 0x3) & 0x1)== 0x1)
        });
        // set prescalers and clock source
        rcu.cfg0().modify(|_, w| unsafe {
            w.adcpsc().bits((cfg.adcpre as u8) & 0x7)
                .apb2psc()
                .bits(cfg.ppre2 as u8)
                .apb1psc()
                .bits(cfg.ppre1 as u8)
                .ahbpsc()
                .bits(cfg.hpre as u8)
                .usbfspsc()
                .bits(cfg.usbpre as u8 & 0x3)
                .usbfspsc_3()
                .bit(((cfg.usbpre as u8 >> 0x2) & 0x1) == 0x1)
                .scs()
                .bits(if cfg.pllmul.is_some() {
                    // PLL
                    0b10
                } else if cfg.hse.is_some() {
                    // HSE
                    0b1
                } else {
                    // HSI
                    0b0
                })
        });
        clocks
    }
}

pub struct BKP {
    _0: (),
}

impl BKP {
    /// Enables write access to the registers in the backup domain
    pub fn constrain(self, bkp: crate::pac::Bkp, pmu: &mut Pmu) -> BackupDomain {
        // Enable the backup interface by setting PWREN and BKPEN
        let rcu = unsafe { &(*crate::pac::Rcu::ptr()) };
        crate::pac::Bkp::enable(rcu);
        crate::pac::Pmu::enable(rcu);

        // Enable access to the backup registers
        pmu.ctl().modify(|_r, w| w.bkpwen().set_bit());

        BackupDomain { _regs: bkp }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
///
/// To acquire it, use the freeze function on the `rcu.cfgr` register. If desired, you can adjust
/// the frequencies using the methods on [cfgr](struct.CFGR.html) before calling freeze.
///
/// ```rust
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcu = dp.RCU.constrain();
/// let mut flash = dp.FLASH.constrain();
///
/// let clocks = rcu.cfgr.freeze(&mut flash.acr);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    ppre2: u8,
    sysclk: Hertz,
    adcclk: Hertz,
    usbclk_valid: bool,
}

impl Clocks {
    /// Returns the frequency of the AHB
    pub const fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the APB1
    pub const fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub const fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    /// Returns the frequency of the APB1 Timers
    pub const fn pclk1_tim(&self) -> Hertz {
        Hertz::from_raw(self.pclk1.raw() * if self.ppre1() == 1 { 1 } else { 2 })
    }

    /// Returns the frequency of the APB2 Timers
    pub const fn pclk2_tim(&self) -> Hertz {
        Hertz::from_raw(self.pclk2.raw() * if self.ppre2() == 1 { 1 } else { 2 })
    }

    pub(crate) const fn ppre1(&self) -> u8 {
        self.ppre1
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) const fn ppre2(&self) -> u8 {
        self.ppre2
    }

    /// Returns the system (core) frequency
    pub const fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    /// Returns the adc clock frequency
    pub const fn adcclk(&self) -> Hertz {
        self.adcclk
    }

    /// Returns whether the USBCLK clock frequency is valid for the USB peripheral
    pub const fn usbclk_valid(&self) -> bool {
        self.usbclk_valid
    }
}

/// Frequency on bus that peripheral is connected in
pub trait BusClock {
    /// Calculates frequency depending on `Clock` state
    fn clock(clocks: &Clocks) -> Hertz;
}

/// Frequency on bus that timer is connected in
pub trait BusTimerClock {
    /// Calculates base frequency of timer depending on `Clock` state
    fn timer_clock(clocks: &Clocks) -> Hertz;
}

impl<T> BusClock for T
where
    T: RcuBus,
    T::Bus: BusClock,
{
    fn clock(clocks: &Clocks) -> Hertz {
        T::Bus::clock(clocks)
    }
}

impl<T> BusTimerClock for T
where
    T: RcuBus,
    T::Bus: BusTimerClock,
{
    fn timer_clock(clocks: &Clocks) -> Hertz {
        T::Bus::timer_clock(clocks)
    }
}

impl BusClock for AHB {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.hclk
    }
}

impl BusClock for APB1 {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.pclk1
    }
}

impl BusClock for APB2 {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.pclk2
    }
}

impl BusTimerClock for APB1 {
    fn timer_clock(clocks: &Clocks) -> Hertz {
        clocks.pclk1_tim()
    }
}

impl BusTimerClock for APB2 {
    fn timer_clock(clocks: &Clocks) -> Hertz {
        clocks.pclk2_tim()
    }
}

/// Bus associated to peripheral
pub trait RcuBus: crate::Sealed {
    /// Bus type;
    type Bus;
}

/// Enable/disable peripheral
pub trait Enable: RcuBus {
    fn enable(rcu: &rcu::RegisterBlock);
    fn disable(rcu: &rcu::RegisterBlock);
}
/// Reset peripheral
pub trait Reset: RcuBus {
    fn reset(rcu: &rcu::RegisterBlock);
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    pub hse: Option<u32>,
    pub pllmul: Option<u8>,
    pub prediv: Option<u8>,
    pub hpre: HPre,
    pub ppre1: PPre,
    pub ppre2: PPre,
    pub usbpre: UsbPre,
    pub adcpre: AdcPre,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            hse: None,
            pllmul: None,
            prediv: None,
            hpre: HPre::Div1,
            ppre1: PPre::Div1,
            ppre2: PPre::Div1,
            usbpre: UsbPre::Div1p5,
            adcpre: AdcPre::Pclk2Div2,
        }
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HPre {
    /// SYSCLK not divided
    Div1 = 7,
    /// SYSCLK divided by 2
    Div2 = 8,
    /// SYSCLK divided by 4
    Div4 = 9,
    /// SYSCLK divided by 8
    Div8 = 10,
    /// SYSCLK divided by 16
    Div16 = 11,
    /// SYSCLK divided by 64
    Div64 = 12,
    /// SYSCLK divided by 128
    Div128 = 13,
    /// SYSCLK divided by 256
    Div256 = 14,
    /// SYSCLK divided by 512
    Div512 = 15,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PPre {
    /// HCLK not divided
    Div1 = 3,
    /// HCLK divided by 2
    Div2 = 4,
    /// HCLK divided by 4
    Div4 = 5,
    /// HCLK divided by 8
    Div8 = 6,
    /// HCLK divided by 16
    Div16 = 7,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum UsbPre {
    /// PLLCLK divided by 1.5
    Div1p5 = 0b000,
    /// PLLCLK divided by 1
    Div1 = 0b001,
    /// PLLCLK divided by 2.5
    Div2p5 = 0b010,
    /// PLLCLK divided by 2
    Div2 = 0b011,
    /// PLLCLK divided by 3
    Div3 = 0b100,
    /// PLLCLK divided by 3.5
    Div3p5 = 0b101,
    /// PLLCLK divided by 4
    Div4 = 0b110,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AdcPre {
    /// PCLK2 divided by 2
    Pclk2Div2 = 0b0000,
    /// PCLK2 divided by 4
    Pclk2Div4 = 0b0001,
    /// PCLK2 divided by 6
    Pclk2Div6 = 0b0010,
    /// PCLK2 divided by 8
    Pclk2Div8 = 0b0011,
    /// PCLK2 divided by 12
    Pclk2Div12 = 0b0101,
    /// PCLK2 divided by 16
    Pclk2Div16 = 0b0111,
    /// HCLK divided by 3
    HclkDiv3 = 0b1000,
    /// HCLK divided by 5
    HclkDiv5 = 0b1001,
    /// HCLK divided by 7
    HclkDiv7 = 0b1010,
    /// HCLK divided by 9
    HclkDiv9 = 0b1011,
}

impl Config {
    pub fn from_cfgr(cfgr: CFGR) -> Self {
        let hse = cfgr.hse;
        let pllsrculk = if let Some(hse) = hse { hse } else { HSI / 2 };

        let mut pllmul = if let Some(sysclk) = cfgr.sysclk {
            sysclk / pllsrculk
        } else {
            1
        };
        let mut prediv = 0;
        if pllmul == 15 {
            pllmul = 30;
            prediv = 1;
        }

        let (pllmul_bits, sysclk) = if pllmul == 1 {
            (None, if let Some(hse) = hse { hse } else { HSI })
        } else {
            let new_pllmul = match pllmul {
                2..=14 => pllmul - 2,
                15 => 14,
                16..=31 => pllmul - 1,
                0 => 1,
                _ => 31,
            };

            (Some(new_pllmul as u8), (pllsrculk * (pllmul)) / (prediv+1))
        };

        let hpre_bits = if let Some(hclk) = cfgr.hclk {
            match sysclk / hclk {
                0..=1 => HPre::Div1,
                2 => HPre::Div2,
                3..=5 => HPre::Div4,
                6..=11 => HPre::Div8,
                12..=39 => HPre::Div16,
                40..=95 => HPre::Div64,
                96..=191 => HPre::Div128,
                192..=383 => HPre::Div256,
                _ => HPre::Div512,
            }
        } else {
            HPre::Div1
        };

        let hclk = if hpre_bits as u8 >= 0b1100 {
            sysclk / (1 << (hpre_bits as u8 - 0b0110))
        } else {
            sysclk / (1 << (hpre_bits as u8 - 0b0111))
        };
        assert!(hclk == 120_000_000,"hclk is {}!",hclk);

        let pclk1 = if let Some(pclk1) = cfgr.pclk1 {
            pclk1
        } else if hclk <= 60_000_000 {
            hclk
        } else {
            60_000_000
        };
        let ppre1_bits = match hclk / pclk1 {
            0 | 1 => PPre::Div1,
            2 => PPre::Div2,
            3..=5 => PPre::Div4,
            6..=11 => PPre::Div8,
            _ => PPre::Div16,
        };

        let ppre2_bits = if let Some(pclk2) = cfgr.pclk2 {
            match hclk / pclk2 {
                0..=1 => PPre::Div1,
                2 => PPre::Div2,
                3..=5 => PPre::Div4,
                6..=11 => PPre::Div8,
                _ => PPre::Div16,
            }
        } else {
            PPre::Div1
        };

        let ppre2 = 1 << (ppre2_bits as u8 - 0b011);
        let pclk2 = hclk / (ppre2 as u32);

        // usbpre == false: divide clock by 1.5, otherwise no division
        let usbpre = match (hse, pllmul_bits, sysclk) {
            (Some(_), Some(_), 48_000_000) => UsbPre::Div1,
            (Some(_), Some(_), 72_000_000) => UsbPre::Div1p5,
            (Some(_), Some(_), 96_000_000) => UsbPre::Div2,
            (Some(_), Some(_), 120_000_000) => UsbPre::Div2p5,
            _ => UsbPre::Div1,
        };

        let apre_bits = if let Some(adcclk) = cfgr.adcclk {
            match pclk2 / adcclk {
                0..=2 => AdcPre::Pclk2Div2,
                3..=4 => AdcPre::Pclk2Div4,
                5..=6 => AdcPre::Pclk2Div6,
                7..=8 => AdcPre::Pclk2Div8,
                9..=12 => AdcPre::Pclk2Div12,
                13..=16 => AdcPre::Pclk2Div16,
                _ => AdcPre::Pclk2Div16,
            }
        } else {
            AdcPre::Pclk2Div16
        };

        Self {
            hse,
            pllmul: pllmul_bits,
            prediv: Some(prediv as u8),
            hpre: hpre_bits,
            ppre1: ppre1_bits,
            ppre2: ppre2_bits,
            usbpre,
            adcpre: apre_bits,
        }
    }

    // NOTE: to maintain the invariant that the existence of a Clocks
    // value implies frozen clocks, this function must not be pub.
    fn get_clocks(&self) -> Clocks {
        let sysclk = if let Some(pllmul_bits) = self.pllmul {
            let pllmul : u32 = match pllmul_bits {
                0..=14 => (pllmul_bits + 2) as u32,
                15..=30 => (pllmul_bits + 1) as u32,
                _ => pllmul_bits as u32
            };
            let pllsrculk = if let Some(hse) = self.hse {
                hse
            } else {
                HSI / 2
            };
            match self.prediv {
                Some(prediv) => (pllsrculk / (prediv as u32 + 1)) * (pllmul),
                None => (pllsrculk) * (pllmul)
            }
        } else if let Some(hse) = self.hse {
            hse
        } else {
            HSI
        };

        let hclk = if self.hpre as u8 >= 0b1100 {
            sysclk / (1 << (self.hpre as u8 - 0b0110))
        } else {
            sysclk / (1 << (self.hpre as u8 - 0b0111))
        };

        let ppre1 = 1 << (self.ppre1 as u8 - 0b011);
        let pclk1 = hclk / (ppre1 as u32);

        let ppre2 = 1 << (self.ppre2 as u8 - 0b011);
        let pclk2 = hclk / (ppre2 as u32);

        let apre = (self.adcpre as u8 + 1) << 1;
        let adcclk = pclk2 / (apre as u32);

        // the USB clock is only valid if an external crystal is used, the PLL is enabled, and the
        // PLL output frequency is a supported one.
        let usbclk_valid = matches!(
            (self.hse, self.pllmul, sysclk),
            (Some(_), Some(_), 120_000_000)
                | (Some(_), Some(_), 96_000_000)
                | (Some(_), Some(_), 72_000_000)
                | (Some(_), Some(_), 48_000_000)
        );

        assert!(
            sysclk <= 120_000_000, "sysclk of {} requested!",sysclk
        );
        assert!(
            hclk <= 120_000_000
        );
        assert!(
            pclk1 <= 60_000_000
        );
        assert!(
            pclk2 <= 120_000_000
        );
        assert!(
            adcclk <= 40_000_000
        );

        Clocks {
            hclk: hclk.Hz(),
            pclk1: pclk1.Hz(),
            pclk2: pclk2.Hz(),
            ppre1,
            ppre2,
            sysclk: sysclk.Hz(),
            adcclk: adcclk.Hz(),
            usbclk_valid,
        }
    }
}

#[test]
fn rcu_config_usb() {
    let cfgr = CFGR::default()
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz());

    let config = Config::from_cfgr(cfgr);
    let config_expected = Config {
        hse: Some(8_000_000),
        pllmul: Some(4),
        prediv: Some(0),
        hpre: HPre::Div1,
        ppre1: PPre::Div2,
        ppre2: PPre::Div1,
        usbpre: UsbPre::Div1,
        adcpre: AdcPre::Pclk2Div8,
    };
    assert_eq!(config, config_expected);

    let clocks = config.get_clocks();
    let clocks_expected = Clocks {
        hclk: 48.MHz(),
        pclk1: 24.MHz(),
        pclk2: 48.MHz(),
        ppre1: 2,
        ppre2: 1,
        sysclk: 48.MHz(),
        adcclk: 6.MHz(),
        usbclk_valid: true,
    };
    assert_eq!(clocks, clocks_expected);
}
