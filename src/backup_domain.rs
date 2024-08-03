/*!
  Registers that are not reset as long as Vbat or Vdd has power.

  The registers retain their values during wakes from standby mode or system resets. They also
  retain their value when Vdd is switched off as long as V_BAT is powered.

  The backup domain also contains tamper protection and writes to it must be enabled in order
  to use the real time clock (RTC).

  Write access to the backup domain is enabled in RCC using the `rcc::Rcc::BKP::constrain()`
  function.

  Only the RTC functionality is currently implemented.
*/

use crate::pac::Bkp;

/**
  The existence of this struct indicates that writing to the the backup
  domain has been enabled. It is acquired by calling `constrain` on `rcc::Rcc::BKP`
*/
pub struct BackupDomain {
    pub(crate) _regs: Bkp,
}


impl BackupDomain {
    /// Read a 16-bit value from one of the DR1 to DR42 registers part of the
    /// Backup Data Register. The register argument is a zero based index to the
    /// DRx registers: 0 is DR1, up to 41 for DR42. Providing a number above 41
    /// will panic.
    pub fn read_data_register(&self, register: usize) -> u16 {

        // sigh.
        match register {
            0 => self._regs.data0().read().data().bits(),
            1 => self._regs.data1().read().data().bits(),
            2 => self._regs.data2().read().data().bits(),
            3 => self._regs.data3().read().data().bits(),
            4 => self._regs.data4().read().data().bits(),
            5 => self._regs.data5().read().data().bits(),
            6 => self._regs.data6().read().data().bits(),
            7 => self._regs.data7().read().data().bits(),
            8 => self._regs.data8().read().data().bits(),
            9 => self._regs.data9().read().data().bits(),
            10 => self._regs.data10().read().data().bits(),
            11 => self._regs.data11().read().data().bits(),
            12 => self._regs.data12().read().data().bits(),
            13 => self._regs.data13().read().data().bits(),
            14 => self._regs.data14().read().data().bits(),
            15 => self._regs.data15().read().data().bits(),
            16 => self._regs.data16().read().data().bits(),
            17 => self._regs.data17().read().data().bits(),
            18 => self._regs.data18().read().data().bits(),
            19 => self._regs.data19().read().data().bits(),
            20 => self._regs.data20().read().data().bits(),
            21 => self._regs.data21().read().data().bits(),
            22 => self._regs.data22().read().data().bits(),
            23 => self._regs.data23().read().data().bits(),
            24 => self._regs.data24().read().data().bits(),
            25 => self._regs.data25().read().data().bits(),
            26 => self._regs.data26().read().data().bits(),
            27 => self._regs.data27().read().data().bits(),
            28 => self._regs.data28().read().data().bits(),
            29 => self._regs.data29().read().data().bits(),
            31 => self._regs.data31().read().data().bits(),
            32 => self._regs.data32().read().data().bits(),
            33 => self._regs.data33().read().data().bits(),
            34 => self._regs.data34().read().data().bits(),
            35 => self._regs.data35().read().data().bits(),
            36 => self._regs.data36().read().data().bits(),
            37 => self._regs.data37().read().data().bits(),
            38 => self._regs.data38().read().data().bits(),
            39 => self._regs.data39().read().data().bits(),
            40 => self._regs.data40().read().data().bits(),
            41 => self._regs.data41().read().data().bits(),
            _ => panic!("invalid register!")
        }

    }

    /// Write a 16-bit value to one of the DR1 to DR42 registers part of the
    /// Backup Data Register. The register argument is a zero based index to the
    /// DRx registers: 0 is DR1, up to 41 for DR42. Providing a number above 41
    /// will panic.
    pub fn write_data_register_low(&self, register: usize, data: u16) {
        unsafe {
            match register {
                0 => self._regs.data0().write(|w| w.data().bits(data)),
                1 => self._regs.data1().write(|w| w.data().bits(data)),
                2 => self._regs.data2().write(|w| w.data().bits(data)),
                3 => self._regs.data3().write(|w| w.data().bits(data)),
                4 => self._regs.data4().write(|w| w.data().bits(data)),
                5 => self._regs.data5().write(|w| w.data().bits(data)),
                6 => self._regs.data6().write(|w| w.data().bits(data)),
                7 => self._regs.data7().write(|w| w.data().bits(data)),
                8 => self._regs.data8().write(|w| w.data().bits(data)),
                9 => self._regs.data9().write(|w| w.data().bits(data)),
                10 => self._regs.data10().write(|w| w.data().bits(data)),
                11 => self._regs.data11().write(|w| w.data().bits(data)),
                12 => self._regs.data12().write(|w| w.data().bits(data)),
                13 => self._regs.data13().write(|w| w.data().bits(data)),
                14 => self._regs.data14().write(|w| w.data().bits(data)),
                15 => self._regs.data15().write(|w| w.data().bits(data)),
                16 => self._regs.data16().write(|w| w.data().bits(data)),
                17 => self._regs.data17().write(|w| w.data().bits(data)),
                18 => self._regs.data18().write(|w| w.data().bits(data)),
                19 => self._regs.data19().write(|w| w.data().bits(data)),
                20 => self._regs.data20().write(|w| w.data().bits(data)),
                21 => self._regs.data21().write(|w| w.data().bits(data)),
                22 => self._regs.data22().write(|w| w.data().bits(data)),
                23 => self._regs.data23().write(|w| w.data().bits(data)),
                24 => self._regs.data24().write(|w| w.data().bits(data)),
                25 => self._regs.data25().write(|w| w.data().bits(data)),
                26 => self._regs.data26().write(|w| w.data().bits(data)),
                27 => self._regs.data27().write(|w| w.data().bits(data)),
                28 => self._regs.data28().write(|w| w.data().bits(data)),
                29 => self._regs.data29().write(|w| w.data().bits(data)),
                31 => self._regs.data31().write(|w| w.data().bits(data)),
                32 => self._regs.data32().write(|w| w.data().bits(data)),
                33 => self._regs.data33().write(|w| w.data().bits(data)),
                34 => self._regs.data34().write(|w| w.data().bits(data)),
                35 => self._regs.data35().write(|w| w.data().bits(data)),
                36 => self._regs.data36().write(|w| w.data().bits(data)),
                37 => self._regs.data37().write(|w| w.data().bits(data)),
                38 => self._regs.data38().write(|w| w.data().bits(data)),
                39 => self._regs.data39().write(|w| w.data().bits(data)),
                40 => self._regs.data40().write(|w| w.data().bits(data)),
                41 => self._regs.data41().write(|w| w.data().bits(data)),
                _ => panic!("invalid register!")
            }
        }
    }

    /// Conjures up a new BackupDomain.
    /// 
    /// It is up to the caller to ensure that this will not race with 
    /// any existing instances that touch the backup domain registers.
    /// 
    /// This also does not initialize the backup registers.
    pub unsafe fn conjure() -> Self {
        Self {
            _regs: unsafe { crate::pac::Bkp::steal() }
        }
    }
}
