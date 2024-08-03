//! # Controller Area Network (CAN) Interface
//!
//! ## Alternate function remapping
//!
//! TX: Alternate Push-Pull Output
//! RX: Input
//!
//! ### CAN0
//!
//! | Function | NoRemap | Remap |
//! |----------|---------|-------|
//! | TX       | PA12    | PB9   |
//! | RX       | PA11    | PB8   |
//!
//! ### CAN1
//!
//! | Function | NoRemap | Remap |
//! |----------|---------|-------|
//! | TX       | PB6     | PB13  |
//! | RX       | PB5     | PB12  |

use crate::afio::PCF0;
use crate::gpio::{self, Alternate, Input};
use crate::pac::{self, Rcu};

pub trait Pins: crate::Sealed {
    type Instance;
    fn remap(pcf0: &mut PCF0);
}

impl<INMODE, OUTMODE> crate::Sealed
    for (gpio::PA12<Alternate<OUTMODE>>, gpio::PA11<Input<INMODE>>)
{
}
impl<INMODE, OUTMODE> Pins for (gpio::PA12<Alternate<OUTMODE>>, gpio::PA11<Input<INMODE>>) {
    type Instance = pac::Can0;

    fn remap(pcf0: &mut PCF0) {
        pcf0.modify_pcf0(|_, w| unsafe { w.can0_remap().bits(0) });
    }
}

impl<INMODE, OUTMODE> crate::Sealed for (gpio::PB9<Alternate<OUTMODE>>, gpio::PB8<Input<INMODE>>) {}
impl<INMODE, OUTMODE> Pins for (gpio::PB9<Alternate<OUTMODE>>, gpio::PB8<Input<INMODE>>) {
    type Instance = pac::Can0;

    fn remap(pcf0: &mut PCF0) {
        pcf0.modify_pcf0(|_, w| unsafe { w.can0_remap().bits(0b10) });
    }
}

impl<INMODE, OUTMODE> crate::Sealed
    for (gpio::PB13<Alternate<OUTMODE>>, gpio::PB12<Input<INMODE>>)
{
}
impl<INMODE, OUTMODE> Pins for (gpio::PB13<Alternate<OUTMODE>>, gpio::PB12<Input<INMODE>>) {
    type Instance = pac::Can1;

    fn remap(pcf0: &mut PCF0) {
        pcf0.modify_pcf0(|_, w| w.can1_remap().clear_bit());
    }
}

impl<INMODE, OUTMODE> crate::Sealed for (gpio::PB6<Alternate<OUTMODE>>, gpio::PB5<Input<INMODE>>) {}
impl<INMODE, OUTMODE> Pins for (gpio::PB6<Alternate<OUTMODE>>, gpio::PB5<Input<INMODE>>) {
    type Instance = pac::Can1;

    fn remap(pcf0: &mut PCF0) {
        pcf0.modify_pcf0(|_, w| w.can1_remap().set_bit());
    }
}

/// Interface to the CAN peripheral.
pub struct Can<Instance> {
    _peripheral: Instance,
}

impl<Instance> Can<Instance>
where
    Instance: crate::rcu::Enable,
{
    /// Creates a CAN interaface.
    pub fn new(can: Instance) -> Can<Instance> {
        let rcu = unsafe { &(*Rcu::ptr()) };
        Instance::enable(rcu);

        Can { _peripheral: can }
    }

    /// Routes CAN TX signals and RX signals to pins.
    pub fn assign_pins<P>(&self, _pins: P, pcf0: &mut PCF0)
    where
        P: Pins<Instance = Instance>,
    {
        P::remap(pcf0);
    }
}

unsafe impl bxcan::Instance for Can<pac::Can0> {
    const REGISTERS: *mut bxcan::RegisterBlock = pac::Can0::ptr() as *mut _;
}

unsafe impl bxcan::Instance for Can<pac::Can1> {
    const REGISTERS: *mut bxcan::RegisterBlock = pac::Can1::ptr() as *mut _;
}

unsafe impl bxcan::FilterOwner for Can<pac::Can0> {
    const NUM_FILTER_BANKS: u8 = 28;
}

unsafe impl bxcan::MasterInstance for Can<pac::Can0> {}
