use crate::{pac, gpio::{Floating, Input, Pin}, rcu::{Clocks, Enable}};
use fugit::HertzU32 as Hertz;
use synopsys_usb_otg::UsbPeripheral;
pub use synopsys_usb_otg::UsbBus;

pub struct USB {
    pub usb_global: pac::UsbfsGlobal,
    pub usb_device: pac::UsbfsDevice,
    pub usb_pwrclk: pac::UsbfsPwrclk,
    pub pin_dm: Pin<'A', 11, Input<Floating>>,
    pub pin_dp: Pin<'A', 12, Input<Floating>>,
    pub hclk: Hertz,
}

impl USB {
    pub fn new(
        periphs: (pac::UsbfsGlobal, pac::UsbfsDevice, pac::UsbfsPwrclk),
        pins: (Pin<'A', 11, Input<Floating>>, Pin<'A', 12, Input<Floating>>),
        clocks: &Clocks
    ) -> Self {
        let rcu = unsafe { &(*pac::Rcu::ptr()) };
        pac::UsbfsPwrclk::enable(rcu);

        Self {
            usb_global: periphs.0,
            usb_device: periphs.1,
            usb_pwrclk: periphs.2,
            pin_dm: pins.0,
            pin_dp: pins.1,
            hclk: clocks.hclk(),
        }
    }
}

unsafe impl Sync for USB {}

unsafe impl UsbPeripheral for USB {
    const REGISTERS: *const () = pac::UsbfsGlobal::ptr() as *const ();

    // not an HS device
    const HIGH_SPEED: bool = false;

    // this number may or may not be correct, but it seems to work on the peripheral.
    const FIFO_DEPTH_WORDS: usize = 320;

    const ENDPOINT_COUNT: usize = 4;

    fn enable() {
        let rcu = {unsafe{&(*pac::Rcu::ptr())}};

        cortex_m::interrupt::free(|_| {
            // Enable USB peripheral
            rcu.ahben().modify(|_, w| w.usbfsen().set_bit());
            // Reset USB peripheral
            rcu.ahbrst().modify(|_, w| w.usbfsrst().set_bit());
            rcu.ahbrst().modify(|_, w| w.usbfsrst().clear_bit());
        })

    }

    fn ahb_frequency_hz(&self) -> u32 {
        self.hclk.raw()
    }
}