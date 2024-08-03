// #![allow(missing_docs)]
// #![deny(warnings)]
#![no_std]
#[cfg(not(any(
    feature = "gd32c103",
    feature = "gd32c113",
)))]
compile_error!("Target not found. One of `gd32c103` or `gd32c113` feature flags must be specified.");

// If any two or more targets are specified, print error message.
#[cfg(any(
    all(feature = "gd32c103", feature = "gd32c113"),
))]
compile_error!(
    "Multiple targets specified. Only a single target feature flag can be specified."
);


extern crate cortex_m;
pub extern crate embedded_hal as hal;

#[cfg(feature = "gd32c103")]
pub use gd32c1::gd32c103 as pac;
#[cfg(feature = "gd32c113")]
pub use gd32c1::gd32c113 as pac;
extern crate nb;

pub mod afio;
pub mod backup_domain;
pub mod bb;
pub mod can;
pub mod dma;
pub mod fmc;
pub mod gpio;
pub mod i2c;
pub mod prelude;
pub mod rcu;
// pub mod serial;
pub mod spi;
pub mod time;
pub mod timer;
pub mod crc;
pub mod usb;
pub mod watchdog;

mod sealed {
    pub trait Sealed {}
}

use sealed::Sealed;
