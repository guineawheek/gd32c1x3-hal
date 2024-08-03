//! # Direct Memory Access
#![allow(dead_code)]

use core::{
    marker::PhantomData,
    sync::atomic::{compiler_fence, Ordering},
};
use embedded_dma::{ReadBuffer, WriteBuffer};

#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    Overrun,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Half {
    First,
    Second,
}

pub struct CircBuffer<BUFFER, PAYLOAD>
where
    BUFFER: 'static,
{
    buffer: &'static mut [BUFFER; 2],
    payload: PAYLOAD,
    readable_half: Half,
}

impl<BUFFER, PAYLOAD> CircBuffer<BUFFER, PAYLOAD>
where
    &'static mut [BUFFER; 2]: WriteBuffer,
    BUFFER: 'static,
{
    pub(crate) fn new(buf: &'static mut [BUFFER; 2], payload: PAYLOAD) -> Self {
        CircBuffer {
            buffer: buf,
            payload,
            readable_half: Half::Second,
        }
    }
}

pub trait DmaExt {
    type Channels;

    fn split(self) -> Self::Channels;
}

pub trait TransferPayload {
    fn start(&mut self);
    fn stop(&mut self);
}

pub struct Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    _mode: PhantomData<MODE>,
    buffer: BUFFER,
    payload: PAYLOAD,
}

impl<BUFFER, PAYLOAD> Transfer<R, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn r(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn w(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<MODE, BUFFER, PAYLOAD> Drop for Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    fn drop(&mut self) {
        self.payload.stop();
        compiler_fence(Ordering::SeqCst);
    }
}

/// Read transfer
pub struct R;

/// Write transfer
pub struct W;

macro_rules! dma {
    ($($DMAX:ident: ($dmaX:ident, {
        $($CX:ident: (
            $chXctl:ident,
            $chXcnt:ident,
            $chXpaddr:ident,
            $chXmaddr:ident,

            $ChXctl:ident,
            $ChXcnt:ident,
            $ChXpaddr:ident,
            $ChXmaddr:ident,

            $htifX:ident,
            $tcifX:ident,
            $chtifX:ident,
            $ctcifX:ident,
            $cgifX:ident
        ),)+
    }),)+) => {
        $(
            pub mod $dmaX {
                use core::{sync::atomic::{self, Ordering}, ptr, mem, convert::TryFrom};

                use crate::pac::{Rcu, $DMAX, dma0};

                use crate::dma::{CircBuffer, DmaExt, Error, Event, Half, Transfer, W, RxDma, TxDma, RxTxDma, TransferPayload};
                use crate::rcu::Enable;

                #[allow(clippy::manual_non_exhaustive)]
                pub struct Channels( $(pub $CX),+);

                $(
                    /// A singleton that represents a single DMAx channel (channel X in this case)
                    ///
                    /// This singleton has exclusive access to the registers of the DMAx channel X
                    pub struct $CX { _0: () }

                    impl $CX {
                        /// Associated peripheral `address`
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        pub fn set_peripheral_address(&mut self, address: u32, inc: bool) {
                            self.ch_paddr().write(|w| unsafe { w.bits(address) } );
                            self.ch_ctl().modify(|_, w| w.pnaga().bit(inc) );
                        }

                        /// `address` where from/to data will be read/write
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        pub fn set_memory_address(&mut self, address: u32, inc: bool) {
                            self.ch_maddr().write(|w| unsafe { w.bits(address) } );
                            self.ch_ctl().modify(|_, w| w.mnaga().bit(inc) );
                        }

                        /// Number of bytes to transfer
                        pub fn set_transfer_length(&mut self, len: usize) {
                            unsafe { self.ch_cnt().write(|w| w.bits(u32::try_from(len).unwrap())); }
                        }

                        /// Starts the DMA transfer
                        pub fn start(&mut self) {
                            self.ch_ctl().modify(|_, w| w.chen().set_bit() );
                        }

                        /// Stops the DMA transfer
                        pub fn stop(&mut self) {
                            self.intc().write(|w| w.$cgifX().set_bit());
                            self.ch_ctl().modify(|_, w| w.chen().clear_bit() );
                        }

                        /// Returns `true` if there's a transfer in progress
                        pub fn in_progress(&self) -> bool {
                            self.intf().$tcifX().bit_is_clear()
                        }
                    }

                    impl $CX {
                        pub fn listen(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => self.ch_ctl().modify(|_, w| w.htfie().set_bit()),
                                Event::TransferComplete => {
                                    self.ch_ctl().modify(|_, w| w.ftfie().set_bit())
                                }
                            }
                        }

                        pub fn unlisten(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => {
                                    self.ch_ctl().modify(|_, w| w.htfie().clear_bit())
                                },
                                Event::TransferComplete => {
                                    self.ch_ctl().modify(|_, w| w.ftfie().clear_bit())
                                }
                            }
                        }

                        pub fn ch_ctl(&mut self) -> &dma0::$ChXctl {
                            unsafe { (*$DMAX::ptr()).$chXctl() }
                        }

                        pub fn ch_cnt(&mut self) -> &dma0::$ChXcnt {
                            unsafe { (*$DMAX::ptr()).$chXcnt() }
                        }

                        pub fn ch_paddr(&mut self) -> &dma0::$ChXpaddr {
                            unsafe { (*$DMAX::ptr()).$chXpaddr() }
                        }

                        pub fn ch_maddr(&mut self) -> &dma0::$ChXmaddr {
                            unsafe { (*$DMAX::ptr()).$chXmaddr() }
                        }

                        pub fn intf(&self) -> dma0::intf::R {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$DMAX::ptr()).intf().read() }
                        }

                        pub fn intc(&self) -> &dma0::Intc {
                            unsafe { &(*$DMAX::ptr()).intc() }
                        }

                        pub fn get_cnt(&self) -> u32 {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { &(*$DMAX::ptr())}.$chXcnt().read().bits()
                        }
                    }

                    impl<B, PAYLOAD> CircBuffer<B, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        /// Peeks into the readable half of the buffer
                        pub fn peek<R, F>(&mut self, f: F) -> Result<R, Error>
                            where
                            F: FnOnce(&B, Half) -> R,
                        {
                            let half_being_read = self.readable_half()?;

                            let buf = match half_being_read {
                                Half::First => &self.buffer[0],
                                Half::Second => &self.buffer[1],
                            };

                            // XXX does this need a compiler barrier?
                            let ret = f(buf, half_being_read);


                            let isr = self.payload.channel.intf();
                            let first_half_is_done = isr.$htifX().bit_is_set();
                            let second_half_is_done = isr.$tcifX().bit_is_set();

                            if (half_being_read == Half::First && second_half_is_done) ||
                                (half_being_read == Half::Second && first_half_is_done) {
                                Err(Error::Overrun)
                            } else {
                                Ok(ret)
                            }
                        }

                        /// Returns the `Half` of the buffer that can be read
                        pub fn readable_half(&mut self) -> Result<Half, Error> {
                            let isr = self.payload.channel.intf();
                            let first_half_is_done = isr.$htifX().bit_is_set();
                            let second_half_is_done = isr.$tcifX().bit_is_set();

                            if first_half_is_done && second_half_is_done {
                                return Err(Error::Overrun);
                            }

                            let last_read_half = self.readable_half;

                            Ok(match last_read_half {
                                Half::First => {
                                    if second_half_is_done {
                                        self.payload.channel.intc().write(|w| w.$ctcifX().set_bit());

                                        self.readable_half = Half::Second;
                                        Half::Second
                                    } else {
                                        last_read_half
                                    }
                                }
                                Half::Second => {
                                    if first_half_is_done {
                                        self.payload.channel.intc().write(|w| w.$chtifX().set_bit());

                                        self.readable_half = Half::First;
                                        Half::First
                                    } else {
                                        last_read_half
                                    }
                                }
                            })
                        }

                        /// Stops the transfer and returns the underlying buffer and RxDma
                        pub fn stop(mut self) -> (&'static mut [B; 2], RxDma<PAYLOAD, $CX>) {
                            self.payload.stop();

                            (self.buffer, self.payload)
                        }
                    }

                    impl<BUFFER, PAYLOAD, MODE> Transfer<MODE, BUFFER, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        pub fn is_done(&self) -> bool {
                            !self.payload.channel.in_progress()
                        }

                        pub fn wait(mut self) -> (BUFFER, RxDma<PAYLOAD, $CX>) {
                            while !self.is_done() {}

                            atomic::compiler_fence(Ordering::Acquire);

                            self.payload.stop();

                            // we need a read here to make the Acquire fence effective
                            // we do *not* need this if `dma.stop` does a RMW operation
                            unsafe { ptr::read_volatile(&0); }

                            // we need a fence here for the same reason we need one in `Transfer.wait`
                            atomic::compiler_fence(Ordering::Acquire);

                            // `Transfer` needs to have a `Drop` implementation, because we accept
                            // managed buffers that can free their memory on drop. Because of that
                            // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
                            // and `mem::forget`.
                            //
                            // NOTE(unsafe) There is no panic branch between getting the resources
                            // and forgetting `self`.
                            unsafe {
                                let buffer = ptr::read(&self.buffer);
                                let payload = ptr::read(&self.payload);
                                mem::forget(self);
                                (buffer, payload)
                            }
                        }
                    }

                    impl<BUFFER, PAYLOAD, MODE> Transfer<MODE, BUFFER, TxDma<PAYLOAD, $CX>>
                    where
                        TxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        pub fn is_done(&self) -> bool {
                            !self.payload.channel.in_progress()
                        }

                        pub fn wait(mut self) -> (BUFFER, TxDma<PAYLOAD, $CX>) {
                            while !self.is_done() {}

                            atomic::compiler_fence(Ordering::Acquire);

                            self.payload.stop();

                            // we need a read here to make the Acquire fence effective
                            // we do *not* need this if `dma.stop` does a RMW operation
                            unsafe { ptr::read_volatile(&0); }

                            // we need a fence here for the same reason we need one in `Transfer.wait`
                            atomic::compiler_fence(Ordering::Acquire);

                            // `Transfer` needs to have a `Drop` implementation, because we accept
                            // managed buffers that can free their memory on drop. Because of that
                            // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
                            // and `mem::forget`.
                            //
                            // NOTE(unsafe) There is no panic branch between getting the resources
                            // and forgetting `self`.
                            unsafe {
                                let buffer = ptr::read(&self.buffer);
                                let payload = ptr::read(&self.payload);
                                mem::forget(self);
                                (buffer, payload)
                            }
                        }
                    }

                    impl<BUFFER, PAYLOAD, MODE, TXC> Transfer<MODE, BUFFER, RxTxDma<PAYLOAD, $CX, TXC>>
                    where
                        RxTxDma<PAYLOAD, $CX, TXC>: TransferPayload,
                    {
                        pub fn is_done(&self) -> bool {
                            !self.payload.rxchannel.in_progress()
                        }

                        pub fn wait(mut self) -> (BUFFER, RxTxDma<PAYLOAD, $CX, TXC>) {
                            while !self.is_done() {}

                            atomic::compiler_fence(Ordering::Acquire);

                            self.payload.stop();

                            // we need a read here to make the Acquire fence effective
                            // we do *not* need this if `dma.stop` does a RMW operation
                            unsafe { ptr::read_volatile(&0); }

                            // we need a fence here for the same reason we need one in `Transfer.wait`
                            atomic::compiler_fence(Ordering::Acquire);

                            // `Transfer` needs to have a `Drop` implementation, because we accept
                            // managed buffers that can free their memory on drop. Because of that
                            // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
                            // and `mem::forget`.
                            //
                            // NOTE(unsafe) There is no panic branch between getting the resources
                            // and forgetting `self`.
                            unsafe {
                                let buffer = ptr::read(&self.buffer);
                                let payload = ptr::read(&self.payload);
                                mem::forget(self);
                                (buffer, payload)
                            }
                        }
                    }


                    impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        pub fn peek<T>(&self) -> &[T]
                        where
                            BUFFER: AsRef<[T]>,
                        {
                            let pending = self.payload.channel.get_cnt() as usize;

                            let slice = self.buffer.as_ref();
                            let capacity = slice.len();

                            &slice[..(capacity - pending)]
                        }
                    }

                    impl<RXBUFFER, TXBUFFER, PAYLOAD, TXC> Transfer<W, (RXBUFFER, TXBUFFER), RxTxDma<PAYLOAD, $CX, TXC>>
                    where
                        RxTxDma<PAYLOAD, $CX, TXC>: TransferPayload,
                    {
                        pub fn peek<T>(&self) -> &[T]
                        where
                            RXBUFFER: AsRef<[T]>,
                        {
                            let pending = self.payload.rxchannel.get_cnt() as usize;

                            let slice = self.buffer.0.as_ref();
                            let capacity = slice.len();

                            &slice[..(capacity - pending)]
                        }
                    }
                )+

                impl DmaExt for $DMAX {
                    type Channels = Channels;

                    fn split(self) -> Channels {
                        let rcu = unsafe { &(*Rcu::ptr()) };
                        $DMAX::enable(rcu);

                        // reset the DMA control registers (stops all on-going transfers)
                        $(
                            self.$chXcnt().reset();
                        )+

                        Channels( $($CX { _0: () }),+)
                    }
                }
            }
        )+
    }
}

dma! {
    Dma0: (dma0, {
        C0: (
            ch0ctl, ch0cnt, ch0paddr, ch0maddr,
            Ch0ctl, Ch0cnt, Ch0paddr, Ch0maddr,
            htfif0, ftfif0,
            htfifc0, ftfifc0, gifc0
        ),
        C1: (
            ch1ctl, ch1cnt, ch1paddr, ch1maddr,
            Ch1ctl, Ch1cnt, Ch1paddr, Ch1maddr,
            htfif1, ftfif1,
            htfifc1, ftfifc1, gifc1
        ),
        C2: (
            ch2ctl, ch2cnt, ch2paddr, ch2maddr,
            Ch2ctl, Ch2cnt, Ch2paddr, Ch2maddr,
            htfif2, ftfif2,
            htfifc2, ftfifc2, gifc2
        ),
        C3: (
            ch3ctl, ch3cnt, ch3paddr, ch3maddr,
            Ch3ctl, Ch3cnt, Ch3paddr, Ch3maddr,
            htfif3, ftfif3,
            htfifc3, ftfifc3, gifc3
        ),
        C4: (
            ch4ctl, ch4cnt, ch4paddr, ch4maddr,
            Ch4ctl, Ch4cnt, Ch4paddr, Ch4maddr,
            htfif4, ftfif4,
            htfifc4, ftfifc4, gifc4
        ),
        C5: (
            ch5ctl, ch5cnt, ch5paddr, ch5maddr,
            Ch5ctl, Ch5cnt, Ch5paddr, Ch5maddr,
            htfif5, ftfif5,
            htfifc5, ftfifc5, gifc5
        ),
        C6: (
            ch6ctl, ch6cnt, ch6paddr, ch6maddr,
            Ch6ctl, Ch6cnt, Ch6paddr, Ch6maddr,
            htfif6, ftfif6,
            htfifc6, ftfifc6, gifc6
        ),
    }),
    Dma1: (dma1, {
        C0: (
            ch0ctl, ch0cnt, ch0paddr, ch0maddr,
            Ch0ctl, Ch0cnt, Ch0paddr, Ch0maddr,
            htfif0, ftfif0,
            htfifc0, ftfifc0, gifc0
        ),
        C1: (
            ch1ctl, ch1cnt, ch1paddr, ch1maddr,
            Ch1ctl, Ch1cnt, Ch1paddr, Ch1maddr,
            htfif1, ftfif1,
            htfifc1, ftfifc1, gifc1
        ),
        C2: (
            ch2ctl, ch2cnt, ch2paddr, ch2maddr,
            Ch2ctl, Ch2cnt, Ch2paddr, Ch2maddr,
            htfif2, ftfif2,
            htfifc2, ftfifc2, gifc2
        ),
        C3: (
            ch3ctl, ch3cnt, ch3paddr, ch3maddr,
            Ch3ctl, Ch3cnt, Ch3paddr, Ch3maddr,
            htfif3, ftfif3,
            htfifc3, ftfifc3, gifc3
        ),
        C4: (
            ch4ctl, ch4cnt, ch4paddr, ch4maddr,
            Ch4ctl, Ch4cnt, Ch4paddr, Ch4maddr,
            htfif4, ftfif4,
            htfifc4, ftfifc4, gifc4
        ),
    }),
}

/// DMA Receiver
pub struct RxDma<PAYLOAD, RXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: RXCH,
}

/// DMA Transmitter
pub struct TxDma<PAYLOAD, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: TXCH,
}

/// DMA Receiver/Transmitter
pub struct RxTxDma<PAYLOAD, RXCH, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub rxchannel: RXCH,
    pub txchannel: TXCH,
}

pub trait Receive {
    type RxChannel;
    type TransmittedWord;
}

pub trait Transmit {
    type TxChannel;
    type ReceivedWord;
}

/// Trait for circular DMA readings from peripheral to memory.
pub trait CircReadDma<B, RS>: Receive
where
    &'static mut [B; 2]: WriteBuffer<Word = RS>,
    B: 'static,
    Self: core::marker::Sized,
{
    fn circ_read(self, buffer: &'static mut [B; 2]) -> CircBuffer<B, Self>;
}

/// Trait for DMA readings from peripheral to memory.
pub trait ReadDma<B, RS>: Receive
where
    B: WriteBuffer<Word = RS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn read(self, buffer: B) -> Transfer<W, B, Self>;
}

/// Trait for DMA writing from memory to peripheral.
pub trait WriteDma<B, TS>: Transmit
where
    B: ReadBuffer<Word = TS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn write(self, buffer: B) -> Transfer<R, B, Self>;
}

/// Trait for DMA simultaneously reading and writing within one synchronous operation. Panics if both buffers are not of equal length.
pub trait ReadWriteDma<RXB, TXB, TS>: Transmit
where
    RXB: WriteBuffer<Word = TS>,
    TXB: ReadBuffer<Word = TS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn read_write(self, rx_buffer: RXB, tx_buffer: TXB) -> Transfer<W, (RXB, TXB), Self>;
}
