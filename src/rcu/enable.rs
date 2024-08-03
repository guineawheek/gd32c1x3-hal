use super::*;
use crate::pac;
use crate::bb;

macro_rules! bus {
    ($($PER:ident => ($apbX:ty, $bit:literal),)+) => {
        $(
            impl crate::Sealed for pac::$PER {}

            impl RcuBus for pac::$PER {
                type Bus = $apbX;
            }
            impl Enable for pac::$PER {
                #[inline(always)]
                fn enable(rcu: &rcu::RegisterBlock) {
                    unsafe {
                        bb::set(Self::Bus::en(rcu), $bit);
                    }
                }
                #[inline(always)]
                fn disable(rcu: &rcu::RegisterBlock) {
                    unsafe {
                        bb::clear(Self::Bus::en(rcu), $bit);
                    }
                }
            }
            impl Reset for pac::$PER {
                #[inline(always)]
                fn reset(rcu: &rcu::RegisterBlock) {
                    unsafe {
                        bb::set(Self::Bus::rst(rcu), $bit);
                        bb::clear(Self::Bus::rst(rcu), $bit);
                    }
                }
            }
        )+
    }
}

macro_rules! ahb_bus {
    ($($PER:ident => ($bit:literal),)+) => {
        $(
            impl crate::Sealed for pac::$PER {}

            impl RcuBus for pac::$PER {
                type Bus = AHB;
            }
            impl Enable for pac::$PER {
                #[inline(always)]
                fn enable(rcu: &rcu::RegisterBlock) {
                    unsafe {
                        bb::set(Self::Bus::en(rcu), $bit);
                    }
                }
                #[inline(always)]
                fn disable(rcu: &rcu::RegisterBlock) {
                    unsafe {
                        bb::clear(Self::Bus::en(rcu), $bit);
                    }
                }
            }
        )+
    }
}

bus! {
    Timer1 => (APB1, 0),
    Timer2 => (APB1, 1),
    Timer3 => (APB1, 2),
    Timer4 => (APB1, 3),
    Timer5 => (APB1, 4),
    Timer6 => (APB1, 5),
    Timer11 => (APB1, 6),
    Timer12 => (APB1, 7),
    Timer13 => (APB1, 8),
    Wwdgt  => (APB1,11),
    Spi1   => (APB1,14),
    Spi2   => (APB1,15),
    Usart1 => (APB1,17),
    Usart2 => (APB1,18),
    Uart3  => (APB1,19),
    Uart4  => (APB1,20),
    I2c0   => (APB1,21),
    I2c1   => (APB1,22),
    Can0   => (APB1,25),
    Can1   => (APB1,26),
    Bkp    => (APB1,27),
    Pmu    => (APB1,28),
    Dac    => (APB1,29),
}

bus! {
    Afio => (APB2, 0),
    Gpioa => (APB2, 2),
    Gpiob => (APB2, 3),
    Gpioc => (APB2, 4),
    Gpiod => (APB2, 5),
    Gpioe => (APB2, 6),
    Adc0 => (APB2, 9),
    Adc1 => (APB2, 10),
    Timer0 => (APB2, 11),
    Spi0 => (APB2, 12),
    Timer7 => (APB2, 13),
    Usart0 => (APB2, 14),
    Timer8 => (APB2, 19),
    Timer9 => (APB2, 20),
    Timer10 => (APB2, 21),
}

ahb_bus! {
    Dma0 => (0),
    Dma1 => (1),
    Fmc  => (3),    
    Crc => (6),
    Exmc => (8),
    UsbfsPwrclk => (12),
}
