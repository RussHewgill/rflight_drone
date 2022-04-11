use core::ptr;
use embedded_hal::spi::*;
use stm32f4::stm32f401::{Peripherals, GPIOB, RCC, SPI2};
use stm32f4xx_hal::{
    gpio::{Alternate, Pin, PinExt, PA8, PB13, PB15},
    nb,
    prelude::*,
    rcc::Clocks,
    spi::{Error as SpiError, NoMiso},
    time::*,
};

pub struct BluetoothSpi<SPI, CS, RESET> {
    spi: SPI,
    cs: CS,
    reset: RESET,
}

impl<SPI, CS, RESET> BluetoothSpi<SPI, CS, RESET> {
    pub fn new(spi: SPI, cs: CS, reset: RESET) -> Self {
        Self {
            //
            spi,
            cs,
            reset,
        }
    }
}
