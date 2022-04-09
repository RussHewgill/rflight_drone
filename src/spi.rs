use embedded_hal::spi::*;
use stm32f4xx_hal::{
    gpio::{Pin, PinExt, PA8, PB13, PB15},
    prelude::*,
    rcc::Clocks,
    spi::NoMiso,
    time::*,
};

pub struct Spi3 {
    sck: PB13,
    mosi: PB15,
    miso: NoMiso,
}

impl Spi3 {
    pub fn new(sck: PB13, mosi: PB15, freq: Hertz, clocks: &Clocks) -> Self {
        Self {
            sck,
            mosi,
            miso: NoMiso {},
        }
    }
}
