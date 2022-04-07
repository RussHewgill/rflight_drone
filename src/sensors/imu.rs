// use cortex_m_semihosting::hprintln;

use embedded_hal as hal;
use hal::blocking::spi;

use stm32f4xx_hal::prelude::*;

pub struct IMU<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS> IMU<SPI, CS> {
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self { spi, cs }
    }
}

/// Why?
const SPI_READ: u8 = 0x01;
const SPI_WRITE: u8 = 0x00;

impl<SPI, CS, E, PinError> IMU<SPI, CS>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    CS: hal::digital::v2::OutputPin<Error = PinError>,
{
    pub fn write_reg(&mut self, reg: u8, val: u8) {
        // let mut bytes = [(reg << 1) | SPI_WRITE, value];

        // self.cs.set_low();
        // // self.spi.write(&mut bytes);
        // self.cs.set_high();
    }
}
