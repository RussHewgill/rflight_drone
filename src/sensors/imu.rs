// use cortex_m_semihosting::hprintln;

use embedded_hal as hal;
use hal::blocking::spi;

use stm32f4xx_hal::prelude::*;

/// https://www.st.com/resource/en/datasheet/lsm6dsl.pdf

pub struct IMU<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS> IMU<SPI, CS> {
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self { spi, cs }
    }
}

/// datasheet: 6.4.1, 6.4.2
const SPI_READ: u8 = 0x01;
const SPI_WRITE: u8 = 0x00;

impl<SPI, CS, E, PinError> IMU<SPI, CS>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    CS: hal::digital::v2::OutputPin<Error = PinError>,
{
    // pub fn set_3wire(&mut self) {
    // }

    pub fn write_reg(&mut self, reg: u8, val: u8) {
        let mut bytes = [(reg << 1) | SPI_WRITE, val];
        self.cs.set_low().ok();
        self.spi.write(&mut bytes).ok();
        self.cs.set_high().ok();
    }

    pub fn read_reg(&mut self, reg: u8, buf: &mut [u8]) {
        let mut bytes = [(reg << 1) | SPI_WRITE, 0];
        self.cs.set_low().ok();
        self.spi.transfer(&mut bytes).ok();
        self.cs.set_high().ok();
        buf[0] = bytes[1];
    }
}
