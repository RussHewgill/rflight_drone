use embedded_hal as hal;
use hal::spi::{
    self,
    blocking::{Read, Transfer, Write},
};

use stm32f4xx_hal::prelude::*;

#[derive(Debug)]
pub struct Barometer<SPI, CS> {
    pub spi: SPI,
    pub cs: CS,
}

/// new
impl<SPI, CS> Barometer<SPI, CS> {
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self { spi, cs }
    }
}

/// datasheet: 6.4.1, 6.4.2
const SPI_READ: u8 = 0x01;
const SPI_WRITE: u8 = 0x00;

impl<SPI, CS, E, PinError> Barometer<SPI, CS>
where
    SPI: Write<u8, Error = E> + Read<u8, Error = E>,
    CS: hal::digital::blocking::OutputPin<Error = PinError>,
{
    // pub fn write_reg(&mut self, reg: u8, val: u8) {
    //     let mut bytes = [(reg << 1) | SPI_WRITE, val];
    //     self.cs.set_low().ok();
    //     self.spi.write(&mut bytes).ok();
    //     self.cs.set_high().ok();
    // }
}

// #[allow(non_camel_case_types)]
// #[derive(Debug)]
// #[repr(u8)]
// pub enum BarometerRegisters {
//     //
// }

// impl BarometerRegisters {
//     pub fn get_addr(self) -> u8 {
//         self as u8
//     }
// }
