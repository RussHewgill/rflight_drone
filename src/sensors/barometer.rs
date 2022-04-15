use embedded_hal as hal;

// use hal::spi::{
//     self,
//     blocking::{Read, Transfer, Write},
// };

use stm32f4xx_hal::prelude::*;

#[derive(Debug)]
pub struct Barometer<CS> {
    pub cs: CS,
}

/// new
impl<CS> Barometer<CS> {
    pub fn new(cs: CS) -> Self {
        Self { cs }
    }
}

// impl<CS, PinError> Barometer<CS>
// where
//     CS: hal::digital::blocking::OutputPin<Error = PinError>,
// {
//     // pub fn write_reg(&mut self, reg: u8, val: u8) {
//     //     let mut bytes = [(reg << 1) | SPI_WRITE, val];
//     //     self.cs.set_low().ok();
//     //     self.spi.write(&mut bytes).ok();
//     //     self.cs.set_high().ok();
//     // }
// }

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
