use embedded_hal as hal;

use stm32f4xx_hal::nb;
use stm32f4xx_hal::{
    hal::digital::v2::{InputPin, OutputPin},
    prelude::*,
};

use crate::spi::{Spi3, SpiError};

const SPI_READ: u8 = 0x80; // 0x01 << 7
const SPI_WRITE: u8 = 0x00;

#[derive(Debug)]
pub struct Barometer<CS> {
    pub cs: CS,
}

/// new, config
impl<CS, PinError> Barometer<CS>
where
    CS: OutputPin<Error = PinError>,
{
    pub fn reset(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        self.write_reg(spi, BaroRegister::CTRL_REG1, 0b0000_0001)?; // 3-wire mode
        self.write_reg(spi, BaroRegister::CTRL_REG2, 0b0000_1000)?;
        self.write_reg(spi, BaroRegister::CTRL_REG3, 0b0000_0000)?;
        Ok(())
    }

    pub fn new(cs: CS) -> Self {
        Self { cs }
    }

    pub fn init(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        let mut val1 = 0b0000_0000;

        val1 |= 0b1; // 3-wire mode
        val1 |= 0b10; // Block Data Update

        self.write_reg(spi, BaroRegister::CTRL_REG1, val1)?;
        let mut val2 = 0b0000_0000;

        val2 |= 0b0001_0000; // address increment
        val2 |= 0b1000; // disable i2c
        self.write_reg(spi, BaroRegister::CTRL_REG2, val2)?;

        Ok(())
    }

    /// needs short wait before data is ready, even if status register is high
    /// about 50 nops
    pub fn one_shot(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        let current = self.read_reg(spi, BaroRegister::CTRL_REG2)?;
        self.write_reg(spi, BaroRegister::CTRL_REG2, current | 0b1)?;
        Ok(())
    }

    pub fn set_data_rate(
        &mut self,
        spi: &mut Spi3,
        rate: BaroDataRate,
    ) -> nb::Result<(), SpiError> {
        let current = self.read_reg(spi, BaroRegister::CTRL_REG1)?;
        let current = current & !0b0111_0000;
        self.write_reg(spi, BaroRegister::CTRL_REG1, current | rate.to_val())?;
        Ok(())
    }
}

/// read data
impl<CS, PinError> Barometer<CS>
where
    CS: OutputPin<Error = PinError>,
{
    /// pressure, temperature
    pub fn read_new_data_available(
        &mut self,
        spi: &mut Spi3,
    ) -> nb::Result<(bool, bool), SpiError> {
        let status = self.read_reg(spi, BaroRegister::STATUS)?;

        Ok((
            (status & 0b0000_0001) == 0b1,
            (status & 0b0000_0010) == 0b10,
        ))
    }

    pub fn read_data(&mut self, spi: &mut Spi3) -> nb::Result<f32, SpiError> {
        let mut data = [0u8; 3];
        self.read_reg_mult(spi, BaroRegister::PRESS_OUT_XL, &mut data)?;
        Ok(Self::convert_raw_data(data[0], data[1], data[2]))
    }

    pub fn read_temperature_data(&mut self, spi: &mut Spi3) -> nb::Result<f32, SpiError> {
        let mut data = [0u8; 2];
        self.read_reg_mult(spi, BaroRegister::TEMP_OUT_L, &mut data)?;

        let l = data[0];
        let h = data[1];
        let t = l as i16 | ((h as i16) << 8);

        Ok(t as f32 / 100.0)
    }

    fn convert_raw_data(xl: u8, l: u8, h: u8) -> f32 {
        const SCALE: f32 = 4096.0;
        let xs: [u8; 4] = [0, h, l, xl];
        u32::from_be_bytes(xs) as f32 / SCALE
    }
}

impl<CS, PinError> Barometer<CS>
where
    CS: OutputPin<Error = PinError>,
{
    pub fn read_reg_mult(
        &mut self,
        spi: &mut Spi3,
        start_reg: BaroRegister,
        bytes: &mut [u8],
    ) -> nb::Result<(), SpiError> {
        let addr = start_reg.to_addr() | SPI_READ;

        self.cs.set_low().ok();

        spi.send_blocking(addr)?;
        // spi.read(&mut out)?;
        spi.read_mult(bytes)?;

        self.cs.set_high().ok();
        Ok(())
    }

    pub fn read_reg(
        &mut self,
        spi: &mut Spi3,
        reg: BaroRegister,
    ) -> nb::Result<u8, SpiError> {
        let mut out = 0u8;
        let addr = reg.to_addr() | SPI_READ;

        // let mut spi = self.spi.borrow_mut();

        self.cs.set_low().ok();

        spi.send_blocking(addr)?;
        spi.read(&mut out)?;

        self.cs.set_high().ok();
        Ok(out)
    }

    pub fn write_reg(
        &mut self,
        spi: &mut Spi3,
        reg: BaroRegister,
        val: u8,
    ) -> nb::Result<(), SpiError> {
        let addr = reg.to_addr() | SPI_WRITE;

        // let mut spi = self.spi.borrow_mut();

        self.cs.set_low().ok();

        spi.send_blocking(addr)?;
        spi.send_blocking(val)?;

        self.cs.set_high().ok();
        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum BaroDataRate {
    PowerDown = 0b000,
    R1        = 0b001,
    R10       = 0b010,
    R25       = 0b011,
    R50       = 0b100,
    R75       = 0b101,
}

impl BaroDataRate {
    fn to_val(self) -> u8 {
        (self as u8) << 4
    }
}

#[allow(non_camel_case_types)]
#[derive(Debug)]
#[repr(u8)]
pub enum BaroRegister {
    WHO_AM_I     = 0x0F,
    CTRL_REG1    = 0x10,
    CTRL_REG2    = 0x11,
    CTRL_REG3    = 0x12,
    FIFO_CTRL    = 0x14,
    // REF_P_XL     = 0x15,
    // REF_P_L      = 0x16,
    // REF_P_H      = 0x17,
    // RPDS_L       = 0x18,
    // RPDS_H       = 0x19,
    RES_CONF     = 0x1A,
    STATUS       = 0x27,
    PRESS_OUT_XL = 0x28,
    PRESS_OUT_L  = 0x29,
    PRESS_OUT_H  = 0x2A,
    TEMP_OUT_L   = 0x2B,
    TEMP_OUT_H   = 0x2C,
    // LPFP_RES     = 0x33,
}

impl BaroRegister {
    pub fn to_addr(self) -> u8 {
        self as u8
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
