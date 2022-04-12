// use core::cell::RefCell;

use cortex_m_semihosting::hprintln;
use embedded_hal as hal;

use stm32f4xx_hal::nb;
use stm32f4xx_hal::prelude::*;

use crate::spi::{Spi3, SpiError};

pub struct Magnetometer<CS> {
    // spi: Spi3,
    cs: CS,
}

const SPI_READ: u8 = 0x80; // 0x01 << 7
const SPI_WRITE: u8 = 0x00;

impl<CS, PinError> Magnetometer<CS>
where
    CS: hal::digital::blocking::OutputPin<Error = PinError>,
{
    pub fn new(cs: CS) -> Self {
        Self { cs }
    }

    pub fn reset(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        self.write_reg(spi, MagRegister::CFG_REG_A, 0b0010_0000)?;
        Ok(())
    }

    pub fn init(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        /// Enable temperature compensation
        /// LP = 0, high power mode
        /// ODR = 00, Output Data Rate = 10Hz
        /// MD  = 00, Continuous mode
        // self.write_reg(MagRegister::CFG_REG_A, 0x80)?;
        /// Single mode
        self.write_reg(spi, MagRegister::CFG_REG_A, 0x81)?;
        // self.write_reg(MagRegister::CFG_REG_C, 0x01)?; // Data Ready signal pin isn't connected
        Ok(())
    }

    pub fn read_new_data_available(&mut self, spi: &mut Spi3) -> nb::Result<bool, SpiError> {
        let status = self.read_reg(spi, MagRegister::STATUS_REG)?;
        hprintln!("s: {:#010b}", status);
        Ok((status & 0b0000_1000) == 0b1000)
    }

    pub fn read_temp(&mut self, spi: &mut Spi3) -> nb::Result<i16, SpiError> {
        let temp_l = self.read_reg(spi, MagRegister::TEMP_OUT_L_REG)?;
        let temp_h = self.read_reg(spi, MagRegister::TEMP_OUT_H_REG)?;

        hprintln!("temp_l: {:#010b}", temp_l);
        hprintln!("temp_h: {:#010b}", temp_h);

        Ok(0)
    }

    pub fn read_data(&mut self, spi: &mut Spi3) -> nb::Result<[i16; 3], SpiError> {
        let outx_l = self.read_reg(spi, MagRegister::OUTX_L_REG)?;
        let outx_h = self.read_reg(spi, MagRegister::OUTX_H_REG)?;

        hprintln!("outx_l: {:#010b}", outx_l);
        hprintln!("outx_h: {:#010b}", outx_h);

        let outy_l = self.read_reg(spi, MagRegister::OUTY_L_REG)?;
        let outy_h = self.read_reg(spi, MagRegister::OUTY_H_REG)?;

        let outz_l = self.read_reg(spi, MagRegister::OUTZ_L_REG)?;
        let outz_h = self.read_reg(spi, MagRegister::OUTZ_H_REG)?;

        let mut out = [0u16; 3];

        out[0] |= outx_l as u16;
        out[0] |= (outx_h as u16) << 8;

        out[1] |= outy_l as u16;
        out[1] |= (outy_h as u16) << 8;

        out[2] |= outz_l as u16;
        out[2] |= (outz_h as u16) << 8;

        let mut out2 = [0i16; 3];

        out2[0] = out[0] as i16;
        out2[1] = out[1] as i16;
        out2[2] = out[2] as i16;

        Ok(out2)
    }
}

impl<CS, PinError> Magnetometer<CS>
where
    CS: hal::digital::blocking::OutputPin<Error = PinError>,
{
    pub fn read_reg(&mut self, spi: &mut Spi3, reg: MagRegister) -> nb::Result<u8, SpiError> {
        let mut out = 0u8;
        let addr = reg.to_addr() | SPI_READ;

        // let mut spi = self.spi.borrow_mut();

        self.cs.set_low().ok();

        spi.send(addr)?;
        spi.read(&mut out)?;

        self.cs.set_high().ok();
        Ok(out)
    }

    pub fn write_reg(
        &mut self,
        spi: &mut Spi3,
        reg: MagRegister,
        val: u8,
    ) -> nb::Result<(), SpiError> {
        let addr = reg.to_addr() | SPI_WRITE;

        // let mut spi = self.spi.borrow_mut();

        self.cs.set_low().ok();

        spi.send(addr)?;
        spi.send(val)?;

        self.cs.set_high().ok();
        Ok(())
    }
}

#[allow(non_camel_case_types)]
#[derive(Debug)]
#[repr(u8)]
pub enum MagRegister {
    OFFSET_X_REG_L = 0x45,
    OFFSET_X_REG_H = 0x46,
    OFFSET_Y_REG_L = 0x47,
    OFFSET_Y_REG_H = 0x48,
    OFFSET_Z_REG_L = 0x49,
    OFFSET_Z_REG_H = 0x4A,
    WHO_AM_I = 0x4F,
    CFG_REG_A = 0x60,
    CFG_REG_B = 0x61,
    CFG_REG_C = 0x62,
    INT_CTRL_REG = 0x63,
    INT_SOURCE_REG = 0x64,
    INT_THS_L_REG = 0x65,
    INT_THS_H_REG = 0x66,
    STATUS_REG = 0x67,
    OUTX_L_REG = 0x68,
    OUTX_H_REG = 0x69,
    OUTY_L_REG = 0x6A,
    OUTY_H_REG = 0x6B,
    OUTZ_L_REG = 0x6C,
    OUTZ_H_REG = 0x6D,
    TEMP_OUT_L_REG = 0x6E,
    TEMP_OUT_H_REG = 0x6F,
}

impl MagRegister {
    pub fn to_addr(self) -> u8 {
        self as u8
    }
}
