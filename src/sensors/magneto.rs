// use core::cell::RefCell;

// use cortex_m_semihosting::hprintln;
use embedded_hal as hal;

use stm32f4xx_hal::nb;
use stm32f4xx_hal::{
    hal::digital::v2::{InputPin, OutputPin},
    prelude::*,
};

use crate::spi::{Spi3, SpiError};

#[derive(Debug)]
pub struct Magnetometer<CS> {
    // spi: Spi3,
    cs: CS,
}

const SPI_READ: u8 = 0x80; // 0x01 << 7
const SPI_WRITE: u8 = 0x00;

/// init, config
impl<CS, PinError> Magnetometer<CS>
where
    CS: OutputPin<Error = PinError>,
{
    pub fn new(cs: CS) -> Self {
        Self { cs }
    }

    pub fn reset(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        self.write_reg(spi, MagRegister::CFG_REG_A, 0b0010_0000)?;
        Ok(())
    }

    pub fn init_continuous(
        &mut self,
        spi: &mut Spi3,
        odr: MagDataRate,
    ) -> nb::Result<(), SpiError> {
        /// Enable temperature compensation
        /// LP = 0, high power mode
        /// ODR = 00, Output Data Rate = 10Hz
        /// MD  = 00, Continuous mode
        // self.write_reg(spi, MagRegister::CFG_REG_A, 0x80)?;
        let val = 0
            | 0b1000_0000 // temperature compensation
            | odr.to_val();
        self.write_reg(spi, MagRegister::CFG_REG_A, val)?;

        // self.write_reg(MagRegister::CFG_REG_C, 0x01)?; // Data Ready signal pin isn't connected
        Ok(())
    }

    pub fn init_single(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
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

    /// 10, 20, 50, 100 Hz
    pub fn set_data_rate(
        &mut self,
        spi: &mut Spi3,
        rate: usize,
    ) -> nb::Result<(), SpiError> {
        let rate = match rate {
            0 | 1 | 2 | 3 => (rate as u8) << 2,
            _ => panic!("set_data_rate, bad rate"),
        };
        let current = self.read_reg(spi, MagRegister::CFG_REG_A)?;
        self.write_reg(spi, MagRegister::CFG_REG_A, current | rate)?;
        Ok(())
    }

    pub fn set_offset_cancellation(
        &mut self,
        spi: &mut Spi3,
        enable: bool,
    ) -> nb::Result<(), SpiError> {
        let current = self.read_reg(spi, MagRegister::CFG_REG_B)?;

        let val = if enable {
            current & 0b0010
        } else {
            current & !(0b0010)
        };

        self.write_reg(spi, MagRegister::CFG_REG_B, val)?;

        Ok(())
    }

    pub fn set_hard_iron_offset(
        &mut self,
        spi: &mut Spi3,
        offset: [f32; 3],
    ) -> nb::Result<(), SpiError> {
        let (x_l, x_h) = Self::convert_raw_data_reverse(offset[0]);
        let (y_l, y_h) = Self::convert_raw_data_reverse(offset[1]);
        let (z_l, z_h) = Self::convert_raw_data_reverse(offset[2]);

        self.write_reg(spi, MagRegister::OFFSET_X_REG_H, x_h)?;
        self.write_reg(spi, MagRegister::OFFSET_X_REG_L, x_l)?;

        self.write_reg(spi, MagRegister::OFFSET_Y_REG_H, y_h)?;
        self.write_reg(spi, MagRegister::OFFSET_Y_REG_L, y_l)?;

        self.write_reg(spi, MagRegister::OFFSET_Z_REG_H, z_h)?;
        self.write_reg(spi, MagRegister::OFFSET_Z_REG_L, z_l)?;

        Ok(())
    }
}

/// read and convert data
impl<CS, PinError> Magnetometer<CS>
where
    CS: OutputPin<Error = PinError>,
{
    /// 1.5 milliGauss / LSB
    /// = 150 nT / LSB
    // const SCALE: f32 = 1.5;
    const SCALE: f32 = 150.0;

    pub fn read_new_data_available(
        &mut self,
        spi: &mut Spi3,
    ) -> nb::Result<bool, SpiError> {
        let status = self.read_reg(spi, MagRegister::STATUS_REG)?;
        Ok((status & 0b0000_1000) == 0b1000)
    }

    // pub fn read_temp(&mut self, spi: &mut Spi3) -> nb::Result<i16, SpiError> {
    //     let temp_l = self.read_reg(spi, MagRegister::TEMP_OUT_L_REG)?;
    //     let temp_h = self.read_reg(spi, MagRegister::TEMP_OUT_H_REG)?;
    //     // hprintln!("temp_l: {:#010b}", temp_l);
    //     // hprintln!("temp_h: {:#010b}", temp_h);
    //     Ok(0)
    // }

    pub fn read_data(&mut self, spi: &mut Spi3) -> nb::Result<[f32; 3], SpiError> {
        // pub fn read_data(&mut self, spi: &mut Spi3) -> nb::Result<[i16; 3], SpiError> {
        let mut data = [0u8; 6];

        self.read_reg_mult(spi, MagRegister::OUTX_L_REG, &mut data)?;

        let out = [
            Self::convert_raw_data(data[0], data[1]),
            Self::convert_raw_data(data[2], data[3]),
            Self::convert_raw_data(data[4], data[5]),
        ];

        Ok(out)
    }

    /// (low, high)
    fn convert_raw_data_reverse(x: f32) -> (u8, u8) {
        let x = (x / Self::SCALE) as i16;

        let h = x.overflowing_shr(8).0 as u8;
        let l = (x & 0b1111_1111) as u8;

        (l, h)
    }

    fn convert_raw_data(l: u8, h: u8) -> f32 {
        let v0 = l as i16 | ((h as i16) << 8);
        ((v0 as f32) / (i16::MAX as f32)) * Self::SCALE
        // v0
    }
}

/// read, write reg
impl<CS, PinError> Magnetometer<CS>
where
    CS: OutputPin<Error = PinError>,
{
    pub fn read_reg_mult(
        &mut self,
        spi: &mut Spi3,
        start_reg: MagRegister,
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
        reg: MagRegister,
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
        reg: MagRegister,
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
pub enum MagDataRate {
    R10  = 0b00,
    R20  = 0b01,
    R50  = 0b10,
    R100 = 0b11,
}

impl MagDataRate {
    fn to_val(self) -> u8 {
        (self as u8) << 2
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
    WHO_AM_I       = 0x4F,
    CFG_REG_A      = 0x60,
    CFG_REG_B      = 0x61,
    CFG_REG_C      = 0x62,
    INT_CTRL_REG   = 0x63,
    INT_SOURCE_REG = 0x64,
    INT_THS_L_REG  = 0x65,
    INT_THS_H_REG  = 0x66,
    STATUS_REG     = 0x67,
    OUTX_L_REG     = 0x68,
    OUTX_H_REG     = 0x69,
    OUTY_L_REG     = 0x6A,
    OUTY_H_REG     = 0x6B,
    OUTZ_L_REG     = 0x6C,
    OUTZ_H_REG     = 0x6D,
    TEMP_OUT_L_REG = 0x6E,
    TEMP_OUT_H_REG = 0x6F,
}

impl MagRegister {
    pub fn to_addr(self) -> u8 {
        self as u8
    }
}
