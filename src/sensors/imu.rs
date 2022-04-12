// use cortex_m_semihosting::hprintln;

use embedded_hal as hal;
use hal::spi::{
    self,
    blocking::{Read, Transfer, Write},
};

use stm32f4xx_hal::nb;
use stm32f4xx_hal::prelude::*;

use derive_new::new;

use crate::spi::{Spi3, SpiError};

/// https://www.st.com/resource/en/datasheet/lsm6dsl.pdf

#[derive(new)]
pub struct IMU<CS> {
    cs: CS,
}

const SPI_READ: u8 = 0x80; // 0x01 << 7
const SPI_WRITE: u8 = 0x00;

impl<CS, PinError> IMU<CS>
where
    CS: hal::digital::blocking::OutputPin<Error = PinError>,
{
    pub fn reset(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        unimplemented!()
    }

    pub fn init(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        unimplemented!()
    }
}

impl<CS, PinError> IMU<CS>
where
    CS: hal::digital::blocking::OutputPin<Error = PinError>,
{
    pub fn read_reg(&mut self, spi: &mut Spi3, reg: IMURegister) -> nb::Result<u8, SpiError> {
        let mut out = 0u8;
        let addr = reg.to_addr() | SPI_READ;

        self.cs.set_low().ok();

        spi.send(addr)?;
        spi.read(&mut out)?;

        self.cs.set_high().ok();
        Ok(out)
    }

    pub fn write_reg(
        &mut self,
        spi: &mut Spi3,
        reg: IMURegister,
        val: u8,
    ) -> nb::Result<(), SpiError> {
        let addr = reg.to_addr() | SPI_WRITE;

        self.cs.set_low().ok();

        spi.send(addr)?;
        spi.send(val)?;

        self.cs.set_high().ok();
        Ok(())
    }
}

#[cfg(feature = "nope")]
mod prev {

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
        // SPI: Transfer<u8, Error = E> + Write<u8, Error = E> + Read<u8, Error = E>,
        SPI: Write<u8, Error = E> + Read<u8, Error = E>,
        CS: hal::digital::blocking::OutputPin<Error = PinError>,
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
            // let mut bytes = [(reg << 1) | SPI_READ, 0];
            self.cs.set_low().ok();
            //
            self.cs.set_high().ok();
            // buf[0] = bytes[1];
        }

        // pub fn read_reg(&mut self, reg: u8, buf: &mut [u8]) {
        //     let mut bytes = [(reg << 1) | SPI_WRITE, 0];
        //     self.cs.set_low().ok();
        //     // self.spi.transfer(&mut bytes).ok();
        //     self.cs.set_high().ok();
        //     buf[0] = bytes[1];
        // }
    }
}

#[derive(Debug)]
#[rustfmt::skip]
#[repr(u8)]
pub enum AccelPowerModes {
    PowerDown = 0b0000,
    Low26     = 0b0010,
    Low52     = 0b0011,
    Normal104 = 0b0100,
    Normal208 = 0b0101,
}

impl AccelPowerModes {
    pub fn to_val(self) -> u8 {
        (self as u8) << 4
    }
}

#[allow(non_camel_case_types)]
#[derive(Debug)]
#[repr(u8)]
pub enum IMURegister {
    // RESERVED = 0x00,
    FUNC_CFG_ACCESS = 0x01,
    // RESERVED = 0x02,
    // RESERVED = 0x03,
    SENSOR_SYNC_TIME_FRAME = 0x04,
    SENSOR_SYNC_RES_RATIO = 0x05,
    FIFO_CTRL1 = 0x06,
    FIFO_CTRL2 = 0x07,
    FIFO_CTRL3 = 0x08,
    FIFO_CTRL4 = 0x09,
    FIFO_CTRL5 = 0x0A,
    DRDY_PULSE_CFG_G = 0x0B,
    // RESERVED = 0x0C,
    INT1_CTRL = 0x0D,
    INT2_CTRL = 0x0E,
    WHO_AM_I = 0x0F,
    CTRL1_XL = 0x10,
    CTRL2_G = 0x11,
    CTRL3_C = 0x12,
    CTRL4_C = 0x13,
    CTRL5_C = 0x14,
    CTRL6_C = 0x15,
    CTRL7_G = 0x16,
    CTRL8_XL = 0x17,
    CTRL9_XL = 0x18,
    CTRL10_C = 0x19,

    MASTER_CONFIG = 0x1A,
    WAKE_UP_SRC = 0x1B,
    TAP_SRC = 0x1C,
    D6D_SRC = 0x1D,
    STATUS_REG = 0x1E,
    RESERVED = 0x1F,
    OUT_TEMP_L = 0x20,
    OUT_TEMP_H = 0x21,
    OUTX_L_G = 0x22,
    OUTX_H_G = 0x23,
    OUTY_L_G = 0x24,
    OUTY_H_G = 0x25,
    OUTZ_L_G = 0x26,
    OUTZ_H_G = 0x27,
    OUTX_L_XL = 0x28,
    OUTX_H_XL = 0x29,
    OUTY_L_XL = 0x2A,
    OUTY_H_XL = 0x2B,
    OUTZ_L_XL = 0x2C,
    OUTZ_H_XL = 0x2D,
    SENSORHUB1_REG = 0x2E,
    SENSORHUB2_REG = 0x2F,
    SENSORHUB3_REG = 0x30,
    SENSORHUB4_REG = 0x31,
    SENSORHUB5_REG = 0x32,
    SENSORHUB6_REG = 0x33,
    SENSORHUB7_REG = 0x34,
    SENSORHUB8_REG = 0x35,
    SENSORHUB9_REG = 0x36,
    SENSORHUB10_REG = 0x37,
    SENSORHUB11_REG = 0x38,
    SENSORHUB12_REG = 0x39,

    FIFO_STATUS1 = 0x3A,
    FIFO_STATUS2 = 0x3B,
    FIFO_STATUS3 = 0x3C,
    FIFO_STATUS4 = 0x3D,
    FIFO_DATA_OUT_L = 0x3E,
    FIFO_DATA_OUT_H = 0x3F,
    TIMESTAMP0_REG = 0x40,
    TIMESTAMP1_REG = 0x41,
    TIMESTAMP2_REG = 0x42,
    // RESERVED = 0x43,
    STEP_TIMESTAMP_L = 0x49,
    STEP_TIMESTAMP_H = 0x4A,
    STEP_COUNTER_L = 0x4B,
    STEP_COUNTER_H = 0x4C,
    SENSORHUB13_REG = 0x4D,
    SENSORHUB14_REG = 0x4E,
    SENSORHUB15_REG = 0x4F,
    SENSORHUB16_REG = 0x50,
    SENSORHUB17_REG = 0x51,
    SENSORHUB18_REG = 0x52,
    FUNC_SRC1 = 0x53,
    FUNC_SRC2 = 0x54,
    WRIST_TILT_IA = 0x55,
    // RESERVED = 0x56,
    TAP_CFG = 0x58,
    TAP_THS_6D = 0x59,
    INT_DUR2 = 0x5A,
    WAKE_UP_THS = 0x5B,
    WAKE_UP_DUR = 0x5C,
    FREE_FALL = 0x5D,
    MD1_CFG = 0x5E,
    MD2_CFG = 0x5F,
    MASTER_CMD_CODE = 0x60,

    SENS_SYNC_SPI_ERROR_CODE = 0x61,
    // RESERVED = 0x62,
    OUT_MAG_RAW_X_L = 0x66,
    OUT_MAG_RAW_X_H = 0x67,
    OUT_MAG_RAW_Y_L = 0x68,
    OUT_MAG_RAW_Y_H = 0x69,
    OUT_MAG_RAW_Z_L = 0x6A,
    OUT_MAG_RAW_Z_H = 0x6B,
    // RESERVED = 0x6C,
    X_OFS_USR = 0x73,
    Y_OFS_USR = 0x74,
    Z_OFS_USR = 0x75,
    // RESERVED = 0x76,
}

impl IMURegister {
    pub fn to_addr(self) -> u8 {
        self as u8
    }
}
