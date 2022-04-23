// use cortex_m_semihosting::hprintln;

use embedded_hal as hal;
use hal::spi::{
    self,
    // blocking::{Read, Transfer, Write},
};

use stm32f4xx_hal::nb;
use stm32f4xx_hal::{
    hal::digital::v2::{InputPin, OutputPin},
    prelude::*,
};

use derive_new::new;

use crate::spi::{Spi3, SpiError};

/// https://www.st.com/resource/en/datasheet/lsm6dsl.pdf

// #[derive(new)]
#[derive(Debug)]
pub struct IMU<CS> {
    cs:         CS,
    acc_scale:  AccScale,
    gyro_scale: GyroScale,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccScale {
    S2  = 2,
    S4  = 4,
    S8  = 8,
    S16 = 16,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GyroScale {
    S250  = 250,
    S500  = 500,
    S1000 = 1000,
    S2000 = 2000,
}

impl<CS> IMU<CS> {
    pub fn new(cs: CS) -> Self {
        Self {
            cs,
            acc_scale: AccScale::S2,
            gyro_scale: GyroScale::S250,
        }
    }
}

const SPI_READ: u8 = 0x80; // 0x01 << 7
const SPI_WRITE: u8 = 0x00;

impl<CS, PinError> IMU<CS>
where
    // CS: hal::digital::blocking::OutputPin<Error = PinError>,
    CS: OutputPin<Error = PinError>,
{
    pub fn reset(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        unimplemented!()
    }

    pub fn init(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        let val = 0b0000_0000
            // | 0b0100_0000 // Block Data Update
            | 0b1000 // SIM
            | 0b0100; // IF_INC

        // /// BDU = 1, Block Data Update
        /// SIM = 1, 3 wire SPI mode
        /// IF_INC = 1, increment on multiple byte access
        self.write_reg(spi, IMURegister::CTRL3_C, val)?;

        /// Accelerometer
        /// ODR_XL = 0100 = 104 Hz Normal mode
        self.write_reg(spi, IMURegister::CTRL1_XL, 0b0100_0000)?;

        /// Gyro
        /// ODR_GL = 0100 = 104 Hz Normal mode
        self.write_reg(spi, IMURegister::CTRL2_G, 0b0100_0000)?;

        Ok(())
    }

    pub fn power_down(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        self.write_reg(spi, IMURegister::CTRL1_XL, 0b0000_0000)?;
        self.write_reg(spi, IMURegister::CTRL2_G, 0b0000_0000)?;
        Ok(())
    }
}

impl<CS, PinError> IMU<CS>
where
    // CS: hal::digital::blocking::OutputPin<Error = PinError>,
    CS: OutputPin<Error = PinError>,
{
    pub fn read_new_data_available(
        &mut self,
        spi: &mut Spi3,
    ) -> nb::Result<[bool; 3], SpiError> {
        const TDA: u8 = 0b0100;
        const GDA: u8 = 0b0010;
        const XLDA: u8 = 0b0001;

        let b = self.read_reg(spi, IMURegister::STATUS_REG)?;

        Ok([(b & TDA) == TDA, (b & GDA) == GDA, (b & XLDA) == XLDA])
    }

    pub fn read_data(
        &mut self,
        spi: &mut Spi3,
    ) -> nb::Result<([f32; 3], [f32; 3]), SpiError> {
        // let mut data = [0u8; 14]; // extra for temp
        let mut data = [0u8; 12];

        self.read_reg_mult(spi, IMURegister::OUTX_L_G, &mut data)?;

        let gyro_data = &data[0..6];

        let gyro = [
            Self::convert_raw_data(gyro_data[0], gyro_data[1], self.gyro_scale as u8),
            Self::convert_raw_data(gyro_data[2], gyro_data[3], self.gyro_scale as u8),
            Self::convert_raw_data(gyro_data[4], gyro_data[5], self.gyro_scale as u8),
        ];

        let acc_data = &data[6..12];

        let acc = [
            Self::convert_raw_data(acc_data[0], acc_data[1], self.acc_scale as u8),
            Self::convert_raw_data(acc_data[2], acc_data[3], self.acc_scale as u8),
            Self::convert_raw_data(acc_data[4], acc_data[5], self.acc_scale as u8),
        ];

        Ok((gyro, acc))
    }

    // pub fn read_accel_data(&mut self, spi: &mut Spi3) -> nb::Result<[f32; 3], SpiError> {
    //     // let outx_l = self.read_reg(spi, IMURegister::OUTX_L_XL)?;
    //     // let outx_h = self.read_reg(spi, IMURegister::OUTX_H_XL)?;

    //     // let outy_l = self.read_reg(spi, IMURegister::OUTY_L_XL)?;
    //     // let outy_h = self.read_reg(spi, IMURegister::OUTY_H_XL)?;

    //     // let outz_l = self.read_reg(spi, IMURegister::OUTZ_L_XL)?;
    //     // let outz_h = self.read_reg(spi, IMURegister::OUTZ_H_XL)?;

    //     let mut data = [0u8; 6];

    //     self.read_reg_mult(spi, IMURegister::OUTX_L_XL, bytes)

    //     Ok([
    //         Self::convert_raw_data(outx_h, outx_l, self.acc_scale),
    //         Self::convert_raw_data(outy_h, outy_l, self.acc_scale),
    //         Self::convert_raw_data(outz_h, outz_l, self.acc_scale),
    //     ])
    // }
    // pub fn read_gyro_data(&mut self, spi: &mut Spi3) -> nb::Result<[f32; 3], SpiError> {
    //     let outx_l = self.read_reg(spi, IMURegister::OUTX_L_G)?;
    //     let outx_h = self.read_reg(spi, IMURegister::OUTX_H_G)?;

    //     let outy_l = self.read_reg(spi, IMURegister::OUTY_L_G)?;
    //     let outy_h = self.read_reg(spi, IMURegister::OUTY_H_G)?;

    //     let outz_l = self.read_reg(spi, IMURegister::OUTZ_L_G)?;
    //     let outz_h = self.read_reg(spi, IMURegister::OUTZ_H_G)?;

    //     Ok([
    //         Self::convert_raw_data(outx_h, outx_l, self.gyro_scale),
    //         Self::convert_raw_data(outy_h, outy_l, self.gyro_scale),
    //         Self::convert_raw_data(outz_h, outz_l, self.gyro_scale),
    //     ])
    // }

    fn convert_raw_data(l: u8, h: u8, scale: u8) -> f32 {
        let v0 = l as i16 | ((h as i16) << 8);
        ((v0 as f32) / (i16::MAX as f32)) * scale as f32
    }
}

impl<CS, PinError> IMU<CS>
where
    // CS: hal::digital::blocking::OutputPin<Error = PinError>,
    CS: OutputPin<Error = PinError>,
{
    pub fn read_reg_mult(
        &mut self,
        spi: &mut Spi3,
        start_reg: IMURegister,
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
        reg: IMURegister,
    ) -> nb::Result<u8, SpiError> {
        let mut out = 0u8;
        let addr = reg.to_addr() | SPI_READ;

        self.cs.set_low().ok();

        spi.send_blocking(addr)?;
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

        spi.send_blocking(addr)?;
        spi.send_blocking(val)?;

        self.cs.set_high().ok();
        Ok(())
    }
}

#[cfg(feature = "nope")]
mod prev {

    pub struct IMU<SPI, CS> {
        spi: SPI,
        cs:  CS,
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
    FUNC_CFG_ACCESS          = 0x01,
    // RESERVED = 0x02,
    // RESERVED = 0x03,
    SENSOR_SYNC_TIME_FRAME   = 0x04,
    SENSOR_SYNC_RES_RATIO    = 0x05,
    FIFO_CTRL1               = 0x06,
    FIFO_CTRL2               = 0x07,
    FIFO_CTRL3               = 0x08,
    FIFO_CTRL4               = 0x09,
    FIFO_CTRL5               = 0x0A,
    DRDY_PULSE_CFG_G         = 0x0B,
    // RESERVED = 0x0C,
    INT1_CTRL                = 0x0D,
    INT2_CTRL                = 0x0E,
    WHO_AM_I                 = 0x0F,
    CTRL1_XL                 = 0x10,
    CTRL2_G                  = 0x11,
    CTRL3_C                  = 0x12,
    CTRL4_C                  = 0x13,
    CTRL5_C                  = 0x14,
    CTRL6_C                  = 0x15,
    CTRL7_G                  = 0x16,
    CTRL8_XL                 = 0x17,
    CTRL9_XL                 = 0x18,
    CTRL10_C                 = 0x19,

    MASTER_CONFIG            = 0x1A,
    WAKE_UP_SRC              = 0x1B,
    TAP_SRC                  = 0x1C,
    D6D_SRC                  = 0x1D,
    STATUS_REG               = 0x1E,
    RESERVED                 = 0x1F,
    OUT_TEMP_L               = 0x20,
    OUT_TEMP_H               = 0x21,
    OUTX_L_G                 = 0x22,
    OUTX_H_G                 = 0x23,
    OUTY_L_G                 = 0x24,
    OUTY_H_G                 = 0x25,
    OUTZ_L_G                 = 0x26,
    OUTZ_H_G                 = 0x27,
    OUTX_L_XL                = 0x28,
    OUTX_H_XL                = 0x29,
    OUTY_L_XL                = 0x2A,
    OUTY_H_XL                = 0x2B,
    OUTZ_L_XL                = 0x2C,
    OUTZ_H_XL                = 0x2D,
    SENSORHUB1_REG           = 0x2E,
    SENSORHUB2_REG           = 0x2F,
    SENSORHUB3_REG           = 0x30,
    SENSORHUB4_REG           = 0x31,
    SENSORHUB5_REG           = 0x32,
    SENSORHUB6_REG           = 0x33,
    SENSORHUB7_REG           = 0x34,
    SENSORHUB8_REG           = 0x35,
    SENSORHUB9_REG           = 0x36,
    SENSORHUB10_REG          = 0x37,
    SENSORHUB11_REG          = 0x38,
    SENSORHUB12_REG          = 0x39,

    FIFO_STATUS1             = 0x3A,
    FIFO_STATUS2             = 0x3B,
    FIFO_STATUS3             = 0x3C,
    FIFO_STATUS4             = 0x3D,
    FIFO_DATA_OUT_L          = 0x3E,
    FIFO_DATA_OUT_H          = 0x3F,
    TIMESTAMP0_REG           = 0x40,
    TIMESTAMP1_REG           = 0x41,
    TIMESTAMP2_REG           = 0x42,
    // RESERVED = 0x43,
    STEP_TIMESTAMP_L         = 0x49,
    STEP_TIMESTAMP_H         = 0x4A,
    STEP_COUNTER_L           = 0x4B,
    STEP_COUNTER_H           = 0x4C,
    SENSORHUB13_REG          = 0x4D,
    SENSORHUB14_REG          = 0x4E,
    SENSORHUB15_REG          = 0x4F,
    SENSORHUB16_REG          = 0x50,
    SENSORHUB17_REG          = 0x51,
    SENSORHUB18_REG          = 0x52,
    FUNC_SRC1                = 0x53,
    FUNC_SRC2                = 0x54,
    WRIST_TILT_IA            = 0x55,
    // RESERVED = 0x56,
    TAP_CFG                  = 0x58,
    TAP_THS_6D               = 0x59,
    INT_DUR2                 = 0x5A,
    WAKE_UP_THS              = 0x5B,
    WAKE_UP_DUR              = 0x5C,
    FREE_FALL                = 0x5D,
    MD1_CFG                  = 0x5E,
    MD2_CFG                  = 0x5F,
    MASTER_CMD_CODE          = 0x60,

    SENS_SYNC_SPI_ERROR_CODE = 0x61,
    // RESERVED = 0x62,
    OUT_MAG_RAW_X_L          = 0x66,
    OUT_MAG_RAW_X_H          = 0x67,
    OUT_MAG_RAW_Y_L          = 0x68,
    OUT_MAG_RAW_Y_H          = 0x69,
    OUT_MAG_RAW_Z_L          = 0x6A,
    OUT_MAG_RAW_Z_H          = 0x6B,
    // RESERVED = 0x6C,
    X_OFS_USR                = 0x73,
    Y_OFS_USR                = 0x74,
    Z_OFS_USR                = 0x75,
    // RESERVED = 0x76,
}

impl IMURegister {
    pub fn to_addr(self) -> u8 {
        self as u8
    }
}
