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

use self::config::ImuConfig;

/// https://www.st.com/resource/en/datasheet/lsm6dsl.pdf

// #[derive(new)]
#[derive(Debug)]
pub struct IMU<CS> {
    cs:  CS,
    cfg: ImuConfig,
}

/// new
impl<CS> IMU<CS> {
    pub fn new(cs: CS) -> Self {
        Self {
            cs,
            cfg: ImuConfig::default(),
        }
    }
}

const SPI_READ: u8 = 0x80; // 0x01 << 7
const SPI_WRITE: u8 = 0x00;

/// Init
impl<CS, PinError> IMU<CS>
where
    CS: OutputPin<Error = PinError>,
{
    pub fn reset(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        let val = 0b1; // reset
        self.write_reg(spi, IMURegister::CTRL3_C, val)?;
        Ok(())
    }

    #[cfg(feature = "nope")]
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

    pub fn init(&mut self, spi: &mut Spi3, cfg: ImuConfig) -> nb::Result<(), SpiError> {
        self.write_reg(spi, IMURegister::CTRL1_XL, cfg.reg_ctrl1_xl())?;
        self.write_reg(spi, IMURegister::CTRL2_G, cfg.reg_ctrl2_g())?;
        self.write_reg(spi, IMURegister::CTRL3_C, cfg.reg_ctrl3_c())?;

        self.write_reg(spi, IMURegister::CTRL4_C, cfg.reg_ctrl4_c())?;
        self.write_reg(spi, IMURegister::CTRL6_C, cfg.reg_ctrl6_c())?;
        self.write_reg(spi, IMURegister::CTRL7_G, cfg.reg_ctrl7_g())?;
        self.write_reg(spi, IMURegister::CTRL8_XL, cfg.reg_ctrl8_xl())?;

        Ok(())
    }

    pub fn power_down(&mut self, spi: &mut Spi3) -> nb::Result<(), SpiError> {
        self.write_reg(spi, IMURegister::CTRL1_XL, 0b0000_0000)?;
        self.write_reg(spi, IMURegister::CTRL2_G, 0b0000_0000)?;
        Ok(())
    }
}

impl<CS> IMU<CS> {
    pub fn get_cfg(&self) -> &ImuConfig {
        &self.cfg
    }
}

/// read data
impl<CS, PinError> IMU<CS>
where
    CS: OutputPin<Error = PinError>,
{
    /// temperature, gyro, accel
    pub fn read_new_data_available(
        &mut self,
        spi: &mut Spi3,
    ) -> nb::Result<(bool, bool, bool), SpiError> {
        const TDA: u8 = 0b0100;
        const GDA: u8 = 0b0010;
        const XLDA: u8 = 0b0001;

        let b = self.read_reg(spi, IMURegister::STATUS_REG)?;

        Ok(((b & TDA) == TDA, (b & GDA) == GDA, (b & XLDA) == XLDA))
    }

    pub fn read_data_gyro(&mut self, spi: &mut Spi3) -> nb::Result<[f32; 3], SpiError> {
        let mut gyro_data = [0u8; 6];
        self.read_reg_mult(spi, IMURegister::OUTX_L_G, &mut gyro_data)?;

        Ok([
            Self::convert_raw_data(
                gyro_data[0],
                gyro_data[1],
                self.cfg.gyro_scale.to_sensitivity(),
            ),
            Self::convert_raw_data(
                gyro_data[2],
                gyro_data[3],
                self.cfg.gyro_scale.to_sensitivity(),
            ),
            Self::convert_raw_data(
                gyro_data[4],
                gyro_data[5],
                self.cfg.gyro_scale.to_sensitivity(),
            ),
        ])
    }

    pub fn read_data_acc(&mut self, spi: &mut Spi3) -> nb::Result<[f32; 3], SpiError> {
        let mut acc_data = [0u8; 6];
        self.read_reg_mult(spi, IMURegister::OUTX_L_XL, &mut acc_data)?;

        Ok([
            Self::convert_raw_data(
                acc_data[0],
                acc_data[1],
                self.cfg.acc_scale.to_sensitivity(),
            ),
            Self::convert_raw_data(
                acc_data[2],
                acc_data[3],
                self.cfg.acc_scale.to_sensitivity(),
            ),
            Self::convert_raw_data(
                acc_data[4],
                acc_data[5],
                self.cfg.acc_scale.to_sensitivity(),
            ),
        ])
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
            Self::convert_raw_data(
                gyro_data[0],
                gyro_data[1],
                self.cfg.gyro_scale.to_sensitivity(),
            ),
            Self::convert_raw_data(
                gyro_data[2],
                gyro_data[3],
                self.cfg.gyro_scale.to_sensitivity(),
            ),
            Self::convert_raw_data(
                gyro_data[4],
                gyro_data[5],
                self.cfg.gyro_scale.to_sensitivity(),
            ),
        ];

        let acc_data = &data[6..12];

        let acc = [
            Self::convert_raw_data(
                acc_data[0],
                acc_data[1],
                self.cfg.acc_scale.to_sensitivity(),
            ),
            Self::convert_raw_data(
                acc_data[2],
                acc_data[3],
                self.cfg.acc_scale.to_sensitivity(),
            ),
            Self::convert_raw_data(
                acc_data[4],
                acc_data[5],
                self.cfg.acc_scale.to_sensitivity(),
            ),
        ];

        Ok((gyro, acc))
    }

    pub fn read_temperature_data(&mut self, spi: &mut Spi3) -> nb::Result<f32, SpiError> {
        let mut data = [0u8; 2];
        self.read_reg_mult(spi, IMURegister::OUT_TEMP_L, &mut data)?;

        let l = data[0];
        let h = data[1];

        let t = l as i16 | ((h as i16) << 8);

        let t = t as f32 / 256.0;
        let t = t + 25.0;

        Ok(t)
    }

    pub fn convert_raw_data(l: u8, h: u8, scale: f32) -> f32 {
        let v0 = l as i16 | ((h as i16) << 8);
        // ((v0 as f32) / (i16::MAX as f32)) * scale
        (v0 as f32) * scale
    }
}

/// read, write reg
impl<CS, PinError> IMU<CS>
where
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

pub mod config {

    pub use self::accel::*;
    pub use self::gyro::*;

    pub trait ImuSetting {
        fn to_val(self) -> u8;
    }

    #[derive(Debug, Clone, Copy)]
    pub struct ImuConfig {
        /// General
        pub block_data_update:           bool,
        /// Accelerometer
        pub acc_power:                   AccelPowerModes,
        pub acc_scale:                   AccelScaleFactor,
        pub acc_high_perf_mode_disable:  bool,
        /// Only used when data rate > 1.67 kHz
        pub acc_analog_lp_bandwidth:     AccelAnalogBandwidth,
        pub acc_digital_filter_config:   AccelDigFilterConfig,
        pub acc_filter_input_composite:  AccelInputComposite,
        /// Gyro
        pub gyro_power:                  GyroPowerModes,
        pub gyro_scale:                  GyroScaleFactor,
        pub gyro_high_perf_mode_disable: bool,
        pub gyro_hp_filter_enable:       bool,
        pub gyro_hp_filter_cutoff:       GyroHpFilterCutoff,
        pub gyro_lp_filter_enable:       bool,
        pub gyro_lp_bandwidth:           GyroLpBandwidth,
    }

    impl Default for ImuConfig {
        fn default() -> Self {
            Self {
                /// General
                block_data_update:           false,
                /// Accelerometer
                acc_power:                   AccelPowerModes::PowerDown,
                acc_scale:                   AccelScaleFactor::S2,
                acc_high_perf_mode_disable:  false,
                acc_analog_lp_bandwidth:     AccelAnalogBandwidth::BW1500,
                acc_digital_filter_config:   AccelDigFilterConfig::Odr2,
                acc_filter_input_composite:  AccelInputComposite::LowLatency,
                /// Gyro
                gyro_power:                  GyroPowerModes::PowerDown,
                gyro_scale:                  GyroScaleFactor::S250,
                gyro_high_perf_mode_disable: false,
                gyro_hp_filter_enable:       false,
                gyro_hp_filter_cutoff:       GyroHpFilterCutoff::C16,
                gyro_lp_filter_enable:       false,
                gyro_lp_bandwidth:           GyroLpBandwidth::Normal,
            }
        }
    }

    impl ImuConfig {
        pub fn reg_ctrl1_xl(&self) -> u8 {
            let mut out = 0b0000_0000;
            out |= self.acc_power.to_val();

            let out = self.acc_digital_filter_config.reg_ctrl1_xl(out);

            out
        }

        pub fn reg_ctrl2_g(&self) -> u8 {
            let mut out = 0b0000_0000;
            out |= self.gyro_power.to_val();
            out |= self.gyro_scale.to_val();
            out
        }

        pub fn reg_ctrl3_c(&self) -> u8 {
            let mut out = 0b0000_0000;

            out |= 0b0000_0100; // address increment

            out |= 0b0000_1000; // SPI 3-wire mode

            if self.block_data_update {
                out |= 0b0100_0000;
            }
            out
        }

        pub fn reg_ctrl4_c(&self) -> u8 {
            let mut out = 0b0000_0000;

            out |= 0b0100; // disable i2c interface

            if self.gyro_lp_filter_enable {
                out |= 0b0010;
            }
            out
        }

        // pub fn reg_ctrl5_c(&self) -> u8 {
        //     let mut out = 0b0000_0000;
        //     out
        // }

        pub fn reg_ctrl6_c(&self) -> u8 {
            let mut out = 0b0000_0000;

            if self.acc_high_perf_mode_disable {
                out |= 0b0001_0000;
            }

            out |= self.gyro_lp_bandwidth.to_val();

            out
        }

        pub fn reg_ctrl7_g(&self) -> u8 {
            let mut out = 0b0000_0000;

            if !self.gyro_high_perf_mode_disable {
                out |= !0b1000_0000;
            }
            if self.gyro_hp_filter_enable {
                out |= 0b0100_0000;
            }
            out |= self.gyro_hp_filter_cutoff.to_val();

            out
        }

        pub fn reg_ctrl8_xl(&self) -> u8 {
            let mut out = 0b0000_0000;

            if self.acc_filter_input_composite == AccelInputComposite::LowNoise {
                out |= 0b0000_1000;
            }

            let out = self.acc_digital_filter_config.reg_ctrl8_xl(out);

            out
        }

        // pub fn reg_ctrl9_xl(&self) -> u8 {
        //     let mut out = 0b0000_0000;
        //     out
        // }
    }

    mod accel {
        use fugit::HertzU32;

        use super::ImuSetting;

        #[derive(Debug, Clone, Copy)]
        #[repr(u8)]
        /// CTRL1_XL:
        ///   LPF1_BW_SEL
        /// CTRL8_XL:
        ///   HP_SLOPE_XL_EN
        ///   HPCF_XL[1:0]
        ///   LPF2_XL_EN
        ///   INPUT_COMPOSITE
        pub enum AccelDigFilterConfig {
            Odr2,
            Odr4,

            OdrLowPass50,
            OdrLowPass100,
            OdrLowPass9,
            OdrLowPass400,

            OdrHighPass4,
            OdrHighPass100,
            OdrHighPass9,
            OdrHighPass400,
        }

        impl AccelDigFilterConfig {
            pub fn reg_ctrl1_xl(self, mut b: u8) -> u8 {
                use self::AccelDigFilterConfig::*;
                match self {
                    Odr2 => b |= 0b00,
                    Odr4 => b |= 0b10,
                    _ => {}
                }
                b
            }

            pub fn reg_ctrl8_xl(self, mut b: u8) -> u8 {
                use self::AccelDigFilterConfig::*;
                match self {
                    Odr2 | Odr4 => b &= !(0b1000_0000),

                    OdrLowPass50 => {
                        b |= 0b1000_0000; // LPF2_XL_EN
                    }
                    OdrLowPass100 => {
                        b |= 0b1000_0000; // LPF2_XL_EN
                        b |= 0b0010_0000; // HPCF_XL[1:0]
                    }
                    OdrLowPass9 => {
                        b |= 0b1000_0000; // LPF2_XL_EN
                        b |= 0b0100_0000; // HPCF_XL[1:0]
                    }
                    OdrLowPass400 => {
                        b |= 0b1000_0000; // LPF2_XL_EN
                        b |= 0b0110_0000; // HPCF_XL[1:0]
                    }

                    OdrHighPass4 => {
                        b &= !(0b0000_1000); // INPUT_COMPOSITE
                    }
                    OdrHighPass100 => {
                        b &= !(0b0000_1000); // INPUT_COMPOSITE
                        b |= 0b0010_0000; // HPCF_XL[1:0]
                    }
                    OdrHighPass9 => {
                        b &= !(0b0000_1000); // INPUT_COMPOSITE
                        b |= 0b0100_0000; // HPCF_XL[1:0]
                    }
                    OdrHighPass400 => {
                        b &= !(0b0000_1000); // INPUT_COMPOSITE
                        b |= 0b0110_0000; // HPCF_XL[1:0]
                    }
                }
                b
            }
        }

        #[derive(Debug, Clone, Copy)]
        #[repr(u8)]
        pub enum AccelAnalogBandwidth {
            BW1500 = 0,
            BW400  = 1,
        }

        impl ImuSetting for AccelAnalogBandwidth {
            fn to_val(self) -> u8 {
                self as u8
            }
        }

        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[repr(u8)]
        pub enum AccelInputComposite {
            LowLatency = 0,
            LowNoise   = 1,
        }

        impl ImuSetting for AccelInputComposite {
            fn to_val(self) -> u8 {
                (self as u8) << 3
            }
        }

        #[derive(Debug, Clone, Copy)]
        #[repr(u8)]
        pub enum AccelScaleFactor {
            S2  = 2,
            S4  = 4,
            S8  = 8,
            S16 = 16,
        }

        impl ImuSetting for AccelScaleFactor {
            fn to_val(self) -> u8 {
                (self as u8) << 2
            }
        }

        impl AccelScaleFactor {
            pub fn to_sensitivity(self) -> f32 {
                match self {
                    Self::S2 => 0.000061,
                    Self::S4 => 0.000122,
                    Self::S8 => 0.000244,
                    Self::S16 => 0.000488,
                }
            }
        }

        #[derive(Debug, Clone, Copy)]
        #[repr(u8)]
        pub enum AccelPowerModes {
            PowerDown = 0b0000,
            Low1p6    = 0b1011,
            Low12p5   = 0b0001,
            Low26     = 0b0010,
            Low52     = 0b0011,
            Normal104 = 0b0100,
            Normal208 = 0b0101,
            High416   = 0b0110,
            High833   = 0b0111,
            High1660  = 0b1000,
            High3330  = 0b1001,
            High6660  = 0b1010,
        }

        impl AccelPowerModes {
            pub fn to_hertz(&self) -> HertzU32 {
                use self::AccelPowerModes::*;
                use fugit::RateExtU32;
                #[rustfmt::skip]
                match self {
                    PowerDown => 0u32.Hz(),
                    Low1p6    => 6u32.Hz(),
                    Low12p5   => 12u32.Hz(),
                    Low26     => 26u32.Hz(),
                    Low52     => 52u32.Hz(),
                    Normal104 => 104u32.Hz(),
                    Normal208 => 208u32.Hz(),
                    High416   => 416u32.Hz(),
                    High833   => 833u32.Hz(),
                    High1660  => 1660u32.Hz(),
                    High3330  => 3330u32.Hz(),
                    High6660  => 6660u32.Hz(),
                }
            }
        }

        impl ImuSetting for AccelPowerModes {
            fn to_val(self) -> u8 {
                (self as u8) << 4
            }
        }
    }

    mod gyro {
        use fugit::HertzU32;

        use super::ImuSetting;

        #[derive(Debug, Clone, Copy)]
        #[repr(u8)]
        pub enum GyroLpBandwidth {
            Normal     = 0b00,
            Narrow     = 0b01,
            VeryNarrow = 0b10,
            Wide       = 0b11,
        }

        impl ImuSetting for GyroLpBandwidth {
            fn to_val(self) -> u8 {
                // (self as u8) << 0
                self as u8
            }
        }

        #[derive(Debug, Clone, Copy)]
        #[repr(u8)]
        pub enum GyroScaleFactor {
            S250  = 0b00,
            S500  = 0b01,
            S1000 = 0b10,
            S2000 = 0b11,
        }

        impl ImuSetting for GyroScaleFactor {
            fn to_val(self) -> u8 {
                (self as u8) << 2
            }
        }

        impl GyroScaleFactor {
            pub fn to_sensitivity(self) -> f32 {
                match self {
                    Self::S250 => 0.00875,
                    Self::S500 => 0.0175,
                    Self::S1000 => 0.035,
                    Self::S2000 => 0.07,
                }
            }

            #[cfg(feature = "nope")]
            pub fn to_scale(self) -> f32 {
                match self {
                    Self::S250 => 250.0,
                    Self::S500 => 500.0,
                    Self::S1000 => 1000.0,
                    Self::S2000 => 2000.0,
                }
            }
        }

        #[derive(Debug, Clone, Copy)]
        #[repr(u8)]
        /// in milliHertz
        pub enum GyroHpFilterCutoff {
            C16   = 0b00,
            C65   = 0b01,
            C260  = 0b10,
            C1040 = 0b11,
        }

        impl ImuSetting for GyroHpFilterCutoff {
            fn to_val(self) -> u8 {
                (self as u8) << 4
            }
        }

        #[derive(Debug, Clone, Copy)]
        #[repr(u8)]
        pub enum GyroPowerModes {
            PowerDown = 0b0000,
            Low12p5   = 0b0001,
            Low26     = 0b0010,
            Low52     = 0b0011,
            Normal104 = 0b0100,
            Normal208 = 0b0101,
            High416   = 0b0110,
            High833   = 0b0111,
            High1660  = 0b1000,
            High3330  = 0b1001,
            High6660  = 0b1010,
        }

        impl GyroPowerModes {
            pub fn to_hertz(&self) -> HertzU32 {
                use self::GyroPowerModes::*;
                use fugit::RateExtU32;
                #[rustfmt::skip]
                match self {
                    PowerDown => 0u32.Hz(),
                    Low12p5   => 12u32.Hz(),
                    Low26     => 26u32.Hz(),
                    Low52     => 52u32.Hz(),
                    Normal104 => 104u32.Hz(),
                    Normal208 => 208u32.Hz(),
                    High416   => 416u32.Hz(),
                    High833   => 833u32.Hz(),
                    High1660  => 1660u32.Hz(),
                    High3330  => 3330u32.Hz(),
                    High6660  => 6660u32.Hz(),
                }
            }
        }

        impl ImuSetting for GyroPowerModes {
            fn to_val(self) -> u8 {
                (self as u8) << 4
            }
        }
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
