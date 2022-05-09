pub mod ahrs;
pub mod barometer;
pub mod imu;
pub mod magneto;

use stm32f4::stm32f401::SPI2;
use stm32f4xx_hal::gpio::{Alternate, Output, Pin};
use stm32f4xx_hal::nb;

use crate::spi::Spi3;

use nalgebra::{self as na, UnitQuaternion, Vector3};

use self::{barometer::Barometer, imu::IMU, magneto::Magnetometer};

// #[derive(Debug, Clone, Copy, PartialEq)]
// pub enum SensorType {
//     IMU,
//     Mag,
//     Baro,
// }

pub type V3 = Vector3<f32>;
pub type UQuat = UnitQuaternion<f32>;
pub type Rot3 = na::Rotation3<f32>;

#[derive(Debug)]
pub struct Sensors {
    spi:          Spi3,
    // spi:          SensSpi,
    imu:          IMU<Pin<'A', 8, Output>>,
    magnetometer: Magnetometer<Pin<'B', 12, Output>>,
    barometer:    Barometer<Pin<'C', 13, Output>>,

    // pub data: SensorData,
    pub offset_gyro: V3,
    pub offset_acc:  V3,
    pub offset_mag:  V3,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct DataVal {
    // data:    [f32; 3],
    data:    V3,
    changed: bool,
}

/// Board: (battery connector in upper right, pointing up)
///     +X = Right
///     +Y = Up
///     +Z = Normal
/// Acc: +ve g means that axis is pointing up
///     +X = Up
///     +Y = Left
///     +Z = Normal
/// Gyro: (p,r,q)
///     +p = CW rotation about x
///     +r = CW rotation about y
///     +y = CW rotation about z
/// Mag: (x,y,z)
///     +X = Right
///     +Y = Down
///     +Z = Normal
#[derive(Debug, Default, Clone, Copy)]
pub struct SensorData {
    pub imu_acc:      DataVal,
    pub imu_gyro:     DataVal,
    pub magnetometer: DataVal,
    pub barometer:    DataVal,
}

impl DataVal {
    pub fn update(&mut self, data: [f32; 3]) {
        self.data = data.into();
        self.changed = true;
    }
    pub fn read_and_reset(&mut self) -> V3 {
        self.changed = false;
        self.data
    }
    pub fn is_changed(&self) -> bool {
        self.changed
    }
}

/// new
impl Sensors {
    pub fn new(
        spi: Spi3,
        // spi: SensSpi,
        imu: IMU<Pin<'A', 8, Output>>,
        magnetometer: Magnetometer<Pin<'B', 12, Output>>,
        barometer: Barometer<Pin<'C', 13, Output>>,
    ) -> Self {
        Self {
            spi,
            imu,
            magnetometer,
            barometer,
            // data: SensorData::default(),
            offset_gyro: V3::default(),
            offset_acc: V3::default(),
            offset_mag: V3::default(),
        }
    }
}

// /// set offsets
// impl Sensors {
//     pub fn set_offsets(&mut self) {
//         unimplemented!()
//     }
//     pub fn set_offset_gyro(&mut self, gyro: V3) {
//         unimplemented!()
//     }
// }

/// with_spi
impl Sensors {
    pub fn with_spi_imu<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(&mut Spi3, &mut IMU<Pin<'A', 8, Output>>) -> T,
    {
        f(&mut self.spi, &mut self.imu)
    }

    pub fn with_spi_mag<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(&mut Spi3, &mut Magnetometer<Pin<'B', 12, Output>>) -> T,
    {
        f(&mut self.spi, &mut self.magnetometer)
    }

    pub fn with_spi_baro<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(&mut Spi3, &mut Barometer<Pin<'C', 13, Output>>) -> T,
    {
        f(&mut self.spi, &mut self.barometer)
    }
}

/// read data
impl Sensors {
    pub fn read_data_imu(&mut self, data: &mut SensorData, discard: bool) {
        if let Ok((data_gyro, data_acc)) = self.with_spi_imu(|spi, imu| {
            while !(imu.read_new_data_available(spi).unwrap().iter().any(|x| *x)) {
                cortex_m::asm::nop();
            }
            imu.read_data(spi)
        }) {
            if !discard {
                let data_gyro = [
                    -data_gyro[1], // pitch
                    data_gyro[0],  // roll
                    data_gyro[2],  // yaw
                ];
                data.imu_gyro.update(data_gyro);

                let data_acc = [
                    -data_acc[1], //
                    data_acc[0],
                    data_acc[2],
                ];
                data.imu_acc.update(data_acc);
            }
        } else {
            // unimplemented!()
        }
    }

    pub fn read_data_mag(&mut self, data: &mut SensorData) {
        if let Ok(mag_data) = self.with_spi_mag(|spi, mag| {
            if mag.read_new_data_available(spi).unwrap() {
                mag.read_data(spi)
            } else {
                Err(nb::Error::WouldBlock)
            }
        }) {
            let mag_data = [
                -mag_data[1], //
                mag_data[0],
                mag_data[2],
            ];
            data.magnetometer.update(mag_data);
        } else {
            // unimplemented!()
        }
    }

    // TODO:
    pub fn read_data_baro(&mut self) {
        // if let Ok(data) = self.with_spi_baro(|spi, baro| baro.read_data(spi)) {
        //     self.data.barometer.update(data);
        // } else {
        //     unimplemented!()
        // }
        unimplemented!()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SensorOffset {
    time:     f32,
    t0:       f32,
    t1:       f32,
    finished: bool,

    gyro_calc_offset: V3,
    acc_calc_offset:  V3,

    gyro_offset: V3,
    acc_offset:  V3,
}

impl Default for SensorOffset {
    fn default() -> Self {
        Self {
            time:             0.0,
            t0:               1.0,
            t1:               2.0,
            finished:         false,
            gyro_calc_offset: V3::default(),
            acc_calc_offset:  V3::default(),
            gyro_offset:      V3::default(),
            acc_offset:       V3::default(),
        }
    }
}

impl SensorOffset {
    pub fn reset(&mut self) {
        self.time = 0.0;
        self.finished = false;
        self.gyro_calc_offset = V3::default();
        self.acc_calc_offset = V3::default();
        self.gyro_offset = V3::default();
        self.acc_offset = V3::default();
    }

    pub fn update(&mut self, delta_time: f32, gyro: V3, acc: V3) {
        if self.finished {
            return;
        }
        self.time += delta_time;
        if self.time > self.t0 {
            self.gyro_calc_offset += gyro;
            self.acc_calc_offset += acc;

            if self.time > self.t1 {
                self.gyro_offset = self.gyro_calc_offset * 0.00125;
                self.acc_offset = self.acc_calc_offset * 0.00125;

                self.finished = true;
            }
        }
    }
}
