pub mod ahrs;
pub mod barometer;
pub mod imu;
pub mod magneto;

use stm32f4::stm32f401::SPI2;
use stm32f4xx_hal::gpio::{Alternate, Output, Pin};
use stm32f4xx_hal::nb;

use crate::spi::Spi3;

use nalgebra::{UnitQuaternion, Vector3};

use self::{barometer::Barometer, imu::IMU, magneto::Magnetometer};

// #[derive(Debug, Clone, Copy, PartialEq)]
// pub enum SensorType {
//     IMU,
//     Mag,
//     Baro,
// }

pub type V3 = Vector3<f32>;
pub type UQuat = UnitQuaternion<f32>;

#[derive(Debug)]
pub struct Sensors {
    spi:          Spi3,
    // spi:          SensSpi,
    imu:          IMU<Pin<'A', 8, Output>>,
    magnetometer: Magnetometer<Pin<'B', 12, Output>>,
    barometer:    Barometer<Pin<'C', 13, Output>>,
    // pub data: SensorData,
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
/// Acc: (x,y,z) same as board
/// Gyro: (p,r,q)
///     +p = rotation about x
///     +r = rotation about y
///     +z = rotation about z
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
        }
    }
}

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
                data.imu_gyro.update(data_gyro);
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
