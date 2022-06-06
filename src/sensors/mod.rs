pub mod ahrs;
pub mod barometer;
pub mod filtering;
pub mod imu;
pub mod magneto;

use fugit::{HertzU32, RateExtU32};
use stm32f4::stm32f401::SPI2;
use stm32f4xx_hal::gpio::{Alternate, Output, Pin};
use stm32f4xx_hal::nb;

use crate::spi::Spi3;

use nalgebra::{self as na, UnitQuaternion, Vector3};

use self::filtering::SensorFilters;
use self::{barometer::Barometer, imu::IMU, magneto::Magnetometer};

// #[derive(Clone, Copy, PartialEq)]
// pub enum SensorType {
//     IMU,
//     Mag,
//     Baro,
// }

pub type V3 = Vector3<f32>;
pub type UQuat = UnitQuaternion<f32>;
pub type Rot3 = na::Rotation3<f32>;

// #[derive()]
pub struct Sensors {
    spi:          Spi3,
    // spi:          SensSpi,
    imu:          IMU<Pin<'A', 8, Output>>,
    magnetometer: Magnetometer<Pin<'B', 12, Output>>,
    barometer:    Barometer<Pin<'C', 13, Output>>,
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
#[derive(Default, Clone, Copy)]
pub struct SensorData {
    pub imu_acc:          DataVal<V3>,
    pub imu_gyro:         DataVal<V3>,
    pub magnetometer:     DataVal<V3>,
    pub baro_pressure:    DataVal<f32>,
    pub baro_temperature: DataVal<f32>,
}

#[derive(Default, Clone, Copy)]
pub struct DataVal<T> {
    // data:    [f32; 3],
    data:    T,
    changed: bool,
}

impl<T: Copy> DataVal<T> {
    pub fn update(&mut self, data: T) {
        self.data = data;
        self.changed = true;
    }
    pub fn read_and_reset(&mut self) -> T {
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

impl Sensors {
    /// TODO: Baro rate
    // pub fn get_rates(&self) -> (HertzU32, HertzU32, HertzU32, HertzU32) {
    pub fn get_rates(&self) -> (HertzU32, HertzU32, HertzU32) {
        let cfg_imu = self.imu.get_cfg();

        let rate_gyro = cfg_imu.gyro_power.to_hertz();
        let rate_acc = cfg_imu.acc_power.to_hertz();

        let rate_mag = self.magnetometer.get_data_rate().to_hertz();

        // let rate_baro = 0u32.Hz();

        // (rate_gyro, rate_acc, rate_mag, rate_baro)
        (rate_gyro, rate_acc, rate_mag)
    }
}

/// read data directly
#[cfg(feature = "nope")]
impl Sensors {
    pub fn _read_data_imu(&mut self) -> Option<(V3, V3)> {
        self.with_spi_imu(|spi, imu| {
            // while !(imu.read_new_data_available(spi).unwrap().iter().any(|x| *x)) {
            //     cortex_m::asm::nop();
            // }

            if !(imu.read_new_data_available(spi).unwrap().iter().any(|x| *x)) {
                None
            } else {
                let (data_gyro, data_acc) = imu.read_data(spi).unwrap();
                let data_gyro = V3::new(
                    -data_gyro[1], // pitch
                    data_gyro[0],  // roll
                    data_gyro[2],  // yaw
                );
                /// roll, pitch, yaw to match na::Quat
                let data_acc = V3::new(
                    -data_acc[0], //
                    data_acc[1],
                    data_acc[2],
                );
                Some((data_gyro, data_acc))
            }
        })
    }

    pub fn _read_data_mag(&mut self) -> Option<V3> {
        self.with_spi_mag(|spi, mag| {
            // while !mag.read_new_data_available(spi).unwrap() {
            //     cortex_m::asm::nop();
            // }

            if !mag.read_new_data_available(spi).unwrap() {
                None
            } else {
                let mag_data = mag.read_data(spi).unwrap();

                /// from ST firmware
                /// works correctly with Fusion AHRS
                let mag_data = V3::new(
                    -mag_data[1], //
                    mag_data[0],
                    mag_data[2],
                );

                Some(mag_data)
            }
        })
    }
}

/// read data to SensorData, and swap axes
impl Sensors {
    /// skips updating if data not ready
    pub fn read_data_imu(
        &mut self,
        data: &mut SensorData,
        filters: &mut SensorFilters,
    ) -> bool {
        self.with_spi_imu(|spi, imu| {
            if imu.read_new_data_available(spi).unwrap().iter().any(|x| *x) {
                let (data_gyro, data_acc) = imu.read_data(spi).unwrap();

                let data_gyro = V3::new(
                    -data_gyro[1], // pitch
                    data_gyro[0],  // roll
                    data_gyro[2],  // yaw
                );

                let data_gyro = filters.update_gyro(data_gyro);

                data.imu_gyro.update(data_gyro);

                /// roll, pitch, yaw to match na::Quat
                let data_acc = V3::new(
                    -data_acc[0], //
                    data_acc[1],
                    data_acc[2],
                );

                let data_acc = filters.update_acc(data_acc);

                data.imu_acc.update(data_acc);
                true
            } else {
                false
            }
        })
    }

    #[cfg(feature = "nope")]
    pub fn read_data_mag(&mut self, data: &mut SensorData, filters: &mut SensorFilters) {
        self.with_spi_mag(|spi, mag| {
            if mag.read_new_data_available(spi).unwrap() {
                let mag_data = mag.read_data(spi).unwrap();

                /// from ST firmware
                /// works correctly with Fusion AHRS
                let mag_data = V3::new(
                    -mag_data[1], //
                    mag_data[0],
                    mag_data[2],
                );

                let mag_data = filters.update_mag(mag_data);

                data.magnetometer.update(mag_data);
            }
        });
    }
}

/// read data to SensorData, and swap axes
impl Sensors {
    #[cfg(feature = "nope")]
    pub fn read_data_imu(&mut self, data: &mut SensorData, filters: &mut SensorFilters) {
        if let Ok((data_gyro, data_acc)) = self.with_spi_imu(|spi, imu| {
            while !(imu.read_new_data_available(spi).unwrap().iter().any(|x| *x)) {
                cortex_m::asm::nop();
            }
            imu.read_data(spi)
        }) {
            let data_gyro = V3::new(
                -data_gyro[1], // pitch
                data_gyro[0],  // roll
                data_gyro[2],  // yaw
            );

            let data_gyro = filters.update_gyro(data_gyro);

            data.imu_gyro.update(data_gyro);

            // /// rot about x,y,z
            // let data_acc = [
            //     -data_acc[1], //
            //     data_acc[0],
            //     data_acc[2],
            // ];

            /// roll, pitch, yaw to match na::Quat
            let data_acc = V3::new(
                -data_acc[0], //
                data_acc[1],
                data_acc[2],
            );

            let data_acc = filters.update_acc(data_acc);

            data.imu_acc.update(data_acc);
        } else {
            // unimplemented!()
        }
    }

    // #[cfg(feature = "nope")]
    pub fn read_data_mag(&mut self, data: &mut SensorData) {
        if let Ok(mag_data) = self.with_spi_mag(|spi, mag| {
            if mag.read_new_data_available(spi).unwrap() {
                mag.read_data(spi)
            } else {
                Err(nb::Error::WouldBlock)
            }
        }) {
            //

            /// from ST firmware
            /// works correctly with Fusion AHRS
            let mag_data = V3::new(
                -mag_data[1], //
                mag_data[0],
                mag_data[2],
            );

            // /// from datasheets, wrong ??
            // let mag_data = V3::new(
            //     mag_data[0], //
            //     -mag_data[1],
            //     mag_data[2],
            // );

            data.magnetometer.update(mag_data);
        } else {
            // unimplemented!()
        }
    }

    pub fn read_data_baro(&mut self, data: &mut SensorData) {
        self.with_spi_baro(|spi, baro| {
            let (pressure, temp) = baro.read_new_data_available(spi).unwrap();
            if pressure {
                let pressure = baro.read_data(spi).unwrap();
                data.baro_pressure.update(pressure);
            }
            if temp {
                let temp = baro.read_temperature_data(spi).unwrap();
                data.baro_temperature.update(temp);
            }
        });

        // if let Ok(baro_data) = self.with_spi_baro(|spi, baro| {
        //     if baro.read_new_data_available(spi).unwrap().0 {
        //         baro.read_data(spi)
        //     } else {
        //         Err(nb::Error::WouldBlock)
        //     }
        // }) {
        //     unimplemented!()
        // }
    }
}

#[derive(Clone, Copy)]
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
