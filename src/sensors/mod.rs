pub mod barometer;
pub mod imu;
pub mod magneto;

use stm32f4xx_hal::gpio::{Output, Pin};

use crate::spi::Spi3;

use self::{barometer::Barometer, imu::IMU, magneto::Magnetometer};

use derive_new::new;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SensorType {
    IMU,
    Mag,
    Baro,
}

#[derive(new)]
pub struct Sensors {
    spi: Spi3,
    imu: IMU<Pin<'A', 8, Output>>,
    magnetometer: Magnetometer<Pin<'B', 12, Output>>,
    barometer: Barometer<Pin<'C', 13, Output>>,
}

impl Sensors {
    // pub fn read_reg(&mut self, )
}

/// getters
impl Sensors {
    pub fn get_imu(&mut self) -> (&mut Spi3, &mut IMU<Pin<'A', 8, Output>>) {
        (&mut self.spi, &mut self.imu)
    }
    pub fn get_mag(&mut self) -> (&mut Spi3, &mut Magnetometer<Pin<'B', 12, Output>>) {
        (&mut self.spi, &mut self.magnetometer)
    }
    pub fn get_baro(&mut self) -> (&mut Spi3, &mut Barometer<Pin<'C', 13, Output>>) {
        (&mut self.spi, &mut self.barometer)
    }
}
