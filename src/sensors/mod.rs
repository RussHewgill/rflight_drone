pub mod barometer;
pub mod imu;
pub mod magneto;

use stm32f4::stm32f401::SPI2;
use stm32f4xx_hal::gpio::{Alternate, Output, Pin};

use crate::spi::Spi3;

use self::{barometer::Barometer, imu::IMU, magneto::Magnetometer};

// #[derive(Debug, Clone, Copy, PartialEq)]
// pub enum SensorType {
//     IMU,
//     Mag,
//     Baro,
// }

// pub type SensSpi = stm32f4xx_hal::spi::Spi<
//     SPI2,
//     (
//         Pin<'B', 13, Alternate<5>>,
//         stm32f4xx_hal::gpio::NoPin,
//         Pin<'B', 15, Alternate<5>>,
//     ),
//     stm32f4xx_hal::spi::TransferModeBidi,
// >;

pub struct Sensors {
    spi:          Spi3,
    // spi:          SensSpi,
    imu:          IMU<Pin<'A', 8, Output>>,
    magnetometer: Magnetometer<Pin<'B', 12, Output>>,
    barometer:    Barometer<Pin<'C', 13, Output>>,
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

impl Sensors {
    // pub fn read_reg(&mut self, )

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

/// getters
impl Sensors {
    // pub fn get_imu(&mut self) -> (&mut Spi3, &mut IMU<Pin<'A', 8, Output>>) {
    //     (&mut self.spi, &mut self.imu)
    // }
    // pub fn get_mag(&mut self) -> (&mut Spi3, &mut Magnetometer<Pin<'B', 12, Output>>) {
    //     (&mut self.spi, &mut self.magnetometer)
    // }
    // pub fn get_baro(&mut self) -> (&mut Spi3, &mut Barometer<Pin<'C', 13, Output>>) {
    //     (&mut self.spi, &mut self.barometer)
    // }
}
