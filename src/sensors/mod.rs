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

#[derive(Debug)]
pub struct Sensors {
    spi:          Spi3,
    // spi:          SensSpi,
    imu:          IMU<Pin<'A', 8, Output>>,
    magnetometer: Magnetometer<Pin<'B', 12, Output>>,
    barometer:    Barometer<Pin<'C', 13, Output>>,

    pub data: SensorData,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct DataVal {
    data:    [f32; 3],
    changed: bool,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct SensorData {
    pub imu_acc:      DataVal,
    pub imu_gyro:     DataVal,
    pub magnetometer: DataVal,
    pub barometer:    DataVal,
}

impl DataVal {
    pub fn update(&mut self, data: [f32; 3]) {
        self.data = data;
        self.changed = true;
    }
    pub fn read_and_reset(&mut self) -> [f32; 3] {
        self.changed = false;
        self.data
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
            data: SensorData::default(),
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
    pub fn read_data_imu(&mut self) {
        if let Ok((data_gyro, data_acc)) =
            self.with_spi_imu(|spi, imu| imu.read_data(spi))
        {
            self.data.imu_gyro.update(data_gyro);
            self.data.imu_acc.update(data_acc);
        } else {
            unimplemented!()
        }
    }

    pub fn read_data_mag(&mut self) {
        if let Ok(data) = self.with_spi_mag(|spi, mag| mag.read_data(spi)) {
            self.data.magnetometer.update(data);
        } else {
            unimplemented!()
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
