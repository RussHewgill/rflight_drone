use fugit::{HertzU32, RateExtU32};

use nalgebra::{self as na};

use super::V3;

pub use self::iir::IIRFilter;

#[derive(Clone, Copy)]
pub struct SensorFilters {
    gyro_iir: IIRFilter,
}

/// new
impl SensorFilters {
    pub fn new() -> Self {
        Self {
            gyro_iir: IIRFilter::default(),
        }
    }
}

/// update
impl SensorFilters {
    pub fn update_gyro(&mut self, gyro: V3) -> V3 {
        // let gyro = self.gyro_iir.iir_update(gyro);

        gyro
    }

    pub fn update_acc(&mut self, acc: V3) -> V3 {
        acc
    }

    pub fn update_mag(&mut self, mag: V3) -> V3 {
        mag
    }

    pub fn update_baro(&mut self, baro: V3) -> V3 {
        baro
    }
}

mod iir {
    use super::V3;

    #[derive(Default, Clone, Copy)]
    pub struct IIRFilter {
        gyro_x_pre: [V3; 2],
        gyro_y_pre: [V3; 2],
    }

    impl IIRFilter {
        pub fn iir_update(&mut self, gyro: V3) -> V3 {
            /// XXX: 100 hz, 800 hz ???
            let gyro_fil_coeff = (
                0.94280904158206336,  // 0, a1
                -0.33333333333333343, // 1, a2
                0.09763107293781749,  // 2, b0
                0.19526214587563498,  // 3, b1
                0.09763107293781749,  // 4, b2
            );

            let gyro_fil_x = gyro_fil_coeff.2 * gyro.x
                + gyro_fil_coeff.3 * self.gyro_x_pre[0].x
                + gyro_fil_coeff.4 * self.gyro_x_pre[1].x
                + gyro_fil_coeff.0 * self.gyro_y_pre[0].x
                + gyro_fil_coeff.1 * self.gyro_y_pre[1].x;
            let gyro_fil_y = gyro_fil_coeff.2 * gyro.y
                + gyro_fil_coeff.3 * self.gyro_x_pre[0].y
                + gyro_fil_coeff.4 * self.gyro_x_pre[1].y
                + gyro_fil_coeff.0 * self.gyro_y_pre[0].y
                + gyro_fil_coeff.1 * self.gyro_y_pre[1].y;
            let gyro_fil_z = gyro_fil_coeff.2 * gyro.z
                + gyro_fil_coeff.3 * self.gyro_x_pre[0].z
                + gyro_fil_coeff.4 * self.gyro_x_pre[1].z
                + gyro_fil_coeff.0 * self.gyro_y_pre[0].z
                + gyro_fil_coeff.1 * self.gyro_y_pre[1].z;

            let gyro_fil = V3::new(gyro_fil_x, gyro_fil_y, gyro_fil_z);

            self.gyro_x_pre[1] = self.gyro_x_pre[0];
            self.gyro_y_pre[1] = self.gyro_y_pre[0];

            self.gyro_x_pre[0] = gyro;
            self.gyro_y_pre[0] = gyro_fil;

            gyro_fil
        }
    }
}
