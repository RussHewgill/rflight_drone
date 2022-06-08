use fugit::{HertzU32, RateExtU32};

use nalgebra::{self as na};

use biquad::*;

use super::V3;

pub use self::biquad_wrapper::*;
pub use self::iir::*;
pub use self::pt1::*;
pub use self::rpm::*;

#[derive(Clone, Copy)]
pub struct SensorFilters {
    gyro_iir:          (bool, IIRFilter),
    // gyro_biquad_lowpass: (bool, BiquadFilter),
    gyro_biquad_notch: (bool, BiquadFilter),
}

/// new
impl SensorFilters {
    pub fn new() -> Self {
        // let gyro_biquad_lowpass =
        //     (true, BiquadFilter::new(200.hz(), 3330.hz(), Type::LowPass));

        let gyro_biquad_notch =
            (true, BiquadFilter::new(200.hz(), 3330.hz(), Type::Notch));

        Self {
            gyro_iir: (true, IIRFilter::default()),
            // gyro_biquad_lowpass,
            gyro_biquad_notch,
        }
    }
}

/// update
impl SensorFilters {
    pub fn update_gyro(&mut self, mut gyro: V3) -> V3 {
        //

        // if self.gyro_iir.0 {
        //     gyro = self.gyro_iir.1.iir_update(gyro);
        // }

        // if self.gyro_biquad_lowpass.0 {
        //     gyro = self.gyro_biquad_lowpass.1.apply(gyro);
        // }

        if self.gyro_biquad_notch.0 {
            gyro = self.gyro_biquad_notch.1.apply(gyro);
        }

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

mod biquad_wrapper {
    use super::V3;

    use biquad::*;

    #[derive(Clone, Copy)]
    pub struct BiquadFilter {
        x: DirectForm2Transposed<f32>,
        y: DirectForm2Transposed<f32>,
        z: DirectForm2Transposed<f32>,
    }

    /// new
    impl BiquadFilter {
        pub fn new(
            cutoff_freq: biquad::Hertz<f32>,
            sampling_freq: biquad::Hertz<f32>,
            filter_type: Type<f32>,
        ) -> Self {
            // let cutoff = 100.hz();
            // let sampling = 6660.hz();
            let coeffs = Coefficients::<f32>::from_params(
                filter_type,
                sampling_freq,
                cutoff_freq,
                Q_BUTTERWORTH_F32,
            )
            .unwrap();
            let x = DirectForm2Transposed::<f32>::new(coeffs);
            let y = DirectForm2Transposed::<f32>::new(coeffs);
            let z = DirectForm2Transposed::<f32>::new(coeffs);

            Self { x, y, z }
        }
    }

    /// apply
    impl BiquadFilter {
        pub fn apply(&mut self, input: V3) -> V3 {
            let x = self.x.run(input.x);
            let y = self.y.run(input.y);
            let z = self.z.run(input.z);

            V3::new(x, y, z)
        }
    }
}

mod pt1 {
    use core::f32::consts::PI;

    use super::V3;

    #[derive(Clone, Copy)]
    pub struct PT1Filter {
        state:       f32,
        cutoff_freq: f32,
    }

    // impl Default for PT1Filter {
    //     fn default() -> Self {
    //         Self::new()
    //     }
    // }

    impl PT1Filter {
        pub fn new(cutoff_freq: f32) -> Self {
            Self {
                state: 0.0,
                cutoff_freq,
            }
        }

        pub fn filter_gain(f_cut: f32, dt: f32) -> f32 {
            let rc = 1.0 / (2.0 * PI * f_cut);
            dt / (rc + dt)
        }

        pub fn apply(&mut self, input: f32) -> f32 {
            self.state = self.state + self.cutoff_freq * (input - self.state);
            self.state
        }
    }
}

mod rpm {
    use crate::flight_control::MotorOutputs;

    use super::BiquadFilter;
    use super::V3;
    use biquad::*;

    #[derive(Default, Clone, Copy)]
    pub struct RPMFilter {}

    impl RPMFilter {
        pub fn apply(&self, throttle: &MotorOutputs) {
            unimplemented!()
        }
    }

    #[derive(Default, Clone, Copy)]
    pub struct NotchFilter {
        // low_pass: BiquadFilter,
        // high_pass: BiquadFilter,
        // low_freq:  f32,
        // high_freq: f32,
        // band:      f32,
    }

    impl NotchFilter {
        pub fn new(low_freq: f32, high_freq: f32, sampling_freq: f32, band: f32) -> Self {
            // let lowpass =
            //     BiquadFilter::new(low_freq.hz(), sampling_freq.hz(), Type::Notch);

            // let highpass =
            //     BiquadFilter::new(high_freq.hz(), sampling_freq.hz(), Type::HighPass);

            Self {
                // low_freq,
                // high_freq,
                // band,
            }
        }
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
