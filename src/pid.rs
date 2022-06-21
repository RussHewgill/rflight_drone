// use cortex_m_semihosting::hprintln;

use biquad::*;
use nalgebra::{self as na};

// use signalo::filters::mean::mean::Mean;
use signalo::{
    filters::convolve::{
        savitzky_golay::SavitzkyGolay, Config as ConvolveConfig, Convolve,
    },
    filters::median::Median,
    traits::{Filter, WithConfig},
};

use defmt::{println as rprintln, Format};

#[derive(Clone)]
pub struct PID {
    kp: f32,
    ki: f32,
    kd: f32,

    _ki: f32,
    _kd: f32,

    p_limit:      f32,
    i_limit:      f32,
    d_limit:      f32,
    output_limit: f32,

    /// in seconds
    sample_time: f32,

    integral:   f32,
    prev_input: Option<f32>,
    prev_error: Option<f32>,

    pub prev_output: PIDOutput,

    deriv_lowpass: Option<DirectForm2Transposed<f32>>,

    // deriv_filter:  Mean<f32, 3>,
    // deriv_filter:  Convolve<f32, 5>,
    // deriv_filter: Median<f32, 3>,
    setpoint: f32,
}

#[derive(Default, Clone, Copy, Format)]
pub struct PIDOutput {
    pub p:      f32,
    pub i:      f32,
    pub d:      f32,
    pub output: f32,
}

/// new
impl PID {
    pub fn new_limited(kp: f32, ki: f32, kd: f32, sample_time: f32) -> Self {
        let mut out = Self::new(kp, ki, kd, sample_time);
        // out.p_limit = 1.0;
        out.i_limit = 1.0;
        out.d_limit = 1.0;
        out.output_limit = 1.0;
        out
    }

    pub fn new(kp: f32, ki: f32, kd: f32, sample_time: f32) -> Self {
        Self {
            kp,
            ki,
            kd,

            _ki: ki * sample_time,
            _kd: kd / sample_time,

            // p_limit: (f32::MIN, f32::MAX),
            // i_limit: (f32::MIN, f32::MAX),
            // d_limit: (f32::MIN, f32::MAX),
            // output_limit: (f32::MIN, f32::MAX),
            p_limit: f32::MAX,
            i_limit: f32::MAX,
            d_limit: f32::MAX,
            output_limit: f32::MAX,

            sample_time,

            integral: 0.0,
            prev_input: None,
            prev_error: None,

            prev_output: PIDOutput::default(),

            deriv_lowpass: None,
            // deriv_filter: Mean::default(),
            // deriv_filter: Convolve::savitzky_golay(),
            // deriv_filter: Median::default(),
            setpoint: 0.0,
        }
    }
}

/// step
impl PID {
    pub fn step(&mut self, input: f32) -> f32 {
        // let (output, _, _, _) = self._step(input);
        let output = self._step(input);
        output.output
    }

    /// https://github.com/braincore/pid-rs
    pub fn _step(&mut self, input: f32) -> PIDOutput {
        let error = self.setpoint - input;

        let p_unbounded = error * self.kp;
        let p = Self::apply_limit(self.p_limit, p_unbounded);

        // rprintln!("error = {:?}", error);
        // rprintln!("p_unbounded = {:?}", p_unbounded);
        // rprintln!("p = {:?}", p);

        /// Mitigate output jumps when ki(t) != ki(t-1).
        /// While it's standard to use an error_integral that's a running sum of
        /// just the error (no ki), because we support ki changing dynamically,
        /// we store the entire term so that we don't need to remember previous
        /// ki values.
        self.integral = self.integral + error * self._ki;

        /// Mitigate integral windup: Don't want to keep building up error
        /// beyond what i_limit will allow.
        self.integral = Self::apply_limit(self.i_limit, self.integral);

        /// Mitigate derivative kick: Use the derivative of the measurement
        /// rather than the derivative of the error.
        let d_unbounded = -match self.prev_input.as_ref() {
            Some(prev_measurement) => input - *prev_measurement,
            None => 0.0,
        } * self._kd;

        // let d_unbounded = if self.prev_deriv.is_none() {
        //     /// Arducopter:
        //     /// we've just done a reset, suppress the first derivative
        //     /// term as we don't want a sudden change in input to cause
        //     /// a large D output change
        //     self.prev_deriv = Some(0.0);
        //     0.0
        // } else {
        //     -match self.prev_input.as_ref() {
        //         Some(prev_measurement) => input - *prev_measurement,
        //         None => 0.0,
        //     } * self.kd
        // };

        // /// Arducopter:
        // /// discrete low pass filter, cuts out the
        // /// high frequency noise that can drive the controller crazy
        // const F_CUT = 20.0;
        // let rc = 1.0 / (2.0 * core::f32::consts::PI * F_CUT);
        // let d = if let Some(pd) = self.prev_deriv {
        //     pd +
        // } else {
        //     d
        // };

        let d_filtered = if let Some(lowpass) = self.deriv_lowpass.as_mut() {
            let d0 = lowpass.run(d_unbounded);
            // self.deriv_filter.filter(d_unbounded)
            // let d1 = self.deriv_filter.filter(d0);
            d0
        } else {
            d_unbounded
        };

        // let d_filtered = self.deriv_filter.filter(d_unbounded);
        // let d_filtered = d_unbounded;

        let d = Self::apply_limit(self.d_limit, d_filtered);

        let output = p + self.integral + d;
        let output = Self::apply_limit(self.output_limit, output);

        self.prev_error = Some(error);
        self.prev_input = Some(input);
        let out = PIDOutput {
            p,
            i: self.integral,
            d,
            output,
        };

        self.prev_output = out;

        out
    }
}

/// reset, apply limit
impl PID {
    /// Also derivative
    pub fn reset_integral(&mut self) {
        self.integral = 0.0;
        // self.prev_deriv = None;
    }

    fn apply_limit(limit: f32, value: f32) -> f32 {
        use na::ComplexField;
        limit.min(value.abs()) * value.signum()
    }

    #[cfg(feature = "nope")]
    pub fn _step(&mut self, input: f32) -> (f32, f32, f32, f32) {
        let error = self.setpoint - input;

        let mut p = error * self.kp;
        p = p.clamp(self.p_limit.0, self.p_limit.1);

        self.integral = self.integral + error * self.ki;

        self.integral = self.integral.clamp(self.i_limit.0, self.i_limit.1);

        let mut d = if self.prev_deriv.is_none() {
            self.prev_deriv = Some(0.0);
            0.0
        } else {
            -match self.prev_input.as_ref() {
                Some(prev_measurement) => input - *prev_measurement,
                None => 0.0,
            } * self.kd
        };
        self.prev_input = Some(input);
        d = d.clamp(self.d_limit.0, self.d_limit.1);

        let mut output = p + self.integral + d;

        output = output.clamp(self.output_limit.0, self.output_limit.1);

        const MAX_HIST: usize = 1000;

        // if self.p_hist.len() > MAX_HIST {
        //     self.p_hist.pop_back();
        // }
        // self.p_hist.push_front(p);

        // if self.i_hist.len() > MAX_HIST {
        //     self.i_hist.pop_back();
        // }
        // self.i_hist.push_front(self.integral);

        // if self.d_hist.len() > MAX_HIST {
        //     self.d_hist.pop_back();
        // }
        // self.d_hist.push_front(d);

        // if self.output_hist.len() > MAX_HIST {
        //     self.output_hist.pop_back();
        // }
        // self.output_hist.push_front(output);

        // self.prev_output = Some(output);
        (output, p, self.integral, d)
    }
}

/// set param
impl PID {
    pub fn set_param(&mut self, param: PIDParam, val: f32) {
        use self::PIDParam::*;
        #[rustfmt::skip]
        match param {
            Kp        => self.kp = val,
            Ki        => self.set_ki(val),
            Kd        => self.set_kd(val),
            KpLimit     => self.p_limit = val,
            KiLimit     => {
                if self.i_limit != val {
                    self.reset_integral();
                }
                self.i_limit = val;
            },
            KdLimit     => self.d_limit = val,
            OutputLimit => self.output_limit = val,
        }
    }

    pub fn set_kp(&mut self, kp: f32) {
        self.kp = kp;
    }

    pub fn set_ki(&mut self, ki: f32) {
        if self.ki != ki {
            self.reset_integral();
        }
        self.ki = ki;
        self._ki = ki * self.sample_time;
    }

    pub fn set_kd(&mut self, kd: f32) {
        self.kd = kd;
        self._kd = kd / self.sample_time;
    }
}

/// set lowpass filter
impl PID {
    pub fn set_d_lowpass(&mut self, sampling_freq: f32, cutoff_freq: f32) {
        let coeffs = Coefficients::<f32>::from_params(
            Type::LowPass,
            sampling_freq.hz(),
            cutoff_freq.hz(),
            Q_BUTTERWORTH_F32,
            // q_value,
        )
        .unwrap();

        self.deriv_lowpass = Some(DirectForm2Transposed::<f32>::new(coeffs));
    }
}

/// copy settings
impl PID {
    pub fn copy_settings_to(&self, other: &mut PID) {
        other.kp = self.kp;
        other.ki = self.ki;
        other.kd = self.kd;

        other.p_limit = self.p_limit;
        other.i_limit = self.i_limit;
        other.d_limit = self.d_limit;
        other.output_limit = self.output_limit;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
#[repr(u8)]
pub enum PIDParam {
    Kp,
    Ki,
    Kd,
    KpLimit,
    KiLimit,
    KdLimit,
    OutputLimit,
}

impl PIDParam {
    pub const ITER: [PIDParam; 7] = [
        PIDParam::Kp,
        PIDParam::Ki,
        PIDParam::Kd,
        PIDParam::KpLimit,
        PIDParam::KiLimit,
        PIDParam::KdLimit,
        PIDParam::OutputLimit,
    ];

    pub fn from_u8(p: u8) -> Option<Self> {
        match p {
            0 => Some(Self::Kp),
            1 => Some(Self::Ki),
            2 => Some(Self::Kd),
            3 => Some(Self::KpLimit),
            4 => Some(Self::KiLimit),
            5 => Some(Self::KdLimit),
            6 => Some(Self::OutputLimit),
            _ => None,
        }
    }
}

// impl core::ops::Index<PIDParam> for PID {
//     type Output = f32;
//     fn index(&self, index: PIDParam) -> &Self::Output {
//         match index {
//             PIDParam::Kp => &self.kp,
//             PIDParam::Ki => &self.ki,
//             PIDParam::Kd => &self.kd,
//             PIDParam::KpLimit => &self.p_limit,
//             PIDParam::KiLimit => &self.i_limit,
//             PIDParam::KdLimit => &self.d_limit,
//             PIDParam::OutputLimit => &self.output_limit,
//         }
//     }
// }

// impl core::ops::IndexMut<PIDParam> for PID {
//     fn index_mut(&mut self, index: PIDParam) -> &mut Self::Output {
//         match index {
//             PIDParam::Kp => &mut self.kp,
//             PIDParam::Ki => &mut self.ki,
//             PIDParam::Kd => &mut self.kd,
//             PIDParam::KpLimit => &mut self.p_limit,
//             PIDParam::KiLimit => &mut self.i_limit,
//             PIDParam::KdLimit => &mut self.d_limit,
//             PIDParam::OutputLimit => &mut self.output_limit,
//         }
//     }
// }
