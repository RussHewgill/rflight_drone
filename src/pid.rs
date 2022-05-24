// use cortex_m_semihosting::hprintln;

use nalgebra::{self as na};

use defmt::Format;

#[derive(Debug, Clone, Copy, Format)]
pub struct PID {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,

    pub p_limit:      f32,
    pub i_limit:      f32,
    pub d_limit:      f32,
    pub output_limit: f32,

    pub integral:   f32,
    pub prev_input: Option<f32>,

    pub prev_output: PIDOutput,

    pub setpoint: f32,
}

#[derive(Debug, Default, Clone, Copy, Format)]
pub struct PIDOutput {
    pub p:      f32,
    pub i:      f32,
    pub d:      f32,
    pub output: f32,
}

impl Default for PID {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }
}

/// new
impl PID {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,

            // p_limit: (f32::MIN, f32::MAX),
            // i_limit: (f32::MIN, f32::MAX),
            // d_limit: (f32::MIN, f32::MAX),
            // output_limit: (f32::MIN, f32::MAX),
            p_limit: f32::MAX,
            i_limit: f32::MAX,
            d_limit: f32::MAX,
            output_limit: f32::MAX,

            integral: 0.0,
            prev_input: None,

            prev_output: PIDOutput::default(),

            setpoint: 0.0,
        }
    }
}

/// reset, step
impl PID {
    pub fn reset_integral(&mut self) {
        self.integral = 0.0;
        // self.prev_input = None;
        // self.prev_output = None;

        // self.p_hist.clear();
        // self.i_hist.clear();
        // self.d_hist.clear();
        // self.output_hist.clear();
    }

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

        // Mitigate output jumps when ki(t) != ki(t-1).
        // While it's standard to use an error_integral that's a running sum of
        // just the error (no ki), because we support ki changing dynamically,
        // we store the entire term so that we don't need to remember previous
        // ki values.
        self.integral = self.integral + error * self.ki;

        // Mitigate integral windup: Don't want to keep building up error
        // beyond what i_limit will allow.
        self.integral = Self::apply_limit(self.i_limit, self.integral);

        // Mitigate derivative kick: Use the derivative of the measurement
        // rather than the derivative of the error.
        let d_unbounded = -match self.prev_input.as_ref() {
            Some(prev_measurement) => input - *prev_measurement,
            None => 0.0,
        } * self.kd;
        self.prev_input = Some(input);
        let d = Self::apply_limit(self.d_limit, d_unbounded);

        let output = p + self.integral + d;
        let output = Self::apply_limit(self.output_limit, output);

        let out = PIDOutput {
            p,
            i: self.integral,
            d,
            output,
        };

        self.prev_output = out;

        out
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

        let mut d = -match self.prev_input.as_ref() {
            Some(prev_measurement) => input - *prev_measurement,
            None => 0.0,
        } * self.kd;
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
            Ki        => self.ki = val,
            Kd        => self.kd = val,
            KpLimit     => self.p_limit = val,
            KiLimit     => self.i_limit = val,
            KdLimit     => self.d_limit = val,
            OutputLimit => self.output_limit = val,
        }
    }
}

#[derive(Debug, Clone, Copy)]
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
