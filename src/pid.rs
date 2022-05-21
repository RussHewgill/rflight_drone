// use cortex_m_semihosting::hprintln;

#[derive(Debug, Clone, Copy)]
pub struct PID {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,

    pub p_limit: (f32, f32),
    pub i_limit: (f32, f32),
    pub d_limit: (f32, f32),

    pub output_limit: (f32, f32),

    pub integral:   f32,
    pub prev_input: Option<f32>,

    pub setpoint: f32,
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

            p_limit: (f32::MIN, f32::MAX),
            i_limit: (f32::MIN, f32::MAX),
            d_limit: (f32::MIN, f32::MAX),
            output_limit: (f32::MIN, f32::MAX),

            integral: 0.0,
            prev_input: None,

            setpoint: 0.0,
        }
    }
}

impl PID {
    pub fn reset(&mut self) {
        self.integral = 0.0;
        // self.prev_input = None;
        // self.prev_output = None;

        // self.p_hist.clear();
        // self.i_hist.clear();
        // self.d_hist.clear();
        // self.output_hist.clear();
    }

    pub fn step(&mut self, input: f32) -> f32 {
        let (output, _, _, _) = self._step(input);
        output
    }

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
