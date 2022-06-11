use defmt::{println as rprintln, Format};

use crate::misc::Axis;

#[derive(Default, Clone, Copy, Format)]
pub struct ControlRates {
    pub roll:     ControlRateAxis,
    pub pitch:    ControlRateAxis,
    pub yaw:      ControlRateAxis,
    pub throttle: ControlRateThrottle,
}

impl ControlRates {
    pub fn get_axis(&self, axis: Axis) -> Option<&ControlRateAxis> {
        match axis {
            Axis::Roll => Some(&self.roll),
            Axis::Pitch => Some(&self.pitch),
            Axis::Yaw => Some(&self.yaw),
            Axis::Throttle => None,
        }
    }
}

#[derive(Clone, Copy, Format)]
pub struct ControlRateAxis {
    pub rc_rate:    f32,
    pub super_rate: f32,
    pub rc_expo:    f32,
}

impl Default for ControlRateAxis {
    fn default() -> Self {
        Self {
            rc_rate:    1.0,
            super_rate: 0.0,
            rc_expo:    0.0,
        }
    }
}

impl ControlRateAxis {
    pub fn apply(&self, input: f32) -> f32 {
        use nalgebra::ComplexField;
        /// BF rcCommand factor
        let q = (input.powi(4) * self.rc_expo) + input * (1.0 - self.rc_expo);
        /// BF expo factor
        let r = 200.0 * q * self.rc_rate;
        /// BF super factor
        let p = 1.0 / (1.0 - (input * self.super_rate));
        r * p
    }
}

#[derive(Clone, Copy, Format)]
pub struct ControlRateThrottle {
    pub mid:  f32,
    pub expo: f32,
}

impl Default for ControlRateThrottle {
    fn default() -> Self {
        Self {
            mid:  0.5,
            expo: 0.0,
        }
    }
}

impl ControlRateThrottle {
    pub fn apply(&self, input: f32) -> f32 {
        input
    }
}
