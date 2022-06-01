use fugit::HertzU32;
use stm32f4::stm32f401::TIM4;
use stm32f4xx_hal::{
    gpio::{Pin, PB6, PB7, PB8, PB9},
    prelude::*,
    rcc::Clocks,
    timer::PwmChannel,
};

use defmt::{debug, println as rprintln};

/// 3 - 2
/// 4 - 1
/// M1: CW,  back right
/// M2: CCW, front right
/// M3: CW,  front left
/// M4: CCW, back left
/// Max duty: 56680
pub struct MotorsPWM {
    armed: bool,
    pin1:  PwmChannel<TIM4, 0>,
    pin2:  PwmChannel<TIM4, 1>,
    pin3:  PwmChannel<TIM4, 2>,
    pin4:  PwmChannel<TIM4, 3>,
}

#[derive(Debug, Clone, Copy)]
pub enum MotorSelect {
    Motor1,
    Motor2,
    Motor3,
    Motor4,
}

/// consts
impl MotorsPWM {
    // pub const MAX_DUTY_CYCLE: u16 = 56680;

    pub const MOTOR_MAX_PWM: f32 = 56680.0;
    pub const MOTOR_MAX_PWM_I: u16 = 56680;

    // pub const MOTOR_MAX_PWM: f32 = 1900.0;
    // pub const MOTOR_MAX_PWM_I: u16 = 1900;

    // /// from ST firmware, probably not very accurate
    // pub const MOTOR_MIN_THROTTLE: f32 = 200.0;

    // pub const MOTOR_MAX_THROTTLE: Option<f32> = None;
    pub const MOTOR_MAX_THROTTLE: Option<f32> = Some(0.02);

    /// from ST firmware
    pub const FREQ: HertzU32 = fugit::HertzU32::Hz(494);
    // pub const FREQ: HertzU32 = fugit::HertzU32::Hz(750);
    // pub const FREQ: HertzU32 = fugit::HertzU32::Hz(250);
}

/// new
impl MotorsPWM {
    pub fn new(
        tim4: TIM4,
        pin1: PB6,
        pin2: PB7,
        pin3: PB8,
        pin4: PB9,
        clocks: &Clocks,
    ) -> Self {
        let channels = (
            pin1.into_alternate::<2>(),
            pin2.into_alternate::<2>(),
            pin3.into_alternate::<2>(),
            pin4.into_alternate::<2>(),
        );

        // let time = fugit::TimerDurationU32
        // let time: fugit::TimerDurationU32<494> = 2.millis();
        // let time: fugit::TimerDurationU32<500> = 4.millis();

        // let time: fugit::TimerDurationU32<494> =
        //     fugit::TimerDurationU32::<494>::from_ticks(1999);
        // // rprintln!("time = {:?}", time);

        let pwm = tim4.pwm_hz(channels, Self::FREQ, &clocks);
        // let pwm = tim4.pwm(channels, time, &clocks);

        let (mut pin1, mut pin2, mut pin3, mut pin4) = pwm.split();

        pin1.disable();
        pin2.disable();
        pin3.disable();
        pin4.disable();

        Self {
            armed: false,
            // tim4,
            pin1,
            pin2,
            pin3,
            pin4,
        }
    }
}

/// enable, disable, set pwm
impl MotorsPWM {
    pub fn set_armed(&mut self, armed: bool) {
        rprintln!("setting motors armed = {:?}", armed);

        // rprintln!("DEBUG: motors bypassed");

        self.armed = armed;
        if armed {
            self.enable_all();
        } else {
            self.disable_all();
        }

        //
    }

    pub fn enable_all(&mut self) {
        debug!("enabling all motors");
        self.enable_motor(MotorSelect::Motor1);
        self.enable_motor(MotorSelect::Motor2);
        self.enable_motor(MotorSelect::Motor3);
        self.enable_motor(MotorSelect::Motor4);
    }
    pub fn disable_all(&mut self) {
        debug!("enabling all motors");
        self.disable_motor(MotorSelect::Motor1);
        self.disable_motor(MotorSelect::Motor2);
        self.disable_motor(MotorSelect::Motor3);
        self.disable_motor(MotorSelect::Motor4);
    }

    pub fn enable_motor(&mut self, motor: MotorSelect) {
        self._enable_motor(motor, true);
    }
    pub fn disable_motor(&mut self, motor: MotorSelect) {
        self._enable_motor(motor, false);
    }

    fn _enable_motor(&mut self, motor: MotorSelect, enable: bool) {
        if !self.armed {
            rprintln!("tried to enable motors when not armed");
            return;
        }
        let pin = match motor {
            MotorSelect::Motor1 => {
                if enable {
                    self.pin1.enable();
                } else {
                    self.pin1.disable();
                }
            }
            MotorSelect::Motor2 => {
                if enable {
                    self.pin2.enable();
                } else {
                    self.pin2.disable();
                }
            }
            MotorSelect::Motor3 => {
                if enable {
                    self.pin3.enable();
                } else {
                    self.pin3.disable();
                }
            }
            MotorSelect::Motor4 => {
                if enable {
                    self.pin4.enable();
                } else {
                    self.pin4.disable();
                }
            }
        };
    }

    pub fn set_all_f32(&mut self, pwm: f32) {
        self.set_motor_f32(MotorSelect::Motor1, pwm);
        self.set_motor_f32(MotorSelect::Motor2, pwm);
        self.set_motor_f32(MotorSelect::Motor3, pwm);
        self.set_motor_f32(MotorSelect::Motor4, pwm);
    }

    pub fn set_all_u16(&mut self, pwm: u16) {
        self.set_motor_u16(MotorSelect::Motor1, pwm);
        self.set_motor_u16(MotorSelect::Motor2, pwm);
        self.set_motor_u16(MotorSelect::Motor3, pwm);
        self.set_motor_u16(MotorSelect::Motor4, pwm);
    }

    pub fn set_motor_f32(&mut self, motor: MotorSelect, pwm: f32) {
        if !self.armed {
            return;
        }
        let pwm = if let Some(max) = Self::MOTOR_MAX_THROTTLE {
            pwm.clamp(0.0, max)
        } else {
            pwm.clamp(0.0, 1.0)
        };
        match motor {
            MotorSelect::Motor1 => {
                self.pin1.set_duty((pwm * Self::MOTOR_MAX_PWM) as u16);
            }
            MotorSelect::Motor2 => {
                self.pin2.set_duty((pwm * Self::MOTOR_MAX_PWM) as u16);
            }
            MotorSelect::Motor3 => {
                self.pin3.set_duty((pwm * Self::MOTOR_MAX_PWM) as u16);
            }
            MotorSelect::Motor4 => {
                self.pin4.set_duty((pwm * Self::MOTOR_MAX_PWM) as u16);
            }
        }
    }

    // #[cfg(feature = "nope")]
    pub fn set_motor_u16(&mut self, motor: MotorSelect, pwm: u16) {
        // let pwm = pwm.clamp(0, Self::MOTOR_MAX_PWM_I);
        let pwm = pwm.min(Self::MOTOR_MAX_PWM_I);
        match motor {
            MotorSelect::Motor1 => {
                self.pin1.set_duty(pwm);
            }
            MotorSelect::Motor2 => {
                self.pin2.set_duty(pwm);
            }
            MotorSelect::Motor3 => {
                self.pin3.set_duty(pwm);
            }
            MotorSelect::Motor4 => {
                self.pin4.set_duty(pwm);
            }
        }
    }
}
