use stm32f4::stm32f401::TIM4;
use stm32f4xx_hal::{
    gpio::{Pin, PB6, PB7, PB8, PB9},
    prelude::*,
    rcc::Clocks,
    timer::PwmChannel,
};

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

/// new, consts
impl MotorsPWM {
    // const MAX_DUTY_CYCLE: u16 = 56680;

    const MOTOR_MAX_PWM: f32 = 1900.0;

    // fn tim4_init(tim4: TIM4)

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

        let pwm = tim4.pwm_hz(channels, 494.Hz(), &clocks);

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
    pub fn enable_motor(&mut self, motor: MotorSelect) {
        self._enable_motor(motor, true);
    }
    pub fn disable_motor(&mut self, motor: MotorSelect) {
        self._enable_motor(motor, false);
    }

    pub fn _enable_motor(&mut self, motor: MotorSelect, enable: bool) {
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

    pub fn set_motor(&mut self, motor: MotorSelect, pwm: f32) {
        let pwm = pwm.clamp(0.0, 1.0);
        match motor {
            MotorSelect::Motor1 => {
                unimplemented!()
            }
            MotorSelect::Motor2 => {
                unimplemented!()
            }
            MotorSelect::Motor3 => {
                self.pin3.set_duty((pwm * Self::MOTOR_MAX_PWM) as u16);
            }
            MotorSelect::Motor4 => {
                unimplemented!()
            }
        }
    }
}
