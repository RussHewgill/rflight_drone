use stm32f4::stm32f401::TIM4;
use stm32f4xx_hal::{
    gpio::{Pin, PB6, PB7, PB8, PB9},
    prelude::*,
    rcc::Clocks,
};

/// 3 - 2
/// 4 - 1
/// M1: CW,  back left
/// M2: CCW, front left
/// M3: CW,  front right
/// M4: CCW, back right
pub struct MotorsPWM {
    armed: bool,
    // tim4: TIM4,
    // pin1: PB6,
    // pin2: PB7,
    // pin3: PB8,
    // pin4: PB9,
}

impl MotorsPWM {
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

        Self {
            armed: false,
            // tim4,
            // pin1,
            // pin2,
            // pin3,
            // pin4,
        }
    }
}
