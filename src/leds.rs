use stm32f4xx_hal::gpio::{Output, Pin};

pub struct LEDs {
    led1_pin: Pin<'B', 5, Output>,
    led2_pin: Pin<'B', 4, Output>,
}

impl LEDs {
    pub fn new(led1_pin: Pin<'B', 5, Output>, led2_pin: Pin<'B', 4, Output>) -> Self {
        Self { led1_pin, led2_pin }
    }
}
