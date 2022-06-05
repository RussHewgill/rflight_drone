use stm32f4::stm32f401::ADC1;
use stm32f4xx_hal::{
    adc::config::*,
    adc::Adc,
    adc::*,
    gpio::{Analog, Pin, PB1},
};

pub struct BatteryAdc {
    pub adc:          Adc<ADC1>,
    pub voltage:      Pin<'B', 1, Analog>,
    pub last_reading: f32,
    pub min_voltage:  f32,
}

impl BatteryAdc {
    pub fn new(adc: Adc<ADC1>, voltage: Pin<'B', 1, Analog>) -> Self {
        let mut out = Self {
            adc,
            voltage,
            last_reading: 0.0,
            min_voltage: 3.8,
        };
        out.sample();
        out
    }

    pub fn sample(&mut self) -> f32 {
        const V_REF: f32 = 3.3;
        const R_UP: f32 = 10_000.0;
        const R_DOWN: f32 = 20_000.0;

        let sample = self.adc.convert(&self.voltage, SampleTime::Cycles_3);

        let v_bat: f32 = (sample as f32 * V_REF) / 2u32.pow(12) as f32;
        let v_bat: f32 = v_bat * ((R_UP + R_DOWN) / R_DOWN);

        self.last_reading = v_bat;

        v_bat
    }
}
