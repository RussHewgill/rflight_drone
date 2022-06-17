use stm32f4::stm32f401::{ADC1, EXTI};
use stm32f4xx_hal::{
    adc::config::*,
    adc::Adc,
    adc::*,
    gpio::{Analog, Pin, PB1},
};

use crate::{bt_control::BTController, consts::MAIN_LOOP_FREQ};

use defmt::{println as rprintln, Format};

pub struct BatteryAdc {
    pub adc:              Adc<ADC1>,
    pub voltage:          Pin<'B', 1, Analog>,
    pub last_reading:     f32,
    pub min_voltage_warn: f32,
    pub min_voltage_halt: f32,
}

impl BatteryAdc {
    pub fn battery_warning(
        &mut self,
        bt: &mut BTController,
        // exti: &mut EXTI,
        warn_counter: &mut u32,
        armed: bool,
        send_bt: bool,
    ) -> bool {
        // const MAX_WARNING: u32 = 1;

        /// will disarm after 0.25 seconds below min voltage
        const MAX_WARNING: u32 = MAIN_LOOP_FREQ.to_Hz() / 4;

        let v = self.sample();

        if send_bt {
            // bt.pause_interrupt(exti);
            bt.log_write_batt(v).unwrap();
            // bt.unpause_interrupt(exti);
        }

        // self.min_voltage_warn = 3.9;

        if !armed {
            return false;
        }

        if v <= self.min_voltage_warn {
            if *warn_counter >= MAX_WARNING {
                return true;
            } else {
                *warn_counter += 1;
                rprintln!("Battery warning: {:?}, v = {:?}", *warn_counter, v);
            }
        } else if v <= self.min_voltage_halt {
            rprintln!("Battery Halt: {:?}", v);
            return true;
        } else {
            if *warn_counter > 0 {
                *warn_counter -= 1;
            }
        }

        let x = core::sync::atomic::AtomicBool::new(false);

        false
    }
}

impl BatteryAdc {
    pub fn new(adc: Adc<ADC1>, voltage: Pin<'B', 1, Analog>) -> Self {
        let mut out = Self {
            adc,
            voltage,
            last_reading: 0.0,
            min_voltage_warn: 3.6,
            min_voltage_halt: 3.5,
        };
        out.sample();
        out
    }

    // pub fn sample_avg(&mut self, n: usize) -> f32 {
    //     let mut x = 0.0;
    //     for _ in 0..n {
    //         x += self.sample();
    //     }
    //     let out = x / n as f32;
    //     self.last_reading = out;
    //     out
    // }

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
