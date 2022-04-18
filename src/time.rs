use stm32f4::stm32f401::{self, RCC, TIM2};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::Clocks;

pub struct MonoTimer<T, const FREQ: u32>(T);

impl<const FREQ: u32> MonoTimer<TIM2, FREQ> {
    pub fn new(timer: TIM2, clocks: &Clocks) -> Self {
        // rcc.apb1enr.modify(|r, w| w.tim2en().set_bit());
        // rcc.apb1rstr.modify(|r, w| w.tim2rst().set_bit());
        // rcc.apb1rstr.modify(|r, w| w.tim2rst().clear_bit());

        let pclk_mul = if clocks.ppre1() == 1 { 1 } else { 2 };
        let prescaler = clocks.pclk1().raw() * pclk_mul / FREQ - 1;

        timer.psc.write(|w| w.psc().bits(prescaler as u16));
        timer.arr.write(|w| w.arr().bits(u32::MAX));
        timer.egr.write(|w| w.ug().set_bit());
        timer.sr.modify(|r, w| w.uif().clear_bit());
        timer.cr1.modify(|r, w| w.cen().set_bit().udis().set_bit());

        Self(timer)
    }
}

impl<const FREQ: u32> rtic::Monotonic for MonoTimer<TIM2, FREQ> {
    type Instant = fugit::TimerInstantU32<FREQ>;
    type Duration = fugit::TimerDurationU32<FREQ>;

    unsafe fn reset(&mut self) {
        self.0.dier.modify(|_, w| w.cc1ie().set_bit());
    }

    #[inline(always)]
    fn now(&mut self) -> Self::Instant {
        Self::Instant::from_ticks(self.0.cnt.read().cnt().bits())
    }

    fn set_compare(&mut self, instant: Self::Instant) {
        self.0
            .ccr1
            .write(|w| w.ccr().bits(instant.duration_since_epoch().ticks()));
    }

    fn clear_compare_flag(&mut self) {
        self.0.sr.modify(|_, w| w.cc1if().clear_bit());
    }

    #[inline(always)]
    fn zero() -> Self::Instant {
        Self::Instant::from_ticks(0)
    }
}
