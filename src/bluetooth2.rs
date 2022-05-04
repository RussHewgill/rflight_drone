use arrayvec::ArrayVec;

use byteorder::{ByteOrder, LittleEndian};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use stm32f4::stm32f401::{EXTI, TIM2};
use stm32f4xx_hal::{
    gpio::{ExtiPin, PA4},
    nb,
    prelude::*,
    timer::CounterMs,
};

use crate::{
    bluetooth::{AccessByte, BTError, BTServices},
    bt_control::BTState,
    spi::{Spi4, SpiError},
    uart::UART,
};

pub struct BluetoothSpi2<CS, Reset, Input> {
    spi:   Spi4,
    cs:    CS,
    reset: Reset,
    input: Input,

    buffer: ArrayVec<u8, 512>,

    pub delay: CounterMs<TIM2>,

    pub state:    BTState,
    pub services: BTServices,
}

/// new
impl<CS, Reset, Input> BluetoothSpi2<CS, Reset, Input> {
    pub fn new(
        spi: Spi4,
        cs: CS,
        reset: Reset,
        input: Input,
        delay: CounterMs<TIM2>,
    ) -> Self {
        Self {
            spi,
            cs,
            reset,
            input,
            delay,
            buffer: ArrayVec::new(),
            state: BTState::Disconnected,
            services: BTServices::default(),
        }
    }
}

/// pause/resume/check/clear interrupt
impl<CS, Reset> BluetoothSpi2<CS, Reset, PA4> {
    pub fn unpend(&mut self) {
        cortex_m::peripheral::NVIC::unpend(self.input.interrupt());
    }

    pub fn check_interrupt(&mut self) -> bool {
        self.input.check_interrupt()
    }

    pub fn clear_interrupt(&mut self) {
        // exti.pr.modify(|r, w| w.pr4().set_bit());
        self.input.clear_interrupt_pending_bit();
    }

    pub fn pause_interrupt(&mut self, exti: &mut EXTI) {
        self.input.disable_interrupt(exti);
    }

    pub fn unpause_interrupt(&mut self, exti: &mut EXTI) {
        self.input.enable_interrupt(exti);
    }
}

/// reset, wait
impl<CS, Reset, Input, GpioError> BluetoothSpi2<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
{
    /// ms must be greater than 1
    pub fn wait_ms(&mut self, ms: fugit::MillisDurationU32) {
        // if ms == 1.millis() {
        //     return;
        // }
        use stm32f4xx_hal::timer::Event;
        self.delay.clear_interrupt(Event::Update);
        self.delay.start(ms).unwrap();
        while !self.delay.get_interrupt().contains(Event::Update) {
            cortex_m::asm::nop();
        }
        self.delay.cancel().unwrap();
        self.delay.clear_interrupt(Event::Update);
    }

    pub fn reset(&mut self) -> nb::Result<(), GpioError> {
        self.reset.set_low().map_err(nb::Error::Other)?;
        self.wait_ms(10.millis());
        self.reset.set_high().map_err(nb::Error::Other)?;
        self.wait_ms(10.millis());
        Ok(())
    }

    pub fn data_ready(&self) -> Result<bool, Input::Error> {
        self.input.is_high()
    }
}

/// block until ready
impl<CS, Reset, Input, GpioError> BluetoothSpi2<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
{
    pub fn block_until_ready(
        &mut self,
        access_byte: AccessByte,
        mut uart: Option<&mut UART>,
    ) -> nb::Result<(u16, u16), BTError<SpiError, GpioError>> {
        // let mut x = 0;
        loop {
            // x += 1;
            let mut header = [access_byte as u8, 0x00, 0x00, 0x00, 0x00];

            // self.spi
            //     .transfer(&mut header)
            //     .map_err(BTError::Spi)
            //     .map_err(nb::Error::Other)?;

            match parse_spi_header(&header) {
                Ok(lens) => {
                    // if let Some(ref mut uart) = uart {
                    //     uprintln!(uart, "ready in {:?} loops", x);
                    // }
                    return Ok(lens);
                }
                Err(nb::Error::WouldBlock) => {
                    self.cs
                        .set_high()
                        .map_err(BTError::Gpio)
                        .map_err(nb::Error::Other)?;
                    // self.wait_ms(2.mill)
                    cortex_m::asm::nop();
                    // cortex_m::asm::nop();
                    self.cs
                        .set_low()
                        .map_err(BTError::Gpio)
                        .map_err(nb::Error::Other)?;
                }
                Err(e) => return Err(e),
            }
        }
    }

    fn block_until_ready_for(
        &mut self,
        access: AccessByte,
        uart: Option<&mut UART>,
    ) -> nb::Result<u16, BTError<SpiError, GpioError>> {
        // let (write_len, read_len) = self.block_until_ready(access, uart)?;
        let (write_len, read_len) = self.block_until_ready(access, uart)?;
        Ok(match access {
            AccessByte::Read => read_len,
            AccessByte::Write => write_len,
        })
    }
}

fn parse_spi_header<E>(header: &[u8; 5]) -> Result<(u16, u16), nb::Error<E>> {
    const BNRG_READY: u8 = 0x02;
    if header[0] == BNRG_READY {
        Ok((
            LittleEndian::read_u16(&header[1..]),
            LittleEndian::read_u16(&header[3..]),
        ))
    } else {
        Err(nb::Error::WouldBlock)
    }
}

// impl<CS, Reset, Input> BluetoothSpi2<CS, Reset, Input> {
// }
