use stm32f4::stm32f401::USART1;
use stm32f4xx_hal::{
    gpio::{Alternate, Pin, PA10, PA9},
    prelude::*,
    rcc::Clocks,
    serial::{Rx, Tx},
};

pub struct UART {
    tx: Tx<USART1, u8>,
    rx: Rx<USART1, u8>,
}

impl UART {
    pub fn new(
        usart1: USART1,
        // tx_pin: Pin<'A', 9, Alternate<7>>,
        // rx_pin: Pin<'A', 10, Alternate<7>>,
        tx_pin: PA9,
        rx_pin: PA10,
        clocks: &Clocks,
    ) -> Self {
        let tx_pin = tx_pin.into_alternate();
        let rx_pin = rx_pin.into_alternate();

        let mut serial = usart1
            .serial((tx_pin, rx_pin), 115200.bps(), &clocks)
            .unwrap()
            .with_u8_data();
        let (tx, rx) = serial.split();

        Self { tx, rx }
    }
}
