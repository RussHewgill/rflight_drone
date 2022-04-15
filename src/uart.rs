use stm32f4::stm32f401::USART1;
use stm32f4xx_hal::{
    gpio::{Alternate, Pin, PA10, PA9},
    prelude::*,
    rcc::Clocks,
    serial::{Rx, Tx},
};

pub use core::fmt::Write as CoreWrite;

pub struct UART {
    pub tx: Tx<USART1, u8>,
    pub rx: Rx<USART1, u8>,
}

/// new
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

// impl UART {
//     pub fn send(&mut self, s: &str) {
//         use core::fmt::Write;
//         // writeln!(self.tx, s).unwrap();
//         self.tx.write_str(s).unwrap();
//     }
// }

impl core::fmt::Write for UART {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx.write_str(s)
    }
}

#[macro_export]
macro_rules! uprint {
	  ($uart:expr, $fmt:expr) => {
        // core::fmt::Write::write_str(&mut $uart, concat!($fmt, "\r\n")).unwrap();
        $uart.write_str($fmt).unwrap();
	  };
	  ($uart:expr, $fmt:expr, $($arg:tt)*) => {
        // writeln!($uart.tx, ).unwrap();
        // core::fmt::Write::write_str(&mut $uart, format_args!(concat!($fmt, "\n"), $($arg)*)).unwrap();
        // core::fmt::Write::write_fmt(&mut $uart, format_args!(concat!($fmt, "\r\n"), $($arg)*)).unwrap();
        $uart.write_fmt(format_args!($fmt, $($arg)*)).unwrap();
	  };
}

#[macro_export]
macro_rules! uprintln {
	  ($uart:expr) => {
        writeln!($uart.tx, "").unwrap();
	  };
	  ($uart:expr, $fmt:expr) => {
        // core::fmt::Write::write_str(&mut $uart, concat!($fmt, "\r\n")).unwrap();
        $uart.write_str(concat!($fmt, "\r\n")).unwrap();
	  };
	  ($uart:expr, $fmt:expr, $($arg:tt)*) => {
        // writeln!($uart.tx, ).unwrap();
        // core::fmt::Write::write_str(&mut $uart, format_args!(concat!($fmt, "\n"), $($arg)*)).unwrap();
        // core::fmt::Write::write_fmt(&mut $uart, format_args!(concat!($fmt, "\r\n"), $($arg)*)).unwrap();
        $uart.write_fmt(format_args!(concat!($fmt, "\r\n"), $($arg)*)).unwrap();
	  };
}
