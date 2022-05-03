use stm32f4xx_hal::prelude::*;

use crate::{sensors::V3, uart::*, uprintln};

pub fn print_v3(uart: &mut UART, vec: V3) {
    // uprintln!(uart, "[{:.3},{:.3},{:.3}]", vec.x, vec.y, vec.z);
    uprintln!(uart, "[{:.2},{:.2},{:.2}]", vec.x, vec.y, vec.z);
}

pub fn uart_clear_screen(uart: &mut UART) {
    uprintln!(uart, "\x1B\x5B\x32\x4A");
}

// #[macro_export]
// macro_rules! bt_command_timeout {
// 	  ($command:expr, ) => {
//         writeln!($uart.tx, "").unwrap();
// 	  };
// 	  ($uart:expr, $fmt:expr) => {
//         $uart.write_str(concat!($fmt, "\r\n")).unwrap();
//         // core::fmt::Write::write_str(&mut $uart, concat!($fmt, "\r\n")).unwrap();
// 	  };
// 	  ($uart:expr, $fmt:expr, $($arg:tt)*) => {
//         $uart.write_fmt(format_args!(concat!($fmt, "\r\n"), $($arg)*)).unwrap();
//         // core::fmt::Write::write_fmt(&mut $uart, format_args!(concat!($fmt, "\r\n"), $($arg)*)).unwrap();
// 	  };
// }
