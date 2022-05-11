use stm32f4xx_hal::prelude::*;

use defmt::println as rprintln;

use crate::sensors::V3;

// pub fn print_v3(vec: V3, n: usize) {
//     match n {
//         1 => rprintln!("[{:>4.1},{:>4.1},{:>4.1}]", vec.x, vec.y, vec.z),
//         2 => rprintln!("[{:>5.2},{:>5.2},{:>5.2}]", vec.x, vec.y, vec.z),
//         3 => rprintln!("[{:>6.3},{:>6.3},{:>6.3}]", vec.x, vec.y, vec.z),
//         _ => unimplemented!(),
//     }
// }

pub fn print_v3(label: &str, vec: V3, n: i32) {
    rprintln!(
        "{} {=f32:08}, {=f32:08}, {=f32:08}",
        label,
        round_to(vec.x, n),
        round_to(vec.y, n),
        round_to(vec.z, n),
    );
}

pub fn round_to(f: f32, places: i32) -> f32 {
    use nalgebra::{ComplexField, RealField};
    let x: f32 = 10.0f32.powi(places);
    (f * x).round() / x
}

pub fn r(x: f32) -> f32 {
    use nalgebra::ComplexField;
    (x * 100.0).round() / 100.0
}

pub fn r2(x: f32) -> f32 {
    use nalgebra::ComplexField;
    (x * 10_000.0).round() / 10.0
}

// #[derive(Debug,Clone,Copy)]
// #[repr(transparent)]
// pub struct V3F(pub V3);

// impl defmt::Format for V3F {
//     fn format(&self, fmt: defmt::Formatter) {
//     }
// }

// pub fn uart_clear_screen(uart: &mut UART) {
//     uprintln!(uart, "\x1B\x5B\x32\x4A");
// }

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
