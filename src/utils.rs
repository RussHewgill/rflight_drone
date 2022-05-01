use stm32f4xx_hal::prelude::*;

use crate::{sensors::V3, uart::*, uprintln};

pub fn print_v3(uart: &mut UART, vec: V3) {
    // uprintln!(uart, "[{:.3},{:.3},{:.3}]", vec.x, vec.y, vec.z);
    uprintln!(uart, "[{:.2},{:.2},{:.2}]", vec.x, vec.y, vec.z);
}

pub fn uart_clear_screen(uart: &mut UART) {
    uprintln!(uart, "\x1B\x5B\x32\x4A");
}
