use core::ptr;
use embedded_hal::spi::*;
use stm32f4::stm32f401::{Peripherals, GPIOB, RCC, SPI2};
use stm32f4xx_hal::{
    gpio::{Alternate, Pin, PinExt, PA8, PB13, PB15},
    nb,
    prelude::*,
    rcc::Clocks,
    spi::{Error as SpiError, NoMiso},
    time::*,
};

pub struct Spi3 {
    spi: SPI2,
    sck: Pin<'B', 13, Alternate<5>>,
    mosi: Pin<'B', 15, Alternate<5>>,
    miso: NoMiso,
    output_mode: bool,
}

/// new
impl Spi3 {
    pub fn new(
        // dp: &Peripherals,
        rcc: &RCC,
        gpiob: GPIOB,
        spi: SPI2,
        // sck: PB13,
        // mosi: PB15,
        mode: Mode,
        freq: Hertz,
        // clocks: &Clocks,
    ) -> Self {
        /// Enable SPI2 clock
        rcc.apb1enr.write(|w| w.spi2en().set_bit());

        /// Enable GPIOB
        rcc.ahb1enr.write(|w| w.gpioben().set_bit());

        /// set PB13, PB15 to output
        gpiob
            .moder
            .modify(|r, w| w.moder13().output().moder15().output());

        /// set PB13, PB15 to high speed
        gpiob
            .ospeedr
            .modify(|r, w| w.ospeedr13().high_speed().ospeedr15().high_speed());

        /// set PB13, PB15 to No Pull-up, No Pull-down (No PUPD)
        gpiob
            .pupdr
            .modify(|r, w| w.pupdr13().floating().pupdr15().floating());

        let gpiob = gpiob.split();

        /// XXX: should be external, borrow rules
        let mut cs_magno = gpiob.pb12.into_push_pull_output();
        cs_magno.set_high();

        let sck = gpiob.pb13;
        let mosi = gpiob.pb15;

        let sck = sck.into_push_pull_output().into_alternate::<5>();
        let mosi = mosi.into_push_pull_output().into_alternate::<5>();

        spi.cr1.modify(|_, w| {
            w.cpha() // clock phase
                .bit(mode.phase == Phase::CaptureOnSecondTransition)
                .cpol() // clock polarity
                .bit(mode.polarity == Polarity::IdleHigh)
                .bidimode() // bidirectional half duplex mode
                .set_bit()
                .bidioe() // bidi output mode
                .set_bit()
                .br() // baud rate = 1/16 f_PCLK
                .div16()
                .mstr() // master mode enabled
                .set_bit()
                .ssm() // software slave management
                .set_bit()
                // .ssi()
                // .set_bit()
                .dff() // 8 bit data frame format
                .eight_bit()
                .lsbfirst() // MSB first
                .clear_bit()
                .crcen() // hardware CRC disabled (?)
                .clear_bit()
        });

        spi.cr2.modify(|_, w| {
            w.ssoe()
                .set_bit() // SS output enabled
                .frf()
                .clear_bit() // Motorola frame format (not TI)
        });

        let mut out = Self {
            spi,
            sck,
            mosi,
            miso: NoMiso {},
            output_mode: true,
        };

        out.enable(true);

        out
    }
}

impl Spi3 {
    // pub fn send(&mut self, byte: u8) {
    //     /// wait until Tx buffer is empty
    //     while !self.is_txe() {}
    //     self.send_u8(byte);
    //     while !self.is_txe() {}
    //     while self.is_bsy() {}
    // }

    /// The sequence begins when data are written into the SPI_DR register (Tx buffer)
    /// While (BIDIMODE=1 and BIDIOE=1)
    pub fn send(&mut self, bytes: &[u8]) -> nb::Result<(), SpiError> {
        // if !self.output_mode {
        //     // self.set_bidi_output();
        // }

        // self.set_bidi_output();
        // self.enable(true);
        for b in bytes {
            nb::block!(self.nb_send(*b))?;
        }
        // self.enable(false);
        Ok(())
    }

    pub fn read(&mut self, byte: &mut u8) -> nb::Result<(), SpiError> {
        self.enable(false);
        self.set_bidi_input();
        cortex_m::interrupt::disable();
        self.enable(true);

        {
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
            cortex_m::asm::dsb();
        }

        self.enable(false);
        unsafe {
            cortex_m::interrupt::enable();
        }
        self.set_bidi_output();
        self.enable(true);

        *byte = nb::block!(self.nb_read())?;

        Ok(())
    }

    /// buf.len() > 1
    /// The sequence begins as soon as SPE=1 and BIDIOE=0
    pub fn read_mult(&mut self, buf: &mut [u8]) -> nb::Result<(), SpiError> {
        self.enable(false);
        self.set_bidi_input();

        cortex_m::interrupt::disable();

        let len = buf.len() - 2;
        self.enable(true);
        for b in &mut buf[..len] {
            *b = nb::block!(self.nb_read())?;
        }

        /// In master RX mode the clock is automaticaly generated on the SPI enable.
        /// So to guarantee the clock generation for only one data, the clock must be
        /// disabled after the first bit and before the latest bit of the last Byte received
        /// __DSB instruction are inserted to garantee that clock is Disabled in the right timeframe
        cortex_m::asm::dsb();
        cortex_m::asm::dsb();
        self.enable(false);

        unsafe {
            cortex_m::interrupt::enable();
        }

        self.set_bidi_output();
        self.enable(true);

        Ok(())
    }

    // pub fn send_byte(&mut self, byte: u8) -> nb::Result<(), SpiError> {
    //     self.set_bidi_output();
    //     nb::block!(self.nb_send(byte))?;
    //     Ok(())
    // }

    // pub fn read_byte(&mut self, buf: &mut u8) -> nb::Result<u8, SpiError> {
    //     self.set_bidi_input();
    //     let x = nb::block!(self.nb_read())?;
    //     Ok(x)
    // }
}

/// nb
impl Spi3 {
    #[inline(always)]
    pub fn nb_read(&mut self) -> nb::Result<u8, SpiError> {
        let sr = self.spi.sr.read();

        Err(if sr.ovr().bit_is_set() {
            SpiError::Overrun.into()
        } else if sr.modf().bit_is_set() {
            SpiError::ModeFault.into()
        } else if sr.crcerr().bit_is_set() {
            SpiError::Crc.into()
        } else if sr.rxne().bit_is_set() {
            return Ok(self.read_u8());
        } else {
            nb::Error::WouldBlock
        })
    }

    #[inline(always)]
    pub fn nb_send(&mut self, byte: u8) -> nb::Result<(), SpiError> {
        let sr = self.spi.sr.read();

        Err(if sr.ovr().bit_is_set() {
            // Read from the DR to clear the OVR bit
            let _ = self.spi.dr.read();
            SpiError::Overrun.into()
        } else if sr.modf().bit_is_set() {
            // Write to CR1 to clear MODF
            self.spi.cr1.modify(|_r, w| w);
            SpiError::ModeFault.into()
        } else if sr.crcerr().bit_is_set() {
            // Clear the CRCERR bit
            self.spi.sr.modify(|_r, w| {
                w.crcerr().clear_bit();
                w
            });
            SpiError::Crc.into()
        } else if sr.txe().bit_is_set() {
            self.send_u8(byte);
            return Ok(());
        } else {
            nb::Error::WouldBlock
        })
    }
}

/// unsafe read/write
impl Spi3 {
    #[inline(always)]
    fn read_u8(&mut self) -> u8 {
        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows reading a half-word)
        unsafe { ptr::read_volatile(&self.spi.dr as *const _ as *const u8) }
    }

    #[inline(always)]
    fn send_u8(&mut self, byte: u8) {
        // NOTE(write_volatile) see note above
        unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
    }
}

/// enable
impl Spi3 {
    /// Enable/disable spi
    pub fn enable(&mut self, enable: bool) {
        self.spi.cr1.modify(|_, w| {
            // spe: enable the SPI bus
            w.spe().bit(enable)
        });

        // if enable {
        //     self.spi.cr1.modify(|_, w| {
        //         // spe: enable the SPI bus
        //         w.spe().bit(true)
        //     });
        // } else {
        //     while !self.is_txe() {
        //         cortex_m::asm::nop();
        //     }
        // }
    }

    /// receive
    pub fn set_bidi_input(&mut self) {
        self.spi.cr1.modify(|_, w| w.bidioe().clear_bit());
        self.output_mode = false;
    }

    /// transmit
    pub fn set_bidi_output(&mut self) {
        self.spi.cr1.modify(|_, w| w.bidioe().set_bit());
        self.output_mode = true;
    }
}

/// check flags
impl Spi3 {
    pub fn is_bsy(&self) -> bool {
        self.spi.sr.read().bsy().bit_is_set()
    }

    pub fn is_txe(&self) -> bool {
        self.spi.sr.read().txe().bit_is_set()
    }

    pub fn is_rxne(&self) -> bool {
        self.spi.sr.read().rxne().bit_is_set()
    }

    pub fn is_modf(&self) -> bool {
        self.spi.sr.read().modf().bit_is_set()
    }

    pub fn is_ovr(&self) -> bool {
        self.spi.sr.read().ovr().bit_is_set()
    }
}
