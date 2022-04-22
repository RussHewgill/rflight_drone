pub mod ev_command;
pub mod events;

pub mod command;
pub mod gap;
pub mod gatt;
pub mod opcode;
pub mod rx_buffer;

pub mod hal_bt;

use arrayvec::ArrayVec;
use core::ptr;
use cortex_m::peripheral::{NVIC, SYST};
use cortex_m_semihosting::hprintln;
use embedded_hal as hal;

use hal::digital::v2::{InputPin, OutputPin};

// use hal::{
//     digital::blocking::{InputPin, OutputPin},
//     spi::{
//         self,
//         blocking::{Read, Transfer, TransferInplace, Write},
//     },
// };

use stm32f4::stm32f401::{Peripherals, EXTI, GPIOB, RCC, SPI1, SPI2};
use stm32f4xx_hal::{
    block,
    gpio::{Alternate, Pin, PinExt, PA4, PA8, PB13, PB15},
    nb,
    prelude::*,
    rcc::Clocks,
    spi::{Error as SpiError, NoMiso},
    time::*,
};

use bluetooth_hci::host::uart::{CommandHeader, Hci as HciUart};
use bluetooth_hci::host::Hci;
use bluetooth_hci::{host::HciHeader, Controller, Opcode};

use byteorder::{ByteOrder, LittleEndian};

use crate::{
    bt_control::{service_log::SvLogger, service_sensors::SvSensors, BTState},
    uart::*,
    uprint, uprintln,
};

use self::rx_buffer::Buffer;

/// // SPI Configuration
/// #define BNRG_SPI_MODE               SPI_MODE_MASTER
/// #define BNRG_SPI_DIRECTION          SPI_DIRECTION_2LINES
/// #define BNRG_SPI_DATASIZE           SPI_DATASIZE_8BIT
/// #define BNRG_SPI_CLKPOLARITY        SPI_POLARITY_LOW
/// #define BNRG_SPI_CLKPHASE           SPI_PHASE_1EDGE
/// #define BNRG_SPI_NSS                SPI_NSS_SOFT
/// #define BNRG_SPI_FIRSTBIT           SPI_FIRSTBIT_MSB
/// #define BNRG_SPI_TIMODE             SPI_TIMODE_DISABLED
/// #define BNRG_SPI_CRCPOLYNOMIAL      7
/// #define BNRG_SPI_BAUDRATEPRESCALER  SPI_BAUDRATEPRESCALER_16
/// #define BNRG_SPI_CRCCALCULATION     SPI_CRCCALCULATION_DISABLED

/// // SPI Reset Pin: PB.2
/// #define BNRG_SPI_RESET_PIN          GPIO_PIN_2
/// #define BNRG_SPI_RESET_MODE         GPIO_MODE_OUTPUT_PP
/// #define BNRG_SPI_RESET_PULL         GPIO_PULLUP
/// #define BNRG_SPI_RESET_SPEED        GPIO_SPEED_LOW
/// #define BNRG_SPI_RESET_ALTERNATE    0
/// #define BNRG_SPI_RESET_PORT         GPIOB
/// #define BNRG_SPI_RESET_CLK_ENABLE() __GPIOB_CLK_ENABLE()

/// // SCLK: PA.5
/// #define BNRG_SPI_SCLK_PIN           GPIO_PIN_5
/// #define BNRG_SPI_SCLK_MODE          GPIO_MODE_AF_PP
/// #define BNRG_SPI_SCLK_PULL          GPIO_PULLDOWN
/// #define BNRG_SPI_SCLK_SPEED         GPIO_SPEED_HIGH
/// #define BNRG_SPI_SCLK_ALTERNATE     GPIO_AF5_SPI1
/// #define BNRG_SPI_SCLK_PORT          GPIOA
/// #define BNRG_SPI_SCLK_CLK_ENABLE()  __GPIOA_CLK_ENABLE()

/// // MISO (Master Input Slave Output): PA.6
/// #define BNRG_SPI_MISO_PIN           GPIO_PIN_6
/// #define BNRG_SPI_MISO_MODE          GPIO_MODE_AF_PP
/// #define BNRG_SPI_MISO_PULL          GPIO_NOPULL
/// #define BNRG_SPI_MISO_SPEED         GPIO_SPEED_HIGH
/// #define BNRG_SPI_MISO_ALTERNATE     GPIO_AF5_SPI1
/// #define BNRG_SPI_MISO_PORT          GPIOA
/// #define BNRG_SPI_MISO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()

/// // MOSI (Master Output Slave Input): PA.7
/// #define BNRG_SPI_MOSI_PIN           GPIO_PIN_7
/// #define BNRG_SPI_MOSI_MODE          GPIO_MODE_AF_PP
/// #define BNRG_SPI_MOSI_PULL          GPIO_NOPULL
/// #define BNRG_SPI_MOSI_SPEED         GPIO_SPEED_HIGH
/// #define BNRG_SPI_MOSI_ALTERNATE     GPIO_AF5_SPI1
/// #define BNRG_SPI_MOSI_PORT          GPIOA
/// #define BNRG_SPI_MOSI_CLK_ENABLE()  __GPIOA_CLK_ENABLE()

/// // NSS/CSN/CS: PB.0
/// #define BNRG_SPI_CS_PIN             GPIO_PIN_0
/// #define BNRG_SPI_CS_MODE            GPIO_MODE_OUTPUT_PP
/// #define BNRG_SPI_CS_PULL            GPIO_PULLUP
/// #define BNRG_SPI_CS_SPEED           GPIO_SPEED_HIGH
/// #define BNRG_SPI_CS_ALTERNATE       0
/// #define BNRG_SPI_CS_PORT            GPIOB
/// #define BNRG_SPI_CS_CLK_ENABLE()    __GPIOB_CLK_ENABLE()

/// // IRQ: PA.4
/// #define BNRG_SPI_IRQ_PIN            GPIO_PIN_4
/// #define BNRG_SPI_IRQ_MODE           GPIO_MODE_IT_RISING
/// #define BNRG_SPI_IRQ_PULL           GPIO_NOPULL
/// #define BNRG_SPI_IRQ_SPEED          GPIO_SPEED_HIGH
/// #define BNRG_SPI_IRQ_ALTERNATE      0
/// #define BNRG_SPI_IRQ_PORT           GPIOA
/// #define BNRG_SPI_IRQ_CLK_ENABLE()   __GPIOA_CLK_ENABLE()

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccessByte {
    Write = 0x0A,
    Read  = 0x0B,
}

#[derive(Debug, PartialEq)]
pub enum BTError<SpiError, GpioError> {
    /// SPI errors occur if there is an underlying error during a transfer.
    Spi(SpiError),

    /// GPIO errors occur if there is an underlying error resetting the pin, setting the chip select
    /// pin, or reading if data is available.
    Gpio(GpioError),
}

pub struct BluetoothSpi<'buf, SPI, CS, Reset, Input> {
    // pub struct BluetoothSpi<SPI, CS, Reset, Input> {
    spi:    SPI,
    cs:     CS,
    reset:  Reset,
    input:  Input,
    // buffer: ArrayVec<u8, 256>,
    buffer: Buffer<'buf, u8>,

    pub state:    BTState,
    // pub services: Option<SvLogger>,
    pub services: BTServices,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct BTServices {
    pub logger:  Option<SvLogger>,
    pub sensors: Option<SvSensors>,
}

/// new
impl<'buf, SPI, CS, Reset, Input> BluetoothSpi<'buf, SPI, CS, Reset, Input> {
    pub fn new(
        spi: SPI,
        cs: CS,
        reset: Reset,
        input: Input,
        buffer: &'buf mut [u8],
    ) -> Self {
        Self {
            spi,
            cs,
            reset,
            input,
            buffer: Buffer::new(buffer),

            state: BTState::Disconnected,
            services: BTServices::default(),
        }
    }
}

impl<'buf, SPI, CS, Reset, GpioError> BluetoothSpi<'buf, SPI, CS, Reset, PA4>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = SpiError>
        + hal::blocking::spi::Write<u8, Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
{
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

/// reset, bluenrg fns
// impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
impl<'buf, SPI, CS, Reset, Input, GpioError> BluetoothSpi<'buf, SPI, CS, Reset, Input>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = SpiError>
        + hal::blocking::spi::Write<u8, Error = SpiError>,

    // SPI: Transfer<u8, Error = SpiError>
    //     + Write<u8, Error = SpiError>
    //     + Read<u8, Error = SpiError>
    //     + TransferInplace<u8, Error = SpiError>,
    // CS: OutputPin<Error = GpioError>,
    // Reset: OutputPin<Error = GpioError>,
    // Input: InputPin<Error = GpioError>,
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
{
    pub fn reset_with_delay<D, UXX>(
        &mut self,
        delay: &mut D,
        time: UXX,
    ) -> nb::Result<(), GpioError>
    where
        D: hal::blocking::delay::DelayMs<UXX>,
        UXX: Copy,
    {
        self.reset.set_low().map_err(nb::Error::Other)?;
        // self.reset.set_high().map_err(nb::Error::Other)?;
        delay.delay_ms(time);

        self.reset.set_high().map_err(nb::Error::Other)?;
        // self.reset.set_low().map_err(nb::Error::Other)?;
        delay.delay_ms(time);

        Ok(())
    }

    pub fn _data_ready(&self) -> nb::Result<bool, BTError<SpiError, GpioError>> {
        self.input
            .is_high()
            .map_err(BTError::Gpio)
            .map_err(nb::Error::Other)
    }

    pub fn data_ready(&self) -> Result<bool, Input::Error> {
        self.input.is_high()
    }

    pub fn block_until_ready(
        &mut self,
        access_byte: AccessByte,
        mut uart: Option<&mut UART>,
    ) -> nb::Result<(u16, u16), BTError<SpiError, GpioError>> {
        let mut x = 0;
        loop {
            x += 1;
            // let mut write_header = [access_byte as u8, 0x00, 0x00, 0x00, 0x00];
            // let mut read_header = [0xff; 5];
            // let e = self.spi.transfer(&mut read_header, &write_header);

            let mut header = [access_byte as u8, 0x00, 0x00, 0x00, 0x00];

            self.spi
                .transfer(&mut header)
                .map_err(BTError::Spi)
                .map_err(nb::Error::Other)?;

            if let Some(ref mut uart) = uart {
                for b in header {
                    uprint!(uart, "{:#04x} ", b);
                }
                uprintln!(uart, "");
            }

            match parse_spi_header(&header) {
                Ok(lens) => {
                    if let Some(ref mut uart) = uart {
                        uprintln!(uart, "ready in {:?} loops", x);
                    }
                    return Ok(lens);
                }
                Err(nb::Error::WouldBlock) => {
                    self.cs
                        .set_high()
                        .map_err(BTError::Gpio)
                        .map_err(nb::Error::Other)?;
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

    /// Write data to the chip over the SPI bus. First writes a BlueNRG SPI header to the
    /// controller, indicating the host wants to write. The controller returns one byte indicating
    /// whether or not it is ready, followed by a pair of u16s in little endian: the first is the
    /// number of bytes the controller can receive, and the second is the number of bytes the
    /// controller has ready to transmit.
    ///
    /// If the controller claims to have enough room to receive the header and payload, this writes
    /// the header immediately followed by the payload.
    ///
    /// # Errors
    ///
    /// - Returns nb::Error::WouldBlock if the controller is not ready to receive data or if it
    ///   reports that it does not have enough space to accept the combined header and payload.
    ///
    /// - Returns a communication error if there is an error communicating over the SPI bus.
    fn try_write(
        &mut self,
        header: &[u8],
        payload: &[u8],
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        if !header.is_empty() {
            self.spi
                .write(header)
                .map_err(BTError::Spi)
                .map_err(nb::Error::Other)?;
        }
        if !payload.is_empty() {
            self.spi
                .write(payload)
                .map_err(BTError::Spi)
                .map_err(nb::Error::Other)?;
        }

        Ok(())
    }

    /// Read data from the chip over the SPI bus. First writes a BlueNRG SPI header to the
    /// controller, indicating that the host wants to read. The controller returns one byte
    /// indicating whether or not it is ready, followed by a pair of u16s in little endian: the
    /// first is the number of bytes the controller can receive, and the second is the number of
    /// bytes the controller has ready to transmit.
    ///
    /// If the controller is ready and has data available, reads the available data into the host's
    /// RX buffer, until either there is no more data or the RX buffer is full, whichever comes
    /// first.
    ///
    /// # Errors
    ///
    /// - Returns nb::Error::WouldBlock if the controller is not ready.
    ///
    /// - Returns a communication error if there is an error communicating over the SPI bus.
    fn read_available_data(&mut self) -> nb::Result<(), BTError<SpiError, GpioError>> {
        if !self
            .data_ready()
            .map_err(BTError::Gpio)
            .map_err(nb::Error::Other)?
        {
            return Err(nb::Error::WouldBlock);
        }

        let read_len = self.block_until_ready_for(AccessByte::Read, None)?;
        let mut bytes_available = read_len as usize;
        while bytes_available > 0 && self.buffer.next_contiguous_slice_len() > 0 {
            let transfer_count =
                usize::min(bytes_available, self.buffer.next_contiguous_slice_len());
            {
                let rx = self.buffer.next_mut_slice(transfer_count);
                for byte in rx.iter_mut() {
                    *byte = 0;
                }
                self.spi
                    .transfer(rx)
                    .map_err(BTError::Spi)
                    .map_err(nb::Error::Other)?;
            }
            bytes_available -= transfer_count;
        }

        Ok(())
    }

    #[cfg(feature = "nope")]
    fn read_available_data_into(
        &mut self,
        uart: &mut UART,
        buf: &mut [u8],
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        if !self
            .data_ready()
            .map_err(BTError::Gpio)
            .map_err(nb::Error::Other)?
        {
            return Err(nb::Error::WouldBlock);
        }

        let read_len = self.block_until_ready_for(AccessByte::Read, Some(uart))? as usize;

        if read_len > buf.len() {
            uprintln!(uart, "buffer too small");
            return Ok(());
        }

        self.spi
            .transfer(&mut buf[..read_len])
            .map_err(BTError::Spi)
            .map_err(nb::Error::Other)?;

        Ok(())
    }

    pub fn write_command(
        &mut self,
        opcode: Opcode,
        params: &[u8],
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        const HEADER_LEN: usize = 4;
        let mut header = [0; HEADER_LEN];
        bluetooth_hci::host::uart::CommandHeader::new(opcode, params.len())
            .copy_into_slice(&mut header);

        self.write(&header, params)
    }
}

/// tests
impl<'buf, SPI, CS, Reset, Input, GpioError> BluetoothSpi<'buf, SPI, CS, Reset, Input>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = SpiError>
        + hal::blocking::spi::Write<u8, Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
{
    pub fn test1(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<usize, BTError<SpiError, GpioError>> {
        while !self
            .data_ready()
            .map_err(BTError::Gpio)
            .map_err(nb::Error::Other)?
        {
            return Err(nb::Error::WouldBlock);
            // cortex_m::asm::nop();
        }
        // uprintln!(uart, "wat 2");

        self.cs
            .set_low()
            .map_err(BTError::Gpio)
            .map_err(nb::Error::Other)?;

        // let read_len = block!(self.block_until_ready_for(AccessByte::Read, Some(uart)))?;
        let read_len = block!(self.block_until_ready_for(AccessByte::Read, None))?;

        let mut rx = [0u8; 64];

        uprintln!(uart, "read_len = {:?}", read_len);

        self.spi
            .transfer(&mut rx[..read_len as usize])
            .map_err(BTError::Spi)
            .map_err(nb::Error::Other)?;

        self.cs
            .set_high()
            .map_err(BTError::Gpio)
            .map_err(nb::Error::Other)?;

        for c in rx.chunks(32) {
            for b in c {
                uprint!(uart, "{:#04x} ", b);
            }
            uprintln!(uart, "");
        }
        uprintln!(uart, "");

        let param_len = rx[1] as usize;
        // uprintln!(uart, "param_len = {:?}", param_len);

        Ok(param_len)
    }

    pub fn test2(
        &mut self,
        uart: &mut UART,
        param_len: usize,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        const MAX_EVENT_LENGTH: usize = 255;
        const PACKET_HEADER_LENGTH: usize = 1;
        const EVENT_PACKET_HEADER_LENGTH: usize = 3;
        const PARAM_LEN_BYTE: usize = 2;

        let mut buf = [0; MAX_EVENT_LENGTH + EVENT_PACKET_HEADER_LENGTH];

        self.cs
            .set_low()
            .map_err(BTError::Gpio)
            .map_err(nb::Error::Other)?;
        // self.read_available_data_into(uart, &mut buf[..EVENT_PACKET_HEADER_LENGTH + param_len])?;
        self.cs
            .set_high()
            .map_err(BTError::Gpio)
            .map_err(nb::Error::Other)?;

        // self.read_into(&mut buf[..EVENT_PACKET_HEADER_LENGTH + param_len])?;

        for c in buf.chunks(32) {
            for b in c {
                uprint!(uart, "{:#04x} ", b);
            }
            uprintln!(uart, "");
        }

        Ok(())
    }

    pub fn test3(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        // let e = block!(self.read_local_version_information())?;
        // uprintln!(uart, "e = {:?}", e);

        Ok(())
    }
}

// impl<CS, Reset, Input, GpioError> Controller for BluetoothSpi<CS, Reset, Input>
impl<'buf, SPI, CS, Reset, Input, GpioError> Controller
    for BluetoothSpi<'buf, SPI, CS, Reset, Input>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = SpiError>
        + hal::blocking::spi::Write<u8, Error = SpiError>,
    // SPI: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError> + Read<u8, Error = SpiError>,
    // SPI: Transfer<u8, Error = SpiError>
    //     + Write<u8, Error = SpiError>
    //     + Read<u8, Error = SpiError>
    //     + TransferInplace<u8, Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
{
    type Error = BTError<SpiError, GpioError>;
    type Header = bluetooth_hci::host::uart::CommandHeader;
    type Vendor = BlueNRGTypes;

    fn write(&mut self, header: &[u8], payload: &[u8]) -> nb::Result<(), Self::Error> {
        self.cs
            .set_low()
            .map_err(BTError::Gpio)
            .map_err(nb::Error::Other)?;

        let write_len = self.block_until_ready_for(AccessByte::Write, None)?;
        if (write_len as usize) < header.len() + payload.len() {
            return Err(nb::Error::WouldBlock);
        }

        let result = self.try_write(header, payload);
        self.cs
            .set_high()
            .map_err(BTError::Gpio)
            .map_err(nb::Error::Other)?;
        result
    }

    fn read_into(&mut self, buffer: &mut [u8]) -> nb::Result<(), Self::Error> {
        let result = if buffer.len() > self.buffer.size() {
            self.cs
                .set_low()
                .map_err(BTError::Gpio)
                .map_err(nb::Error::Other)?;
            let r = self.read_available_data();
            self.cs
                .set_high()
                .map_err(BTError::Gpio)
                .map_err(nb::Error::Other)?;
            r
        } else {
            Ok(())
        };

        if buffer.len() <= self.buffer.size() {
            self.buffer.take_slice(buffer.len(), buffer);
            Ok(())
        } else if let Err(e) = result {
            Err(e)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn peek(&mut self, n: usize) -> nb::Result<u8, Self::Error> {
        if n >= self.buffer.size() {
            if !self
                .data_ready()
                .map_err(BTError::Gpio)
                .map_err(nb::Error::Other)?
            {
                return Err(nb::Error::WouldBlock);
            }

            self.cs
                .set_low()
                .map_err(BTError::Gpio)
                .map_err(nb::Error::Other)?;
            let result = self.read_available_data();
            self.cs
                .set_high()
                .map_err(BTError::Gpio)
                .map_err(nb::Error::Other)?;

            if n >= self.buffer.size() {
                if let Err(e) = result {
                    return Err(e);
                }

                // Returns WouldBlock below
            }
        }

        if n < self.buffer.size() {
            Ok(self.buffer.peek(n))
        } else {
            Err(nb::Error::WouldBlock)
        }
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
        // hprintln!("b");
        // hprintln!(
        //     "{:#04x} {:#04x} {:#04x} {:#04x} {:#04x} ",
        //     header[0],
        //     header[1],
        //     header[2],
        //     header[3],
        //     header[4],
        // );

        Err(nb::Error::WouldBlock)
    }
}

/// Specify vendor-specific extensions for the BlueNRG.
pub struct BlueNRGTypes;
impl bluetooth_hci::Vendor for BlueNRGTypes {
    type Status = self::events::Status;
    type Event = self::events::BlueNRGEvent;
}

/// Vendor-specific interpretation of the local version information from the controller.
#[derive(Clone)]
pub struct Version {
    /// Version of the controller hardware.
    pub hw_version: u8,

    /// Major version of the controller firmware
    pub major: u8,

    /// Minor version of the controller firmware
    pub minor: u8,

    /// Patch version of the controller firmware
    pub patch: u8,
}

/// Extension trait to convert [`hci::event::command::LocalVersionInfo`] into the BlueNRG-specific
/// [`Version`] struct.
pub trait LocalVersionInfoExt {
    /// Converts LocalVersionInfo as returned by the controller into a BlueNRG-specific [`Version`]
    /// struct.
    fn bluenrg_version(&self) -> Version;
}

impl<VS> LocalVersionInfoExt for bluetooth_hci::event::command::LocalVersionInfo<VS> {
    fn bluenrg_version(&self) -> Version {
        Version {
            hw_version: (self.hci_revision >> 8) as u8,
            major:      (self.hci_revision & 0xFF) as u8,
            minor:      ((self.lmp_subversion >> 4) & 0xF) as u8,
            patch:      (self.lmp_subversion & 0xF) as u8,
        }
    }
}

/// Hardware event codes returned by the `HardwareError` HCI event.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum HardwareError {
    /// Error on the SPI bus has been detected, most likely caused by incorrect SPI configuration on
    /// the external micro-controller.
    SpiFraming,

    /// Caused by a slow crystal startup and they are an indication that the HS_STARTUP_TIME in the
    /// device configuration needs to be tuned. After this event is recommended to hardware reset
    /// the device.
    RadioState,

    /// Caused by a slow crystal startup and they are an indication that the HS_STARTUP_TIME in the
    /// device configuration needs to be tuned. After this event is recommended to hardware reset
    /// the device.
    TimerOverrun,
}

/// Error type for `TryFrom<u8>` to `HardwareError`. Includes the invalid byte.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct InvalidHardwareError(pub u8);

impl TryFrom<u8> for HardwareError {
    type Error = InvalidHardwareError;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(HardwareError::SpiFraming),
            1 => Ok(HardwareError::RadioState),
            2 => Ok(HardwareError::TimerOverrun),
            _ => Err(InvalidHardwareError(value)),
        }
    }
}

#[cfg(feature = "nope")]
pub mod hci {
    use byteorder::{ByteOrder, LittleEndian};
    use stm32f4xx_hal::nb;

    pub struct BluetoothHCI {
        //
    }

    // pub struct HciRequist {
    //     ogf: u16,
    //     ocf: u16,
    //     // event: i32,
    // }

    impl BluetoothHCI {
        pub fn new() -> Self {
            Self {}
        }
    }

    impl BluetoothHCI {
        pub fn send_req(&mut self) {
            unimplemented!()
        }

        // pub fn read_local_version(&mut self, )
    }
}
