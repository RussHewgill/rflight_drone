use arrayvec::ArrayVec;

use bluetooth_hci::Controller;
use byteorder::{ByteOrder, LittleEndian};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use stm32f4::stm32f401::{EXTI, TIM2};
use stm32f4xx_hal::{
    block,
    gpio::{Alternate, ExtiPin, Pin, PA4},
    nb,
    prelude::*,
    spi::Spi1,
    timer::CounterMs,
};

use bluetooth_hci::{
    event::command::ReturnParameters,
    event::{Event, VendorEvent},
    host::Hci,
    host::{
        uart::{CommandHeader, Hci as HciUart},
        AdvertisingInterval, AdvertisingParameters,
    },
    types::FixedConnectionInterval,
    BdAddr, BdAddrType, ConnectionHandle,
};

use crate::{
    bluetooth::{
        events::BlueNRGEvent, rx_buffer::Buffer2, AccessByte, BTError, BTServices,
        BlueNRGTypes,
    },
    bt_control::BTState,
    spi::{Spi4, SpiError},
    uart::{CoreWrite, UART},
    uprintln,
};

pub struct BluetoothSpi2<CS, Reset, Input> {
    // pub struct BluetoothSpi2<'buf, CS, Reset, Input> {
    // spi:   Spi4,
    spi: Spi1<(
        Pin<'A', 5, Alternate<5>>,
        Pin<'A', 6, Alternate<5>>,
        Pin<'A', 7, Alternate<5>>,
    )>,
    cs:    CS,
    reset: Reset,
    input: Input,

    // buffer: ArrayVec<u8, 512>,
    buffer: Buffer2<u8>,

    // buffer: crate::bluetooth::rx_buffer::Buffer<'buf, u8>,
    pub delay: CounterMs<TIM2>,

    pub state:    BTState,
    pub services: BTServices,
}

/// new
impl<CS, Reset, Input> BluetoothSpi2<CS, Reset, Input> {
    // impl<'buf, CS, Reset, Input> BluetoothSpi2<'buf, CS, Reset, Input> {
    pub fn new(
        // spi: Spi4,
        spi: Spi1<(
            Pin<'A', 5, Alternate<5>>,
            Pin<'A', 6, Alternate<5>>,
            Pin<'A', 7, Alternate<5>>,
        )>,
        cs: CS,
        reset: Reset,
        input: Input,
        delay: CounterMs<TIM2>,
        // buffer: &'buf mut [u8],
    ) -> Self {
        Self {
            spi,
            cs,
            reset,
            input,
            delay,
            buffer: Buffer2::new(),
            // buffer: crate::bluetooth::rx_buffer::Buffer::new(buffer),
            state: BTState::Disconnected,
            services: BTServices::default(),
        }
    }
}

/// pause/resume/check/clear interrupt
impl<CS, Reset> BluetoothSpi2<CS, Reset, PA4> {
    // impl<'buf, CS, Reset> BluetoothSpi2<'buf, CS, Reset, PA4> {
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
// impl<'buf, CS, Reset, Input, GpioError> BluetoothSpi2<'buf, CS, Reset, Input>
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

    pub fn reset(&mut self) -> Result<(), GpioError> {
        // self.reset.set_low().map_err(nb::Error::Other)?;
        self.reset.set_low()?;
        self.wait_ms(10.millis());
        // self.reset.set_high().map_err(nb::Error::Other)?;
        self.reset.set_high()?;
        self.wait_ms(10.millis());
        Ok(())
    }

    pub fn data_ready(&self) -> Result<bool, Input::Error> {
        self.input.is_high()
    }
}

/// block until ready
impl<CS, Reset, Input, GpioError> BluetoothSpi2<CS, Reset, Input>
// impl<'buf, CS, Reset, Input, GpioError> BluetoothSpi2<'buf, CS, Reset, Input>
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

            // let send_header = [access_byte as u8, 0x00, 0x00, 0x00, 0x00];
            // let mut read_header = [0x00, 0x00, 0x00, 0x00, 0x00];

            self.spi
                .transfer(&mut header)
                .map_err(BTError::Spi)
                .map_err(nb::Error::Other)?;

            // self.spi.transfer(&mut read_header, &send_header).unwrap();
            // .map_err(BTError::Spi)
            // .map_err(nb::Error::Other)?;

            // uprintln!(uart.as_mut().unwrap(), "e = {:?}", e);

            // match parse_spi_header(&read_header) {
            match parse_spi_header(&header) {
                Ok(lens) => {
                    // uprintln!(
                    //     uart.as_mut().unwrap(),
                    //     "{:#04x} {:#04x} {:#04x} {:#04x} {:#04x} ",
                    //     read_header[0],
                    //     read_header[1],
                    //     read_header[2],
                    //     read_header[3],
                    //     read_header[4],
                    // );

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
                    // cortex_m::asm::nop();
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

/// try_write, read_available_data
impl<CS, Reset, Input, GpioError> BluetoothSpi2<CS, Reset, Input>
// impl<'buf, CS, Reset, Input, GpioError> BluetoothSpi2<'buf, CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
{
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

        const DUMMY_BYTES: [u8; 512] = [0; 512];

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

                // self.spi
                //     .transfer(rx, &DUMMY_BYTES)
                //     .map_err(BTError::Spi)
                //     .map_err(nb::Error::Other)?;
            }
            bytes_available -= transfer_count;
        }

        Ok(())
    }
}

impl<CS, Reset, Input, GpioError> Controller for BluetoothSpi2<CS, Reset, Input>
// impl<'buf, CS, Reset, Input, GpioError> Controller
//     for BluetoothSpi2<'buf, CS, Reset, Input>
where
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

    /// TODO:
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

    /// TODO:
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

/// read_event_uart
impl<CS, Reset, Input, GpioError> BluetoothSpi2<CS, Reset, Input>
// impl<'buf, CS, Reset, Input, GpioError> BluetoothSpi2<'buf, CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    pub fn read_event_uart(
        &mut self,
        uart: &mut UART,
    ) -> Result<(), BTError<SpiError, GpioError>> {
        let x: Result<
            bluetooth_hci::host::uart::Packet<BlueNRGEvent>,
            bluetooth_hci::host::uart::Error<
                BTError<SpiError, GpioError>,
                crate::bluetooth::events::BlueNRGError,
            >,
        > = block!(self.read());

        match x {
            Ok(p) => {
                let bluetooth_hci::host::uart::Packet::Event(e) = p;
                // uprintln!(uart, "event = {:?}", &e);
                match e {
                    Event::ConnectionComplete(params) => {
                        // handle the new connection
                    }
                    Event::Vendor(
                        crate::bluetooth::events::BlueNRGEvent::HalInitialized(reason),
                    ) => {
                        uprintln!(uart, "bt restarted, reason = {:?}", reason);
                    }
                    Event::CommandComplete(params) => {
                        // Self::handle_event_command_complete(uart, params.return_params);
                        uprintln!(
                            uart,
                            "params.return_params = {:?}",
                            params.return_params
                        );
                    }
                    // Event::LeConnectionComplete(conn) => {
                    //     unimplemented!()
                    // }
                    ev => {
                        uprintln!(uart, "unhandled event = {:?}", ev);
                    }
                }
            }

            Err(e) => {
                let e: bluetooth_hci::host::uart::Error<
                    BTError<SpiError, GpioError>,
                    crate::bluetooth::events::BlueNRGError,
                > = e;
                match e {
                    bluetooth_hci::host::uart::Error::Comm(e) => {
                        uprintln!(uart, "error 0 = {:?}", e);
                    }
                    bluetooth_hci::host::uart::Error::BadPacketType(e) => {
                        uprintln!(uart, "error 1 = {:?}", e);
                    }
                    bluetooth_hci::host::uart::Error::BLE(e) => {
                        uprintln!(uart, "error 2 = {:?}", e);
                    }
                }
            }
        }

        Ok(())
    }
}
