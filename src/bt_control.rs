use embedded_hal as hal;
use hal::digital::v2::{InputPin, OutputPin};

use stm32f4::stm32f401::{RCC, SPI1};
use stm32f4xx_hal::{
    block, nb,
    prelude::*,
    spi::{Error as SpiError, NoMiso},
};

use bluetooth_hci::host::uart::{CommandHeader, Hci as HciUart};
use bluetooth_hci::host::Hci;

use crate::{bluetooth::ev_command::GapInit, uart::*, uprintln};

// use super::gap::Commands as GapCommands;
use super::gatt::Commands as GattCommands;
// use super::hal::Commands as HalCommands;
// use super::l2cap::Commands as L2CapCommands;

use bluetooth_hci::event::command::ReturnParameters;
use bluetooth_hci::event::{Event, VendorEvent};

use crate::bluetooth::ev_command::ReturnParameters as VReturnParameters;
use crate::bluetooth::{events::BlueNRGEvent, BTError, BluetoothSpi};

impl<'buf, SPI, CS, Reset, Input, GpioError> BluetoothSpi<'buf, SPI, CS, Reset, Input>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = SpiError>
        + hal::blocking::spi::Write<u8, Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    fn handle_event_command_complete(
        uart: &mut UART,
        return_params: ReturnParameters<BlueNRGEvent>,
    ) {
        match return_params {
            ReturnParameters::Vendor(VReturnParameters::GattInit(status)) => match status {
                _ => {
                    uprintln!(uart, "status = {:?}", status);
                }
            },
            ReturnParameters::ReadLocalVersionInformation(v) => {
                uprintln!(uart, "v = {:?}", v);
            }
            ps => {
                uprintln!(uart, "Other: return_params = {:?}", ps);
            }
        }
    }

    pub fn _read_event(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<bluetooth_hci::Event<BlueNRGEvent>, BTError<SpiError, GpioError>> {
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
                return Ok(e);
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
                unimplemented!()
            }
        }
    }

    pub fn read_event(&mut self, uart: &mut UART) -> nb::Result<(), BTError<SpiError, GpioError>> {
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
                uprintln!(uart, "event = {:?}", &e);
                match e {
                    Event::ConnectionComplete(params) => {
                        // handle the new connection
                    }
                    Event::Vendor(crate::bluetooth::events::BlueNRGEvent::HalInitialized(
                        reason,
                    )) => {
                        uprintln!(uart, "bt restarted, reason = {:?}", reason);
                    }
                    Event::CommandComplete(params) => {
                        Self::handle_event_command_complete(uart, params.return_params);
                    }
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

    pub fn read_event_gap_init(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<GapInit, BTError<SpiError, GpioError>> {
        match self._read_event(uart)? {
            Event::CommandComplete(params) => match params.return_params {
                ReturnParameters::Vendor(VReturnParameters::GapInit(g)) => return Ok(g),
                _ => unimplemented!(),
            },
            _ => unimplemented!(),
        }
    }
}

fn rewrap_error<E, VE>(e: nb::Error<E>) -> nb::Error<bluetooth_hci::host::uart::Error<E, VE>> {
    match e {
        nb::Error::WouldBlock => nb::Error::WouldBlock,
        nb::Error::Other(err) => nb::Error::Other(bluetooth_hci::host::uart::Error::Comm(err)),
    }
}

fn read_event<E, T, Vendor, VE>(
    controller: &mut T,
) -> nb::Result<bluetooth_hci::Event<Vendor>, bluetooth_hci::host::uart::Error<E, VE>>
where
    T: bluetooth_hci::Controller<Error = E>,
    Vendor: bluetooth_hci::event::VendorEvent<Error = VE>,
{
    const MAX_EVENT_LENGTH: usize = 255;
    const PACKET_HEADER_LENGTH: usize = 1;
    const EVENT_PACKET_HEADER_LENGTH: usize = 3;
    const PARAM_LEN_BYTE: usize = 2;

    let param_len = controller.peek(PARAM_LEN_BYTE).map_err(rewrap_error)? as usize;

    let mut buf = [0; MAX_EVENT_LENGTH + EVENT_PACKET_HEADER_LENGTH];
    controller
        .read_into(&mut buf[..EVENT_PACKET_HEADER_LENGTH + param_len])
        .map_err(rewrap_error)?;

    bluetooth_hci::Event::new(bluetooth_hci::event::Packet(
        &buf[PACKET_HEADER_LENGTH..EVENT_PACKET_HEADER_LENGTH + param_len],
    ))
    .map_err(|e| nb::Error::Other(bluetooth_hci::host::uart::Error::BLE(e)))
}

pub trait HciRead<E, Vendor, VE>: bluetooth_hci::host::Hci<E> {
    /// Reads and returns a packet from the controller. Consumes exactly enough bytes to read the
    /// next packet including its header.
    ///
    /// # Errors
    ///
    /// - Returns [`nb::Error::WouldBlock`] if the controller does not have enough bytes available
    ///   to read the full packet right now.
    /// - Returns [`nb::Error::Other`]`(`[`Error::BadPacketType`]`)` if the next byte is not a valid
    ///   packet type.
    /// - Returns [`nb::Error::Other`]`(`[`Error::BLE`]`)` if there is an error deserializing the
    ///   packet (such as a mismatch between the packet length and the expected length of the
    ///   event). See [`crate::event::Error`] for possible values of `e`.
    /// - Returns [`nb::Error::Other`]`(`[`Error::Comm`]`)` if there is an error reading from the
    ///   controller.
    fn read2(
        &mut self,
    ) -> nb::Result<
        bluetooth_hci::host::uart::Packet<Vendor>,
        bluetooth_hci::host::uart::Error<E, VE>,
    >
    where
        Vendor: bluetooth_hci::event::VendorEvent<Error = VE>;
}

const PACKET_TYPE_HCI_COMMAND: u8 = 0x01;
// const PACKET_TYPE_ACL_DATA: u8 = 0x02;
// const PACKET_TYPE_SYNC_DATA: u8 = 0x03;
const PACKET_TYPE_HCI_EVENT: u8 = 0x04;
const PACKET_TYPE_VS_EVENT: u8 = 0xFF;

impl<E, Vendor, VE, T> HciRead<E, Vendor, VE> for T
where
    T: bluetooth_hci::Controller<Error = E, Header = CommandHeader>,
{
    fn read2(
        &mut self,
    ) -> nb::Result<
        bluetooth_hci::host::uart::Packet<Vendor>,
        bluetooth_hci::host::uart::Error<E, VE>,
    >
    where
        Vendor: bluetooth_hci::event::VendorEvent<Error = VE>,
    {
        match self.peek(0).map_err(rewrap_error)? {
            PACKET_TYPE_HCI_EVENT => {
                Ok(bluetooth_hci::host::uart::Packet::Event(read_event(self)?))
            }
            PACKET_TYPE_VS_EVENT => Ok(bluetooth_hci::host::uart::Packet::Event(read_event(self)?)),
            x => Err(nb::Error::Other(
                bluetooth_hci::host::uart::Error::BadPacketType(x),
            )),
        }
    }
}
