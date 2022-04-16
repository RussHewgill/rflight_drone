use embedded_hal as hal;
use hal::digital::v2::{InputPin, OutputPin};

use stm32f4::stm32f401::{RCC, SPI1};
use stm32f4xx_hal::{
    block, nb,
    prelude::*,
    spi::{Error as SpiError, NoMiso},
};

use bluetooth_hci::host::uart::Hci as HciUart;
use bluetooth_hci::host::Hci;

use crate::{uart::*, uprintln};

// use super::gap::Commands as GapCommands;
use super::gatt::Commands as GattCommands;
// use super::hal::Commands as HalCommands;
// use super::l2cap::Commands as L2CapCommands;

use bluetooth_hci::event::command::ReturnParameters;
use bluetooth_hci::event::{Event, VendorEvent};

use crate::bluetooth::ev_command::ReturnParameters as VReturnParameters;
// use crate::bluetooth::{events::BlueNRGEvent, BTError, BluetoothSpi};
use crate::bluetooth::{events::BlueNRGEvent, BTError};

use crate::bluetooth::ActiveBlueNRG;

impl<'bnrg, 'spi, 'dbuf, SPI, OutputPin1, OutputPin2, InputPin, SpiError, GpioError>
    ActiveBlueNRG<'bnrg, 'spi, 'dbuf, SPI, OutputPin1, OutputPin2, InputPin, GpioError>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = SpiError>
        + hal::blocking::spi::Write<u8, Error = SpiError>,
    OutputPin1: hal::digital::v2::OutputPin<Error = GpioError>,
    OutputPin2: hal::digital::v2::OutputPin<Error = GpioError>,
    InputPin: hal::digital::v2::InputPin<Error = GpioError>,
    SpiError: core::fmt::Debug,
    GpioError: core::fmt::Debug,
{
    pub fn init_bluetooth(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        // bt.reset_with_delay(&mut delay, 10u32).unwrap();

        // self.init()?;

        block!(self.read_local_version_information())?;

        // block!(self.init_gatt());

        let x: Result<
            bluetooth_hci::host::uart::Packet<BlueNRGEvent>,
            bluetooth_hci::host::uart::Error<
                BTError<SpiError, GpioError>,
                crate::bluetooth::events::BlueNRGError,
            >,
        > = block!(self.read());

        // match block!(self.read()) {
        match x {
            Ok(p) => {
                let bluetooth_hci::host::uart::Packet::Event(e) = p;
                match e {
                    // Event::ConnectionComplete(params) => {
                    //     // handle the new connection
                    // }
                    Event::Vendor(crate::bluetooth::events::BlueNRGEvent::HalInitialized(
                        reason,
                    )) => {
                        uprintln!(uart, "bt restarted, reason = {:?}", reason);
                    }
                    Event::CommandComplete(params) => {
                        let params: bluetooth_hci::event::command::CommandComplete<
                            crate::bluetooth::events::BlueNRGEvent,
                        > = params;

                        uprintln!(uart, "CommandComplete");

                        match params.return_params {
                            ReturnParameters::Vendor(VReturnParameters::GattInit(status)) => {
                                match status {
                                    _ => {
                                        uprintln!(uart, "status = {:?}", status);
                                    }
                                }
                            }
                            ReturnParameters::ReadLocalVersionInformation(v) => {
                                unimplemented!()
                            }
                            ps => {
                                uprintln!(uart, "Other: return_params = {:?}", ps);
                            }
                        }

                        // unimplemented!()
                    }
                    // super::ev_command::ReturnParameters::GattInit(status) => {
                    // }
                    ev => {
                        uprintln!(uart, "ev = {:?}", ev);
                    }
                    // _ => unimplemented!(),
                }
            }

            Err(e) => {
                let e: bluetooth_hci::host::uart::Error<
                        BTError<SpiError, GpioError>, crate::bluetooth::events::BlueNRGError> = e;

                match e {
                    bluetooth_hci::host::uart::Error::Comm(e) => {
                        // unimplemented!()
                        uprintln!(uart, "error 0 = {:?}", e);
                    }
                    bluetooth_hci::host::uart::Error::BadPacketType(e) => {
                        uprintln!(uart, "error 1 = {:?}", e);
                        // unimplemented!()
                    }
                    bluetooth_hci::host::uart::Error::BLE(e) => {
                        uprintln!(uart, "error 2 = {:?}", e);
                        // unimplemented!()
                    }
                }

                // uprintln!(uart, "error = {:?}", e);
            }

            // _ => unimplemented!(),

            // Err(e) => match e {
            //     nb::Error::Other(error) => {
            //         uprintln!(uart, "error = {:?}", error);
            //     }
            //     nb::Error::WouldBlock => (),
            // }, // _ => unimplemented!(),
        }

        Ok(())
    }
}

#[cfg(feature = "nope")]
impl<'buf, SPI, CS, Reset, Input, GpioError> BluetoothSpi<'buf, SPI, CS, Reset, Input>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = SpiError>
        + hal::blocking::spi::Write<u8, Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    pub fn init_bluetooth(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        // bt.reset_with_delay(&mut delay, 10u32).unwrap();

        // self.init()?;

        block!(self.read_local_version_information())?;

        // block!(self.init_gatt());

        let x: Result<
            bluetooth_hci::host::uart::Packet<BlueNRGEvent>,
            bluetooth_hci::host::uart::Error<
                BTError<SpiError, GpioError>,
                crate::bluetooth::events::BlueNRGError,
            >,
        > = block!(self.read());

        // match block!(self.read()) {
        match x {
            Ok(p) => {
                let bluetooth_hci::host::uart::Packet::Event(e) = p;
                match e {
                    // Event::ConnectionComplete(params) => {
                    //     // handle the new connection
                    // }
                    Event::Vendor(crate::bluetooth::events::BlueNRGEvent::HalInitialized(
                        reason,
                    )) => {
                        uprintln!(uart, "bt restarted, reason = {:?}", reason);
                    }
                    Event::CommandComplete(params) => {
                        let params: bluetooth_hci::event::command::CommandComplete<
                            crate::bluetooth::events::BlueNRGEvent,
                        > = params;

                        uprintln!(uart, "CommandComplete");

                        match params.return_params {
                            ReturnParameters::Vendor(VReturnParameters::GattInit(status)) => {
                                match status {
                                    _ => {
                                        uprintln!(uart, "status = {:?}", status);
                                    }
                                }
                            }
                            ReturnParameters::ReadLocalVersionInformation(v) => {
                                unimplemented!()
                            }
                            ps => {
                                uprintln!(uart, "Other: return_params = {:?}", ps);
                            }
                        }

                        // unimplemented!()
                    }
                    // super::ev_command::ReturnParameters::GattInit(status) => {
                    // }
                    ev => {
                        uprintln!(uart, "ev = {:?}", ev);
                    }
                    // _ => unimplemented!(),
                }
            }

            Err(e) => {
                let e: bluetooth_hci::host::uart::Error<
                        BTError<SpiError, GpioError>, crate::bluetooth::events::BlueNRGError> = e;

                match e {
                    bluetooth_hci::host::uart::Error::Comm(e) => {
                        // unimplemented!()
                        uprintln!(uart, "error 0 = {:?}", e);
                    }
                    bluetooth_hci::host::uart::Error::BadPacketType(e) => {
                        uprintln!(uart, "error 1 = {:?}", e);
                        // unimplemented!()
                    }
                    bluetooth_hci::host::uart::Error::BLE(e) => {
                        uprintln!(uart, "error 2 = {:?}", e);
                        // unimplemented!()
                    }
                }

                // uprintln!(uart, "error = {:?}", e);
            }

            // _ => unimplemented!(),

            // Err(e) => match e {
            //     nb::Error::Other(error) => {
            //         uprintln!(uart, "error = {:?}", error);
            //     }
            //     nb::Error::WouldBlock => (),
            // }, // _ => unimplemented!(),
        }

        Ok(())
    }
}
