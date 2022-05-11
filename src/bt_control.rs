pub mod service_log;
pub mod service_sensors;

use embedded_hal as hal;
use hal::digital::v2::{InputPin, OutputPin};

// use rtt_target::rprintln;
use defmt::println as rprintln;

use stm32f4::stm32f401::{RCC, SPI1, TIM2};
use stm32f4xx_hal::{
    block,
    dwt::Dwt,
    gpio::{Alternate, Input, Output, Pin, PA4, PA5, PA6, PA7, PB0, PB2},
    nb,
    prelude::*,
    spi::{Error as SpiError, NoMiso, Spi1},
    timer::{CounterMs, DelayMs, SysDelay},
};

use bluetooth_hci::{
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
        ev_command::{GapInit, GattService},
        gap::{AuthenticationRequirements, DiscoverableParameters, LocalName},
        gatt::{
            AddCharacteristicParameters, AddServiceParameters, CharacteristicEvent,
            CharacteristicHandle, CharacteristicPermission, CharacteristicProperty,
            CharacteristicValue, EncryptionKeySize, ServiceHandle,
            UpdateCharacteristicValueParameters,
        },
        hal_bt::ConfigData,
    },
    uart::*,
    uprint, uprintln,
};

use crate::bluetooth::gap::Commands as GapCommands;
use crate::bluetooth::gatt::Commands as GattCommands;
use crate::bluetooth::hal_bt::Commands as HalCommands;

use crate::bluetooth::gap;
use crate::bluetooth::gatt;
use crate::bluetooth::hal_bt;

use bluetooth_hci::event::command::ReturnParameters;
use bluetooth_hci::event::{Event, VendorEvent};

use crate::bluetooth::ev_command::ReturnParameters as VReturnParameters;
use crate::bluetooth::{events::BlueNRGEvent, BTError, BluetoothSpi};

pub use self::uuids::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TimeoutResult {
    Ok,
    Timeout,
}

// pub type BTController<'spi> = BluetoothSpi<
//     'spi,
//     Spi1<
//         (
//             Pin<'A', 5, Alternate<5>>,
//             Pin<'A', 6, Alternate<5>>,
//             Pin<'A', 7, Alternate<5>>,
//         ),
//         stm32f4xx_hal::spi::TransferModeNormal,
//     >,
//     Pin<'B', 0, Output>,
//     Pin<'B', 2, Output>,
//     PA4,
// >;

pub type BTSpi = Spi1<
    (
        Pin<'A', 5, Alternate<5>>,
        Pin<'A', 6, Alternate<5>>,
        Pin<'A', 7, Alternate<5>>,
    ),
    stm32f4xx_hal::spi::TransferModeNormal,
>;

pub type BTController = BluetoothSpi<Pin<'B', 0, Output>, Pin<'B', 2, Output>, PA4>;

pub type BTEvent = bluetooth_hci::event::Event<BlueNRGEvent>;

mod uuids {
    pub const fn uuid_from_hex(uuid: u128) -> crate::bluetooth::gatt::Uuid {
        let x = uuid.to_le_bytes();
        crate::bluetooth::gatt::Uuid::Uuid128(x)
    }

    /// Logger
    pub const UUID_LOG_SERVICE: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x3f44d56a86074db0945b6c285b73d48a);

    pub const UUID_LOG_CHAR: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x1450781d919c49f0a16c0ec28dfb83d5);

    /// Sensors
    pub const UUID_SENSOR_SERVICE: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x639d0157e75a4eb9835298b676f51912);

    pub const UUID_SENSOR_CHAR: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x4f1f7252db544d4faa8f208d48637b3f);
}

/// init
impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    fn init_services(&mut self) -> nb::Result<(), BTError<SpiError, GpioError>> {
        self.init_log_service()?;
        self.init_sensor_service()?;
        Ok(())
    }

    pub fn init_bt(&mut self) -> nb::Result<(), BTError<SpiError, GpioError>> {
        // self.reset_with_delay(delay, 5u32).unwrap();
        self.reset().unwrap();
        self.read_event_uart()?;

        // block!(self.get_firmware_revision())?;
        // self.read_event_uart()?;

        self.reset().unwrap();
        self.read_event_uart()?;

        let addr = gen_bd_addr();

        // block!(self.write_config_data(&ConfigData::public_address(addr).build()))?;
        // block!(self.read_event(uart))?;

        block!(self.le_set_random_address(addr)).unwrap();
        self.read_event_uart()?;

        // uart.unpause();
        block!(self.init_gatt()).unwrap();
        self.read_event_uart()?;
        // uart.pause();

        let role = crate::bluetooth::gap::Role::PERIPHERAL;
        block!(self.init_gap(role, false, 7))?;
        let gap: GapInit = self.read_event_gap_init()?;

        // block!(self.clear_security_database())?;
        // block!(self.read_event(uart))?;

        // uprintln!(uart, "gap = {:?}", gap);

        static BLE_NAME: &'static str = "DRN1120";

        let ps = UpdateCharacteristicValueParameters {
            service_handle:        gap.service_handle,
            characteristic_handle: gap.dev_name_handle,
            offset:                0,
            value:                 BLE_NAME.as_bytes(),
        };
        block!(self.update_characteristic_value(&ps)).unwrap();
        self.read_event_uart()?;

        // block!(self.set_tx_power_level(hal_bt::PowerLevel::DbmNeg2_1))?;
        // self.read_event_uart()?;

        // block!(self.read_bd_addr()).unwrap();
        // block!(self.read_event(uart))?;

        // let addr = BdAddr([
        //     0x43 | 0b1100_0000,
        //     0xc6,
        //     0x3d,
        //     0xd9,
        //     0x74,
        //     0x4f | 0b1100_0000,
        // ]);

        let requirements = AuthenticationRequirements {
            mitm_protection_required:  true,
            out_of_band_auth:          gap::OutOfBandAuthentication::Disabled,
            encryption_key_size_range: (7, 16),
            fixed_pin:                 super::gap::Pin::Fixed(1),
            bonding_required:          true,
        };
        block!(self.set_authentication_requirement(&requirements)).unwrap();
        self.read_event_uart()?;

        let ad_params = AdvertisingParameters {
            advertising_interval:      AdvertisingInterval::for_type(
                bluetooth_hci::host::AdvertisingType::ConnectableUndirected,
            )
            .with_range(
                core::time::Duration::from_millis(200),
                core::time::Duration::from_millis(200),
            )
            .unwrap(),
            own_address_type:          bluetooth_hci::host::OwnAddressType::Random,
            peer_address:              bluetooth_hci::BdAddrType::Random(addr),
            advertising_channel_map:   bluetooth_hci::host::Channels::CH_37,
            advertising_filter_policy:
                bluetooth_hci::host::AdvertisingFilterPolicy::AllowConnectionAndScan,
        };
        block!(self.le_set_advertising_parameters(&ad_params)).unwrap();
        self.read_event_uart()?;

        block!(self.le_set_advertising_data(&[])).unwrap();
        self.read_event_uart()?;

        block!(self.le_set_scan_response_data(&[])).unwrap();
        self.read_event_uart()?;

        // uart.unpause();
        block!(self.le_set_advertise_enable(true))?;
        self.read_event_uart()?;
        // uart.pause();

        let dev_name: &'static [u8; 7] = b"DRN1120";

        let d_params = DiscoverableParameters {
            advertising_type:
                bluetooth_hci::host::AdvertisingType::ConnectableUndirected,
            advertising_interval: None,
            // advertising_interval: Some((
            //     core::time::Duration::from_millis(200),
            //     core::time::Duration::from_millis(200),
            // )),
            address_type:         bluetooth_hci::host::OwnAddressType::Random,
            filter_policy:
                bluetooth_hci::host::AdvertisingFilterPolicy::AllowConnectionAndScan,
            local_name:           Some(LocalName::Complete(&dev_name[..])),
            advertising_data:     &[],
            conn_interval:        (Some(core::time::Duration::from_millis(5000)), None),
        };

        block!(self.set_nondiscoverable()).unwrap();
        self.read_event_uart()?;

        // uart.unpause();
        block!(self.set_discoverable(&d_params)).unwrap();
        self.read_event_uart()?;

        self.init_services()?;

        // block!(self.read_bd_addr()).unwrap();
        // block!(self.read_event(uart))?;

        // let conn_handle: ConnectionHandle = match self._read_event(uart)? {
        //     Event::LeConnectionComplete(conn) => conn.conn_handle,
        //     _ => unimplemented!(),
        // };
        // uprintln!(uart, "conn = {:?}", conn_handle);

        // let xs = [1];
        // let val = UpdateCharacteristicValueParameters {
        //     // conn_handle,
        //     service_handle: console_service.0,
        //     characteristic_handle: console_service.1,
        //     offset: 0,
        //     value: &xs,
        // };
        // // block!(self.write_characteristic_value(&val)).unwrap();
        // block!(self.update_characteristic_value(&val)).unwrap();
        // block!(self.read_event(uart))?;

        // block!(self.read_event(uart))?;
        // block!(self.read_event(uart))?;

        // loop {
        //     block!(self.read_event(uart))?;
        //     if false {
        //         break;
        //     }
        // }

        Ok(())
    }
}

/// send commands
#[cfg(feature = "nope")]
impl<SPI, CS, Reset, Input, GpioError> BluetoothSpi<SPI, CS, Reset, Input>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = SpiError>
        + hal::blocking::spi::Write<u8, Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    #[cfg(feature = "nope")]
    pub fn send_command_timeout_retry<C>(
        &mut self,
        timeout: fugit::MillisDurationU32,
        command: C,
    ) -> nb::Result<
        crate::bluetooth::ev_command::ReturnParameters,
        BTError<SpiError, GpioError>,
    >
    where
        C: FnOnce(
            &mut BluetoothSpi<'buf, SPI, CS, Reset, Input>,
        ) -> nb::Result<(), BTError<SpiError, GpioError>>,
    {
        // block!(command(self))?;

        // self.delay.clear_interrupt(TimerEvent::Update);
        // self.delay.start(timeout).unwrap();
        // let p = loop {
        //     match self.read() {
        //         Ok(p) => break p,
        //         Err(nb::Error::WouldBlock) => {
        //             if self.delay.get_interrupt().contains(TimerEvent::Update) {
        //                 self.delay.cancel().unwrap();
        //                 self.delay.clear_interrupt(TimerEvent::Update);
        //                 return Ok(());
        //             }
        //         }
        //         Err(e) => {
        //             panic!("error 1 = {:?}", e);
        //         }
        //     }
        // };

        unimplemented!()
    }

    #[cfg(feature = "nope")]
    pub fn safe_update_char_value(
        &mut self,
        char_val: UpdateCharacteristicValueParameters,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        unimplemented!()
    }
}

/// read and handle events
impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    pub fn read_events_while_ready(
        &mut self,
        uart: &mut UART,
    ) -> Result<(), BTError<SpiError, GpioError>> {
        while self.data_ready().map_err(BTError::Gpio)? {
            if self.ignore_event_timeout(Some(1000.millis()))? == TimeoutResult::Timeout {
                uprintln!(uart, "read_events_while_ready: timoutout");
                break;
            }
        }
        Ok(())
    }

    /// blocks, but doesn't use nb::block!
    pub fn _read_event_timeout(
        &mut self,
        timeout: fugit::MicrosDurationU32,
        uart: &mut UART,
    ) -> Result<Option<bluetooth_hci::Event<BlueNRGEvent>>, BTError<SpiError, GpioError>>
    {
        use stm32f4xx_hal::timer::Event as TimerEvent;

        self.delay.clear_interrupt(TimerEvent::Update);
        self.delay.start(timeout).unwrap();
        loop {
            match self.read() {
                Ok(p) => {
                    self.delay.cancel().unwrap();
                    self.delay.clear_interrupt(TimerEvent::Update);
                    let bluetooth_hci::host::uart::Packet::Event(e) = p;
                    break Ok(Some(e));
                }
                Err(nb::Error::WouldBlock) => {
                    if self.delay.get_interrupt().contains(TimerEvent::Update) {
                        self.delay.cancel().unwrap();
                        self.delay.clear_interrupt(TimerEvent::Update);
                        return Ok(None);
                    }
                }
                Err(nb::Error::Other(e)) => {
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
                    unimplemented!("_read_event_timeout");
                    // panic!("error 1 = {:?}", e);
                }
            }
        }
    }

    pub fn _read_event(
        &mut self,
        // timeout: Option<fugit::MillisDurationU32>,
        // uart: &mut UART,
        // ) -> nb::Result<bluetooth_hci::Event<BlueNRGEvent>, BTError<SpiError, GpioError>>
    ) -> Result<bluetooth_hci::Event<BlueNRGEvent>, BTError<SpiError, GpioError>> {
        //

        let x: Result<
            bluetooth_hci::host::uart::Packet<BlueNRGEvent>,
            bluetooth_hci::host::uart::Error<
                BTError<SpiError, GpioError>,
                crate::bluetooth::events::BlueNRGError,
            >,
        > = block!(self.read());

        // let x: Result<
        //     bluetooth_hci::host::uart::Packet<BlueNRGEvent>,
        //     bluetooth_hci::host::uart::Error<
        //         BTError<SpiError, GpioError>,
        //         crate::bluetooth::events::BlueNRGError,
        //     >,
        // > = loop {
        //     match self.read() {
        //         Ok(x) => break Ok(x),
        //         Err(nb::Error::WouldBlock) => {
        //             //
        //             // let k = self.data_ready().unwrap();
        //             // uprintln!(uart, "rdy = {:?}", k);
        //             // unimplemented!()
        //         }
        //         Err(other) => {
        //             panic!("_read_event other 0 = {:?}", other);
        //         }
        //     }
        // };

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
                        rprintln!("error 0 = {:?}", defmt::Debug2Format(&e));
                    }
                    bluetooth_hci::host::uart::Error::BadPacketType(e) => {
                        rprintln!("error 1 = {:?}", defmt::Debug2Format(&e));
                    }
                    bluetooth_hci::host::uart::Error::BLE(e) => {
                        rprintln!("error 2 = {:?}", defmt::Debug2Format(&e));
                    }
                }
                unimplemented!()
            }
        }
    }

    pub fn ignore_event_timeout(
        &mut self,
        timeout: Option<fugit::MicrosDurationU32>,
    ) -> Result<TimeoutResult, BTError<SpiError, GpioError>> {
        use bluetooth_hci::Controller;
        use stm32f4xx_hal::timer::Event as TimerEvent;

        let x: Result<
            bluetooth_hci::host::uart::Packet<BlueNRGEvent>,
            bluetooth_hci::host::uart::Error<
                BTError<SpiError, GpioError>,
                crate::bluetooth::events::BlueNRGError,
            >,
        > = if let Some(timeout) = timeout {
            self.delay.clear_interrupt(TimerEvent::Update);
            self.delay.start(timeout).unwrap();

            // uprint!(uart, "w0");
            let p = loop {
                // uprint!(uart, "w1");
                // match self.read2(uart) {
                match self.read() {
                    Ok(p) => break p,
                    Err(nb::Error::WouldBlock) => {
                        if self.delay.get_interrupt().contains(TimerEvent::Update) {
                            // uprint!(uart, "w2");
                            self.delay.cancel().unwrap();
                            self.delay.clear_interrupt(TimerEvent::Update);
                            return Ok(TimeoutResult::Timeout);
                        }
                        // uprint!(uart, "w3");
                    }
                    Err(e) => {
                        panic!("error 1 = {:?}", e);
                    }
                }
            };
            // uprintln!(uart, "ld");

            self.delay.cancel().unwrap();
            self.delay.clear_interrupt(TimerEvent::Update);
            Ok(p)
            // unimplemented!()
        } else {
            block!(self.read())
            // unimplemented!()
        };

        let e = match x {
            Ok(p) => {
                let bluetooth_hci::host::uart::Packet::Event(e) = p;
                e
            }
            _ => unimplemented!(),
        };

        // uprintln!(uart, "event = {:?}", &e);
        match e {
            Event::ConnectionComplete(params) => {
                // handle the new connection
                rprintln!("new connection");
            }
            Event::Vendor(crate::bluetooth::events::BlueNRGEvent::HalInitialized(
                reason,
            )) => {
                rprintln!("bt restarted, reason = {:?}", defmt::Debug2Format(&reason));
            }
            Event::CommandComplete(params) => {
                // Self::handle_event_command_complete(uart, params.return_params);
                // rprintln!("CommandComplete = {:?}", params);
                if params.num_hci_command_packets == 0 {
                    rprintln!("controller required halting packets");
                }
                match params.return_params {
                    ReturnParameters::Vendor(
                        VReturnParameters::GattUpdateCharacteristicValue(status),
                    ) => {
                        if status != bluetooth_hci::Status::Success {
                            rprintln!("status 0 = {:?}", defmt::Debug2Format(&status));
                        } else {
                            // rprintln!("status 1 = {:?}", defmt::Debug2Format(&status));
                        }
                    }
                    _ => {
                        rprintln!(
                            "return_params = {:?}",
                            defmt::Debug2Format(&params.return_params)
                        );
                    }
                }
            }
            // Event::LeConnectionComplete(conn) => {
            //     unimplemented!()
            // }
            ev => {
                rprintln!("unhandled event = {:?}", defmt::Debug2Format(&ev));
            }
        }

        Ok(TimeoutResult::Ok)
    }

    #[cfg(feature = "nope")]
    pub fn ignore_event(&mut self) -> Result<(), BTError<SpiError, GpioError>> {
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
                        // uprintln!(uart, "bt restarted, reason = {:?}", reason);
                    }
                    Event::CommandComplete(params) => {
                        // Self::handle_event_command_complete(uart, params.return_params);
                    }
                    // Event::LeConnectionComplete(conn) => {
                    //     unimplemented!()
                    // }
                    ev => {
                        // uprintln!(uart, "unhandled event = {:?}", ev);
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
                        // uprintln!(uart, "error 0 = {:?}", e);
                    }
                    bluetooth_hci::host::uart::Error::BadPacketType(e) => {
                        // uprintln!(uart, "error 1 = {:?}", e);
                    }
                    bluetooth_hci::host::uart::Error::BLE(e) => {
                        // uprintln!(uart, "error 2 = {:?}", e);
                    }
                }
            }
        }

        Ok(())
    }

    pub fn read_event_uart(
        &mut self,
        // uart: &mut UART,
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
                        rprintln!(
                            "bt restarted, reason = {:?}",
                            defmt::Debug2Format(&reason)
                        );
                    }
                    Event::CommandComplete(params) => {
                        Self::handle_event_command_complete(params.return_params);
                    }
                    // Event::LeConnectionComplete(conn) => {
                    //     unimplemented!()
                    // }
                    ev => {
                        rprintln!("unhandled event = {:?}", defmt::Debug2Format(&ev));
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
                        rprintln!("error 0 = {:?}", defmt::Debug2Format(&e));
                    }
                    bluetooth_hci::host::uart::Error::BadPacketType(e) => {
                        rprintln!("error 1 = {:?}", defmt::Debug2Format(&e));
                    }
                    bluetooth_hci::host::uart::Error::BLE(e) => {
                        rprintln!("error 2 = {:?}", defmt::Debug2Format(&e));
                    }
                }
            }
        }

        Ok(())
    }
}

/// Specific events
impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    fn handle_event_command_complete(
        // uart: &mut UART,
        return_params: ReturnParameters<BlueNRGEvent>,
    ) {
        match return_params {
            ReturnParameters::Vendor(VReturnParameters::GattInit(status)) => match status
            {
                _ => {
                    rprintln!("status = {:?}", defmt::Debug2Format(&status));
                }
            },
            ReturnParameters::ReadLocalVersionInformation(v) => {
                rprintln!("v = {:?}", defmt::Debug2Format(&v));
            }
            ps => {
                rprintln!("Other: return_params = {:?}", defmt::Debug2Format(&ps));
            }
        }
    }

    pub fn read_event_params(
        &mut self,
        // timeout: Option<fugit::MillisDurationU32>,
        // uart: &mut UART,
    ) -> Result<
        bluetooth_hci::event::command::ReturnParameters<BlueNRGEvent>,
        BTError<SpiError, GpioError>,
    > {
        match self._read_event()? {
            Event::CommandComplete(params) => Ok(params.return_params),
            _ => unimplemented!(),
        }
    }

    pub fn read_event_params_vendor(
        &mut self,
        // timeout: Option<fugit::MillisDurationU32>,
        // uart: &mut UART,
    ) -> Result<
        crate::bluetooth::ev_command::ReturnParameters,
        BTError<SpiError, GpioError>,
    > {
        match self._read_event()? {
            Event::CommandComplete(params) => match params.return_params {
                ReturnParameters::Vendor(vs) => Ok(vs),
                other => {
                    panic!("event_params_vendor other 0 = {:?}", other);
                }
            },
            other => {
                panic!("event_params_vendor other 1 = {:?}", other);
            }
        }
    }

    pub fn read_event_gap_init(
        &mut self,
        // uart: &mut UART,
    ) -> Result<GapInit, BTError<SpiError, GpioError>> {
        match self.read_event_params()? {
            ReturnParameters::Vendor(VReturnParameters::GapInit(g)) => return Ok(g),
            _ => unimplemented!(),
        }

        // match self._read_event(uart)? {
        //     Event::CommandComplete(params) => match params.return_params {
        //         ReturnParameters::Vendor(VReturnParameters::GapInit(g)) => return Ok(g),
        //         _ => unimplemented!(),
        //     },
        //     _ => unimplemented!(),
        // }
    }
}

// pub fn handle_event<'buf, SPI, CS, Reset, Input, F, T, GpioError>(
//     bt: &mut BluetoothSpi<'buf, SPI, CS, Reset, Input>,
//     uart: &mut UART,
//     f: F,
// ) -> Option<T>
// where
//     F: FnOnce(&BTEvent) -> Option<T>,
//     SPI: hal::blocking::spi::Transfer<u8, Error = SpiError>
//         + hal::blocking::spi::Write<u8, Error = SpiError>,
//     CS: OutputPin<Error = GpioError>,
//     Reset: OutputPin<Error = GpioError>,
//     Input: InputPin<Error = GpioError>,
//     GpioError: core::fmt::Debug,
// {
//     match block!(bt.read()) {
//         // Ok(Packet::Event(event)) => f(&event),
//         Ok(bluetooth_hci::host::uart::Packet::Event(event)) => f(&event),
//         Err(e) => {
//             uprintln!(uart, "err = {:?}", e);
//             None
//         }
//     }
// }

#[derive(Debug)]
pub enum BTState {
    Disconnected,
    // Connected(ConnectionHandle, BdAddrType, FixedConnectionInterval),
    Connected(ConnectionHandle),
}

impl BTState {
    pub fn is_connected(&self) -> bool {
        match self {
            Self::Connected(_) => true,
            _ => false,
        }
    }

    // fn new_conn(self, st: bluetooth_hci::event::ConnectionComplete<_>) -> Self {
    //     Self::Connected(st.conn_handle)
    // }

    pub fn handle_event(&mut self, event: BTEvent) {
        match event {
            Event::LeConnectionComplete(status) => {
                if status.status == bluetooth_hci::Status::Success {
                    rprintln!("new connection established 0");
                    *self = Self::Connected(status.conn_handle);
                } else {
                    rprintln!(
                        "Connection Complete, but status = {:?}",
                        defmt::Debug2Format(&status.status)
                    );
                }
            }
            Event::ConnectionComplete(status) => {
                if status.status == bluetooth_hci::Status::Success {
                    rprintln!("new connection established 1");
                    *self = Self::Connected(status.conn_handle);
                } else {
                    rprintln!(
                        "Connection Complete, but status = {:?}",
                        defmt::Debug2Format(&status.status)
                    );
                }
            }
            Event::Vendor(crate::bluetooth::events::BlueNRGEvent::HalInitialized(
                reason,
            )) => {
                rprintln!("bt restarted, reason = {:?}", defmt::Debug2Format(&reason));
            }
            Event::CommandComplete(params) => {
                rprintln!("command complete: {:?}", defmt::Debug2Format(&params));
            }
            _ => {
                rprintln!("unhandled event = {:?}", defmt::Debug2Format(&event));
            }
        }
    }
}

pub fn gen_bd_addr() -> BdAddr {
    let device_id = stm32f4xx_hal::signature::Uid::get();
    let waf_num = device_id.waf_num();
    let lot_num = device_id.lot_num().as_bytes();
    /// 51:0B:30:0B:28:C0 on test board
    let addr = bluetooth_hci::BdAddr([
        (lot_num[0].overflowing_shr(24).0) & 0xFF,
        (waf_num) & 0xFF,
        (lot_num[2].overflowing_shr(8).0) & 0xFF,
        (waf_num.overflowing_shr(16).0) & 0xFF,
        ((((0x34 - 48) * 10) + (0x30 - 48)) & 0xFF),
        0xC0,
    ]);
    addr
}
