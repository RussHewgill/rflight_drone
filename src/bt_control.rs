pub mod service_input;
pub mod service_log;
// pub mod service_sensors;

use embedded_hal as hal;
use hal::digital::v2::{InputPin, OutputPin};

// use rtt_target::rprintln;
use defmt::{println as rprintln, Format};

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

use bluetooth_hci_defmt::{
    host::Hci,
    host::{
        uart::{CommandHeader, Hci as HciUart},
        AdvertisingInterval, AdvertisingParameters,
    },
    types::FixedConnectionInterval,
    BdAddr, BdAddrType, ConnectionHandle,
};

use crate::bluetooth::{
    ev_command::{GapInit, GattService},
    gap::{AuthenticationRequirements, DiscoverableParameters, LocalName},
    gatt::{
        AddCharacteristicParameters, AddServiceParameters, CharacteristicEvent,
        CharacteristicHandle, CharacteristicPermission, CharacteristicProperty,
        CharacteristicValue, EncryptionKeySize, ServiceHandle,
        UpdateCharacteristicValueParameters,
    },
    hal_bt::ConfigData,
};

use crate::bluetooth::gap::Commands as GapCommands;
use crate::bluetooth::gatt::Commands as GattCommands;
use crate::bluetooth::hal_bt::Commands as HalCommands;

use crate::bluetooth::gap;
use crate::bluetooth::gatt;
use crate::bluetooth::hal_bt;

use bluetooth_hci_defmt::event::command::ReturnParameters;
use bluetooth_hci_defmt::event::{Event, VendorEvent};

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

pub type BTEvent = bluetooth_hci_defmt::event::Event<BlueNRGEvent>;

mod uuids {
    pub const fn uuid_from_hex(uuid: u128) -> crate::bluetooth::gatt::Uuid {
        let x = uuid.to_le_bytes();
        crate::bluetooth::gatt::Uuid::Uuid128(x)
    }

    pub const UUID_TEST0: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x5724d4c76ed5402cbf0d3765bf4b000d);
    pub const UUID_TEST1: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x5724d4c76ed5402cbf0d3765bf4b000e);

    /// Logger
    pub const UUID_LOG_SERVICE: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x3f44d56a86074db0945b6c285b73d48a);

    pub const UUID_LOG_QUAT_CHAR: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x1450781d919c49f0a16c0ec28dfb83d5);

    pub const UUID_LOG_SENS_CHAR: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x5724d4c76ed5402cbf0d3765bf4b9c5c);

    pub const UUID_LOG_PID_CHAR: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x954ea4cad97841018a6ab34dd34df896);

    // /// Sensors
    // pub const UUID_SENSOR_SERVICE: crate::bluetooth::gatt::Uuid =
    //     uuid_from_hex(0x639d0157e75a4eb9835298b676f51912);

    // pub const UUID_SENSOR_CHAR: crate::bluetooth::gatt::Uuid =
    //     uuid_from_hex(0x4f1f7252db544d4faa8f208d48637b3f);

    /// Input
    pub const UUID_INPUT_SERVICE: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0xa7a2eafd569442debf3bee79d621779c);

    pub const UUID_INPUT_CHAR: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x0d2d404bff6d4e8a97b15bc440288423);

    pub const UUID_INPUT_PID_CFG_CHAR: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0xd0187a4d3bc640818a09c890aa6bfdb2);

    // pub const UUID_INPUT_DESC_THROTTLE: crate::bluetooth::gatt::Uuid =
    //     uuid_from_hex(0x7e6c30a33bb241ef838e15912c120dd0);
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
        self.init_input_service()?;

        // self.init_sensor_service()?;
        Ok(())
    }

    pub fn init_bt(&mut self) -> nb::Result<(), BTError<SpiError, GpioError>> {
        self.reset().unwrap();
        self.read_event_uart()?;

        // block!(self.get_firmware_revision())?;
        // self.read_event_uart()?;

        self.reset().unwrap();
        self.read_event_uart()?;

        let addr = gen_bd_addr();

        // block!(self.write_config_data(&ConfigData::public_address(addr).build()))?;
        // block!(self.read_event(uart))?;

        block!(self.le_set_random_address(addr))
            .unwrap_or_else(|_| panic!("set_random_address"));
        self.read_event_uart()?;

        block!(self.init_gatt()).unwrap();
        self.read_event_uart()?;

        let role = crate::bluetooth::gap::Role::PERIPHERAL;
        block!(self.init_gap(role, false, 7))?;
        let gap: GapInit = self.read_event_gap_init()?;

        // /// XXX: clear paired devices
        // block!(self.clear_security_database())?;
        // self.read_event_uart()?;

        static BLE_NAME: &'static str = "DRN1120";

        let ps = UpdateCharacteristicValueParameters {
            service_handle:        gap.service_handle,
            characteristic_handle: gap.dev_name_handle,
            offset:                0,
            value:                 BLE_NAME.as_bytes(),
        };
        block!(self.update_characteristic_value(&ps))
            .unwrap_or_else(|_| panic!("update_characteristic_value"));
        self.read_event_uart()?;

        // block!(self.set_tx_power_level(hal_bt::PowerLevel::DbmNeg2_1))?;
        // self.read_event_uart()?;

        let requirements = AuthenticationRequirements {
            mitm_protection_required:  true,
            // mitm_protection_required:  false,
            out_of_band_auth:          gap::OutOfBandAuthentication::Disabled,
            encryption_key_size_range: (7, 16),
            fixed_pin:                 super::gap::Pin::Fixed(0),
            // fixed_pin:                 super::gap::Pin::Requested,
            bonding_required:          true,
            // bonding_required:          false,
        };
        block!(self.set_authentication_requirement(&requirements))
            .unwrap_or_else(|_| panic!("set_authentication_requirement"));
        self.read_event_uart()?;

        let ad_params = AdvertisingParameters {
            advertising_interval:      AdvertisingInterval::for_type(
                bluetooth_hci_defmt::host::AdvertisingType::ConnectableUndirected,
            )
            .with_range(
                core::time::Duration::from_millis(200),
                core::time::Duration::from_millis(200),
            )
            .unwrap(),
            own_address_type:          bluetooth_hci_defmt::host::OwnAddressType::Random,
            peer_address:              bluetooth_hci_defmt::BdAddrType::Random(addr),
            advertising_channel_map:   bluetooth_hci_defmt::host::Channels::CH_37,
            advertising_filter_policy:
                bluetooth_hci_defmt::host::AdvertisingFilterPolicy::AllowConnectionAndScan,
        };
        block!(self.le_set_advertising_parameters(&ad_params))
            .unwrap_or_else(|_| panic!("set_set_advertising_parameters"));
        self.read_event_uart()?;

        block!(self.le_set_advertising_data(&[]))
            .unwrap_or_else(|_| panic!("set_set_advertising_data"));
        self.read_event_uart()?;

        block!(self.le_set_scan_response_data(&[]))
            .unwrap_or_else(|_| panic!("set_set_scan_response_data"));
        self.read_event_uart()?;

        // uart.unpause();
        block!(self.le_set_advertise_enable(true))?;
        self.read_event_uart()?;
        // uart.pause();

        self.begin_advertise()?;

        let gatt_event_mask = crate::bluetooth::gatt::Event::all();
        block!(self.set_gatt_event_mask(gatt_event_mask))?;
        self.read_event_uart()?;

        let event_mask = bluetooth_hci_defmt::host::EventFlags::all();
        block!(bluetooth_hci_defmt::host::Hci::set_event_mask(
            self, event_mask
        ))?;
        self.read_event_uart()?;

        let gap_event_mask = crate::bluetooth::gap::EventFlags::all();
        block!(self.set_gap_event_mask(gap_event_mask))?;
        self.read_event_uart()?;

        self.init_services()?;

        // block!(self.get_security_level())?;
        // self.read_event_uart()?;

        // block!(self.read_bd_addr()).unwrap();
        // block!(self.read_event(uart))?;

        Ok(())
    }

    pub fn begin_advertise(&mut self) -> nb::Result<(), BTError<SpiError, GpioError>> {
        let dev_name: &'static [u8; 7] = b"DRN1120";

        let d_params = DiscoverableParameters {
            advertising_type:
                bluetooth_hci_defmt::host::AdvertisingType::ConnectableUndirected,
            // advertising_interval: None,
            advertising_interval: Some((
                core::time::Duration::from_millis(200),
                core::time::Duration::from_millis(200),
            )),
            address_type:         bluetooth_hci_defmt::host::OwnAddressType::Random,
            filter_policy:
                bluetooth_hci_defmt::host::AdvertisingFilterPolicy::AllowConnectionAndScan,
            local_name:           Some(LocalName::Complete(&dev_name[..])),
            advertising_data:     &[],
            // conn_interval:        (Some(core::time::Duration::from_millis(5000)), None),
            conn_interval:        (None, None),
        };

        block!(self.set_nondiscoverable())
            .unwrap_or_else(|_| panic!("set_nondiscoverable"));
        self.read_event_uart()?;

        // uart.unpause();
        block!(self.set_discoverable(&d_params))
            .unwrap_or_else(|_| panic!("set_discoverable"));
        self.read_event_uart()?;

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
    #[cfg(feature = "nope")]
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
    #[cfg(feature = "nope")]
    pub fn _read_event_timeout(
        &mut self,
        timeout: fugit::MicrosDurationU32,
        uart: &mut UART,
    ) -> Result<
        Option<bluetooth_hci_defmt::Event<BlueNRGEvent>>,
        BTError<SpiError, GpioError>,
    > {
        use stm32f4xx_hal::timer::Event as TimerEvent;

        self.delay.clear_interrupt(TimerEvent::Update);
        self.delay.start(timeout).unwrap();
        loop {
            match self.read() {
                Ok(p) => {
                    self.delay.cancel().unwrap();
                    self.delay.clear_interrupt(TimerEvent::Update);
                    let bluetooth_hci_defmt::host::uart::Packet::Event(e) = p;
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
                        bluetooth_hci_defmt::host::uart::Error::Comm(e) => {
                            uprintln!(uart, "error 0 = {:?}", e);
                        }
                        bluetooth_hci_defmt::host::uart::Error::BadPacketType(e) => {
                            uprintln!(uart, "error 1 = {:?}", e);
                        }
                        bluetooth_hci_defmt::host::uart::Error::BLE(e) => {
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
    ) -> Result<bluetooth_hci_defmt::Event<BlueNRGEvent>, BTError<SpiError, GpioError>>
    {
        //

        let x: Result<
            bluetooth_hci_defmt::host::uart::Packet<BlueNRGEvent>,
            bluetooth_hci_defmt::host::uart::Error<
                BTError<SpiError, GpioError>,
                crate::bluetooth::events::BlueNRGError,
            >,
        > = block!(self.read());

        // let x: Result<
        //     bluetooth_hci_defmt::host::uart::Packet<BlueNRGEvent>,
        //     bluetooth_hci_defmt::host::uart::Error<
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
                let bluetooth_hci_defmt::host::uart::Packet::Event(e) = p;
                return Ok(e);
            }
            Err(e) => {
                let e: bluetooth_hci_defmt::host::uart::Error<
                    BTError<SpiError, GpioError>,
                    crate::bluetooth::events::BlueNRGError,
                > = e;
                match e {
                    bluetooth_hci_defmt::host::uart::Error::Comm(e) => {
                        rprintln!("error 0 = {:?}", e);
                        // rprintln!("error 0 = {:?}", e);
                    }
                    bluetooth_hci_defmt::host::uart::Error::BadPacketType(e) => {
                        rprintln!("error 1 = {:?}", e);
                    }
                    bluetooth_hci_defmt::host::uart::Error::BLE(e) => {
                        rprintln!("error 2 = {:?}", e);
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
        use bluetooth_hci_defmt::Controller;
        use stm32f4xx_hal::timer::Event as TimerEvent;

        let x: Result<
            bluetooth_hci_defmt::host::uart::Packet<BlueNRGEvent>,
            bluetooth_hci_defmt::host::uart::Error<
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
                        // panic!("error 1 = {:?}", e);
                        // rprintln!("e = {:?}", e);
                        panic!("error 1");
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
                let bluetooth_hci_defmt::host::uart::Packet::Event(e) = p;
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
                // rprintln!("bt restarted, reason = {:?}", reason);
                rprintln!("bt restarted, reason = {:?}", reason);
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
                        if status != bluetooth_hci_defmt::Status::Success {
                            rprintln!("status 0 = {:?}", status);
                        } else {
                            // rprintln!("status 1 = {:?}", status);
                        }
                    }
                    ReturnParameters::Vendor(
                        VReturnParameters::GattUpdateLongCharacteristicValue(status),
                    ) => {
                        if status != bluetooth_hci_defmt::Status::Success {
                            rprintln!("status 0 = {:?}", status);
                        } else {
                            // rprintln!("status 1 = {:?}", status);
                        }
                    }
                    _ => {
                        rprintln!("return_params = {:?}", params.return_params);
                    }
                }
            }
            // Event::LeConnectionComplete(conn) => {
            //     unimplemented!()
            // }
            ev => {
                rprintln!("unhandled event = {:?}", ev);
            }
        }

        Ok(TimeoutResult::Ok)
    }

    #[cfg(feature = "nope")]
    pub fn ignore_event(&mut self) -> Result<(), BTError<SpiError, GpioError>> {
        let x: Result<
            bluetooth_hci_defmt::host::uart::Packet<BlueNRGEvent>,
            bluetooth_hci_defmt::host::uart::Error<
                BTError<SpiError, GpioError>,
                crate::bluetooth::events::BlueNRGError,
            >,
        > = block!(self.read());

        match x {
            Ok(p) => {
                let bluetooth_hci_defmt::host::uart::Packet::Event(e) = p;
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
                let e: bluetooth_hci_defmt::host::uart::Error<
                    BTError<SpiError, GpioError>,
                    crate::bluetooth::events::BlueNRGError,
                > = e;
                match e {
                    bluetooth_hci_defmt::host::uart::Error::Comm(e) => {
                        // uprintln!(uart, "error 0 = {:?}", e);
                    }
                    bluetooth_hci_defmt::host::uart::Error::BadPacketType(e) => {
                        // uprintln!(uart, "error 1 = {:?}", e);
                    }
                    bluetooth_hci_defmt::host::uart::Error::BLE(e) => {
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
            bluetooth_hci_defmt::host::uart::Packet<BlueNRGEvent>,
            bluetooth_hci_defmt::host::uart::Error<
                BTError<SpiError, GpioError>,
                crate::bluetooth::events::BlueNRGError,
            >,
        > = block!(self.read());

        match x {
            Ok(p) => {
                let bluetooth_hci_defmt::host::uart::Packet::Event(e) = p;
                // uprintln!(uart, "event = {:?}", &e);
                match e {
                    Event::ConnectionComplete(params) => {
                        // handle the new connection
                    }
                    Event::Vendor(
                        crate::bluetooth::events::BlueNRGEvent::HalInitialized(reason),
                    ) => {
                        rprintln!("bt restarted, reason = {:?}", reason);
                    }
                    Event::CommandComplete(params) => {
                        Self::handle_event_command_complete(params.return_params);
                    }
                    // Event::LeConnectionComplete(conn) => {
                    //     unimplemented!()
                    // }
                    ev => {
                        rprintln!("unhandled event = {:?}", ev);
                    }
                }
            }

            Err(e) => {
                let e: bluetooth_hci_defmt::host::uart::Error<
                    BTError<SpiError, GpioError>,
                    crate::bluetooth::events::BlueNRGError,
                > = e;
                match e {
                    bluetooth_hci_defmt::host::uart::Error::Comm(e) => {
                        rprintln!("error 0 = {:?}", e);
                    }
                    bluetooth_hci_defmt::host::uart::Error::BadPacketType(e) => {
                        rprintln!("error 1 = {:?}", e);
                    }
                    bluetooth_hci_defmt::host::uart::Error::BLE(e) => {
                        rprintln!("error 2 = {:?}", e);
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
    pub fn allow_read_write(&mut self, event: &BTEvent) {
        match event {
            Event::Vendor(BlueNRGEvent::AttReadPermitRequest(req)) => {
                // unimplemented!()
            }
            _ => {}
        }
    }

    fn handle_event_command_complete(
        // uart: &mut UART,
        return_params: ReturnParameters<BlueNRGEvent>,
    ) {
        match return_params {
            ReturnParameters::Vendor(VReturnParameters::GattInit(status)) => match status
            {
                _ => {
                    rprintln!("status = {:?}", status);
                }
            },
            ReturnParameters::ReadLocalVersionInformation(v) => {
                rprintln!("v = {:?}", v);
            }
            ps => {
                rprintln!("Other: return_params = {:?}", ps);
            }
        }
    }

    pub fn read_event_params(
        &mut self,
        // timeout: Option<fugit::MillisDurationU32>,
        // uart: &mut UART,
    ) -> Result<
        bluetooth_hci_defmt::event::command::ReturnParameters<BlueNRGEvent>,
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
                    // panic!("event_params_vendor other 0 = {:?}", other);
                    panic!("event_params_vendor other");
                }
            },
            other => {
                // panic!("event_params_vendor other 1 = {:?}", other);
                panic!("event_params_vendor other 1");
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
//         Ok(bluetooth_hci_defmt::host::uart::Packet::Event(event)) => f(&event),
//         Err(e) => {
//             uprintln!(uart, "err = {:?}", e);
//             None
//         }
//     }
// }

pub fn gen_bd_addr() -> BdAddr {
    let device_id = stm32f4xx_hal::signature::Uid::get();
    let waf_num = device_id.waf_num();
    let lot_num = device_id.lot_num().as_bytes();
    /// 51:0B:30:0B:28:C0 on test board
    let addr = bluetooth_hci_defmt::BdAddr([
        (lot_num[0].overflowing_shr(24).0) & 0xFF,
        (waf_num) & 0xFF,
        (lot_num[2].overflowing_shr(8).0) & 0xFF,
        (waf_num.overflowing_shr(16).0) & 0xFF,
        ((((0x34 - 48) * 10) + (0x30 - 48)) & 0xFF),
        0xC0,
    ]);
    addr
}
