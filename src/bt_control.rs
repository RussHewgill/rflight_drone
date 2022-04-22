pub mod service_log;
pub mod service_sensors;

use embedded_hal as hal;
use hal::digital::v2::{InputPin, OutputPin};

use stm32f4::stm32f401::{RCC, SPI1, TIM2};
use stm32f4xx_hal::{
    block,
    gpio::{Alternate, Input, Output, Pin, PA4, PA5, PA6, PA7, PB0, PB2},
    nb,
    prelude::*,
    spi::{Error as SpiError, NoMiso, Spi1},
    timer::{DelayMs, SysDelay},
};

use bluetooth_hci::{host::Hci, types::FixedConnectionInterval, BdAddrType, ConnectionHandle};
use bluetooth_hci::{
    host::{
        uart::{CommandHeader, Hci as HciUart},
        AdvertisingInterval, AdvertisingParameters,
    },
    BdAddr,
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
    uprintln,
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

pub type BTController<'spi> = BluetoothSpi<
    'spi,
    Spi1<(
        Pin<'A', 5, Alternate<5>>,
        Pin<'A', 6, Alternate<5>>,
        Pin<'A', 7, Alternate<5>>,
    )>,
    Pin<'B', 0, Output>,
    Pin<'B', 2, Output>,
    PA4,
>;

pub type BTEvent = bluetooth_hci::event::Event<BlueNRGEvent>;

mod uuids {
    pub const fn uuid_from_hex(uuid: u128) -> crate::bluetooth::gatt::Uuid {
        let x = uuid.to_le_bytes();
        crate::bluetooth::gatt::Uuid::Uuid128(x)
    }

    /// Logger
    pub const UUID_CONSOLE_LOG_SERVICE: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x3f44d56a86074db0945b6c285b73d48a);

    pub const UUID_CONSOLE_LOG_CHAR: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x1450781d919c49f0a16c0ec28dfb83d5);

    /// Sensors

    pub const UUID_SENSOR_SERVICE: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x639d0157e75a4eb9835298b676f51912);

    pub const UUID_SENSOR_CHAR: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x4f1f7252db544d4faa8f208d48637b3f);
}

/// init
impl<'buf, SPI, CS, Reset, Input, GpioError> BluetoothSpi<'buf, SPI, CS, Reset, Input>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = SpiError>
        + hal::blocking::spi::Write<u8, Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    fn init_services(&mut self, uart: &mut UART) -> nb::Result<(), BTError<SpiError, GpioError>> {
        self.init_console_log_service(uart)?;
        self.init_sensor_service(uart)?;
        Ok(())
    }

    pub fn init_bt(
        &mut self,
        uart: &mut UART,
        delay: &mut DelayMs<TIM2>,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        self.reset_with_delay(delay, 5u32).unwrap();
        block!(self.read_event(uart))?;

        let addr = gen_bd_addr();

        // block!(self.write_config_data(&ConfigData::public_address(addr).build()))?;
        // block!(self.read_event(uart))?;

        block!(self.le_set_random_address(addr)).unwrap();
        block!(self.read_event(uart))?;

        block!(self.init_gatt()).unwrap();
        block!(self.read_event(uart))?;

        let role = crate::bluetooth::gap::Role::PERIPHERAL;
        block!(self.init_gap(role, false, 7))?;
        let gap: GapInit = block!(self.read_event_gap_init(uart))?;

        // block!(self.clear_security_database())?;
        // block!(self.read_event(uart))?;

        // uprintln!(uart, "gap = {:?}", gap);

        static BLE_NAME: &'static str = "DRN1120";

        let ps = UpdateCharacteristicValueParameters {
            service_handle: gap.service_handle,
            characteristic_handle: gap.dev_name_handle,
            offset: 0,
            value: BLE_NAME.as_bytes(),
        };
        block!(self.update_characteristic_value(&ps)).unwrap();
        block!(self.read_event(uart))?;

        // block!(bt.set_tx_power_level(hal_bt::PowerLevel::DbmNeg2_1)).unwrap();
        // block!(bt.read_event(&mut uart)).unwrap();

        // block!(self.read_bd_addr()).unwrap();
        // block!(self.read_event(uart))?;

        self.init_services(uart)?;

        // let addr = BdAddr([
        //     0x43 | 0b1100_0000,
        //     0xc6,
        //     0x3d,
        //     0xd9,
        //     0x74,
        //     0x4f | 0b1100_0000,
        // ]);

        let requirements = AuthenticationRequirements {
            mitm_protection_required: true,
            out_of_band_auth: gap::OutOfBandAuthentication::Disabled,
            encryption_key_size_range: (7, 16),
            fixed_pin: super::gap::Pin::Fixed(1),
            bonding_required: true,
        };
        block!(self.set_authentication_requirement(&requirements)).unwrap();
        block!(self.read_event(uart))?;

        let ad_params = AdvertisingParameters {
            advertising_interval: AdvertisingInterval::for_type(
                bluetooth_hci::host::AdvertisingType::ConnectableUndirected,
            )
            .with_range(
                core::time::Duration::from_millis(200),
                core::time::Duration::from_millis(200),
            )
            .unwrap(),
            own_address_type: bluetooth_hci::host::OwnAddressType::Random,
            peer_address: bluetooth_hci::BdAddrType::Random(addr),
            advertising_channel_map: bluetooth_hci::host::Channels::CH_37,
            advertising_filter_policy:
                bluetooth_hci::host::AdvertisingFilterPolicy::AllowConnectionAndScan,
        };
        block!(self.le_set_advertising_parameters(&ad_params)).unwrap();
        block!(self.read_event(uart))?;

        block!(self.le_set_advertising_data(&[])).unwrap();
        block!(self.read_event(uart))?;

        block!(self.le_set_scan_response_data(&[])).unwrap();
        block!(self.read_event(uart))?;

        block!(self.le_set_advertise_enable(true))?;
        block!(self.read_event(uart))?;

        let dev_name: &'static [u8; 7] = b"DRN1120";

        let d_params = DiscoverableParameters {
            advertising_type: bluetooth_hci::host::AdvertisingType::ConnectableUndirected,
            advertising_interval: None,
            // advertising_interval: Some((
            //     core::time::Duration::from_millis(200),
            //     core::time::Duration::from_millis(200),
            // )),
            address_type: bluetooth_hci::host::OwnAddressType::Random,
            filter_policy: bluetooth_hci::host::AdvertisingFilterPolicy::AllowConnectionAndScan,
            local_name: Some(LocalName::Complete(&dev_name[..])),
            advertising_data: &[],
            conn_interval: (Some(core::time::Duration::from_millis(5000)), None),
        };

        block!(self.set_nondiscoverable()).unwrap();
        block!(self.read_event(uart))?;

        block!(self.set_discoverable(&d_params)).unwrap();
        block!(self.read_event(uart))?;

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

    // pub fn init_advertising(
    //     &mut self,
    //     uart: &mut UART,
    // ) -> nb::Result<(), BTError<SpiError, GpioError>> {
    //     unimplemented!()
    // }

    //
}

/// read and handle events
impl<'buf, SPI, CS, Reset, Input, GpioError> BluetoothSpi<'buf, SPI, CS, Reset, Input>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = SpiError>
        + hal::blocking::spi::Write<u8, Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    // fn handle_event_new_conn(uart: &mut UART) -> ConnectionHandle {
    //     unimplemented!()
    // }

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

    pub fn read_event_params(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<
        bluetooth_hci::event::command::ReturnParameters<BlueNRGEvent>,
        BTError<SpiError, GpioError>,
    > {
        match self._read_event(uart)? {
            Event::CommandComplete(params) => Ok(params.return_params),
            _ => unimplemented!(),
        }
    }

    pub fn read_event_params_vendor(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<crate::bluetooth::ev_command::ReturnParameters, BTError<SpiError, GpioError>>
    {
        match self._read_event(uart)? {
            Event::CommandComplete(params) => match params.return_params {
                ReturnParameters::Vendor(vs) => Ok(vs),
                _ => unimplemented!(),
            },
            _ => unimplemented!(),
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
                // uprintln!(uart, "event = {:?}", &e);
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

    pub fn read_event_gap_init(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<GapInit, BTError<SpiError, GpioError>> {
        match self.read_event_params(uart)? {
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

    pub fn handle_event<'buf>(&mut self, uart: &mut UART, event: BTEvent) {
        match event {
            Event::LeConnectionComplete(status) => {
                if status.status == bluetooth_hci::Status::Success {
                    uprintln!(uart, "new connection established 0");
                    *self = Self::Connected(status.conn_handle);
                } else {
                    uprintln!(
                        uart,
                        "Connection Complete, but status = {:?}",
                        status.status
                    );
                }
            }
            Event::ConnectionComplete(status) => {
                if status.status == bluetooth_hci::Status::Success {
                    uprintln!(uart, "new connection established 1");
                    *self = Self::Connected(status.conn_handle);
                } else {
                    uprintln!(
                        uart,
                        "Connection Complete, but status = {:?}",
                        status.status
                    );
                }
            }
            Event::Vendor(crate::bluetooth::events::BlueNRGEvent::HalInitialized(reason)) => {
                uprintln!(uart, "bt restarted, reason = {:?}", reason);
            }
            Event::CommandComplete(params) => {
                uprintln!(uart, "command complete: {:?}", params);
            }
            _ => {
                uprintln!(uart, "unhandled event = {:?}", event);
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
