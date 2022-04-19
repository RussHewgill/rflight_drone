use embedded_hal as hal;
use hal::digital::v2::{InputPin, OutputPin};

use stm32f4::stm32f401::{RCC, SPI1};
use stm32f4xx_hal::{
    block, nb,
    prelude::*,
    spi::{Error as SpiError, NoMiso},
    timer::SysDelay,
};

use bluetooth_hci::host::Hci;
use bluetooth_hci::{
    host::{
        uart::{CommandHeader, Hci as HciUart},
        AdvertisingInterval, AdvertisingParameters,
    },
    BdAddr,
};

use crate::{
    bluetooth::{
        ev_command::GapInit,
        gap::{DiscoverableParameters, LocalName},
        gatt::{
            AddCharacteristicParameters, AddServiceParameters, CharacteristicEvent,
            CharacteristicPermission, CharacteristicProperty, EncryptionKeySize,
            UpdateCharacteristicValueParameters,
        },
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

mod uuids {
    pub const fn uuid_from_hex(uuid: u128) -> crate::bluetooth::gatt::Uuid {
        let x = uuid.to_le_bytes();
        crate::bluetooth::gatt::Uuid::Uuid128(x)
    }

    pub const UUID_CONSOLE_LOG_SERVICE: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x3f44d56a86074db0945b6c285b73d48a);

    pub const UUID_CONSOLE_LOG_CHAR: crate::bluetooth::gatt::Uuid =
        uuid_from_hex(0x1450781d919c49f0a16c0ec28dfb83d5);
}

impl<'buf, SPI, CS, Reset, Input, GpioError> BluetoothSpi<'buf, SPI, CS, Reset, Input>
where
    SPI: hal::blocking::spi::Transfer<u8, Error = SpiError>
        + hal::blocking::spi::Write<u8, Error = SpiError>,
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    pub fn init_bt(
        &mut self,
        uart: &mut UART,
        delay: &mut SysDelay,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        self.reset_with_delay(delay, 5u32).unwrap();
        block!(self.read_event(uart))?;

        block!(self.init_gatt()).unwrap();
        block!(self.read_event(uart))?;

        let role = crate::bluetooth::gap::Role::PERIPHERAL;
        block!(self.init_gap(role, false, 7))?;
        let gap: GapInit = block!(self.read_event_gap_init(uart))?;

        // uprintln!(uart, "gap = {:?}", gap);

        static BLE_NAME: &'static [u8; 7] = b"DRN1120";

        let ps = UpdateCharacteristicValueParameters {
            service_handle: gap.service_handle,
            characteristic_handle: gap.dev_name_handle,
            offset: 0,
            value: &BLE_NAME[..],
        };
        block!(self.update_characteristic_value(&ps)).unwrap();
        block!(self.read_event(uart))?;

        // block!(bt.set_tx_power_level(hal_bt::PowerLevel::DbmNeg2_1)).unwrap();
        // block!(bt.read_event(&mut uart)).unwrap();

        // block!(self.read_bd_addr()).unwrap();
        // block!(self.read_event(uart))?;

        let addr = BdAddr([
            0x43 | 0b1100_0000,
            0xc6,
            0x3d,
            0xd9,
            0x74,
            0x4f | 0b1100_0000,
        ]);

        block!(self.le_set_random_address(addr)).unwrap();
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

        block!(self.read_bd_addr()).unwrap();
        block!(self.read_event(uart))?;

        // block!(self.read_event(uart))?;

        // let conn = match self.read_event_params_vendor(uart)? {
        //     VReturnParameters::GattAddService(service) => service,
        //     _ => unimplemented!(),
        // };

        // self.init_console_log_service(uart)?;

        loop {
            block!(self.read_event(uart))?;
            if false {
                break;
            }
        }

        Ok(())
    }

    pub fn init_advertising(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        unimplemented!()
    }

    pub fn init_console_log_service(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        let params = AddServiceParameters {
            uuid: UUID_CONSOLE_LOG_SERVICE,
            service_type: crate::bluetooth::gatt::ServiceType::Primary,
            max_attribute_records: 8,
        };
        block!(self.add_service(&params))?;

        let service = match self.read_event_params_vendor(uart)? {
            VReturnParameters::GattAddService(service) => service,
            _ => unimplemented!(),
        };
        uprintln!(uart, "service = {:?}", service);

        // let params = AddCharacteristicParameters {
        //     service_handle: service.service_handle,
        //     characteristic_uuid: UUID_CONSOLE_LOG_SERVICE,
        //     characteristic_value_len: 1,
        //     characteristic_properties: CharacteristicProperty::READ | CharacteristicProperty::WRITE,
        //     // | CharacteristicProperty::NOTIFY
        //     security_permissions: CharacteristicPermission::NONE,
        //     gatt_event_mask: CharacteristicEvent::NONE,
        //     encryption_key_size: EncryptionKeySize::with_value(8).unwrap(),
        //     is_variable: true,
        //     fw_version_before_v72: false,
        // };
        // block!(self.add_characteristic(&params))?;

        // block!(self.read_event(uart))?;

        let c = match self.read_event_params_vendor(uart)? {
            VReturnParameters::GattAddCharacteristic(c) => c,
            _ => unimplemented!(),
        };

        uprintln!(uart, "c = {:?}", c);

        Ok(())
    }

    //
}

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
