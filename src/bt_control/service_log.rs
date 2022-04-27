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

use crate::bluetooth::{ev_command::GattService, gap::Commands as GapCommands};
use crate::bluetooth::{
    gatt::{CharacteristicValue, UpdateCharacteristicValueParameters},
    hal_bt::Commands as HalCommands,
};
use crate::{
    bluetooth::gatt::{
        AddCharacteristicParameters, CharacteristicEvent, CharacteristicPermission,
        CharacteristicProperty, Commands as GattCommands, EncryptionKeySize,
    },
    bt_control::UUID_CONSOLE_LOG_CHAR,
    uprint, uprintln,
};

use crate::{
    bluetooth::{
        ev_command::ReturnParameters as VReturnParameters,
        gatt::{AddServiceParameters, CharacteristicHandle, ServiceHandle},
        BTError, BluetoothSpi,
    },
    bt_control::UUID_CONSOLE_LOG_SERVICE,
    uart::*,
};

#[derive(Debug, Clone, Copy)]
pub struct SvLogger {
    // service_handle: ServiceHandle,
    // char_handle:    CharacteristicHandle,
    // XXX:
    pub service_handle: ServiceHandle,
    pub char_handle:    CharacteristicHandle,
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
    pub fn log_write(
        &mut self,
        data: &[u8],
    ) -> nb::Result<bool, BTError<SpiError, GpioError>> {
        let logger = if let Some(logger) = self.services.logger {
            logger
        } else {
            // uprintln!(uart, "no logger?");
            // uprintln!(uart, "");
            return Ok(false);
        };

        let val = UpdateCharacteristicValueParameters {
            service_handle:        logger.service_handle,
            characteristic_handle: logger.char_handle,
            offset:                0,
            value:                 &data,
        };
        block!(self.update_characteristic_value(&val)).unwrap();

        block!(self.ignore_event())?;

        Ok(true)
    }

    pub fn log_write_uart(
        &mut self,
        uart: &mut UART,
        data: &[u8],
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        let logger = if let Some(logger) = self.services.logger {
            logger
        } else {
            uprintln!(uart, "no logger?");
            // uprintln!(uart, "");
            return Ok(());
        };

        let val = UpdateCharacteristicValueParameters {
            service_handle:        logger.service_handle,
            characteristic_handle: logger.char_handle,
            offset:                0,
            value:                 &data,
        };
        block!(self.update_characteristic_value(&val)).unwrap();

        // let val = CharacteristicValue {
        //     service_handle: console_service.0,
        //     characteristic_handle: console_service.1,
        //     offset: 0,
        //     value: &data,
        // };
        // block!(self.write_characteristic_value(&val)).unwrap();

        block!(self.read_event_uart(uart))?;

        Ok(())
    }

    pub fn init_console_log_service(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        let params = AddServiceParameters {
            uuid:                  UUID_CONSOLE_LOG_SERVICE,
            service_type:          crate::bluetooth::gatt::ServiceType::Primary,
            max_attribute_records: 8,
        };
        block!(self.add_service(&params))?;

        let service: GattService = match self.read_event_params_vendor(uart)? {
            VReturnParameters::GattAddService(service) => service,
            _ => unimplemented!(),
        };
        uprintln!(uart, "service = {:?}", service);

        let params0 = AddCharacteristicParameters {
            service_handle:            service.service_handle,
            characteristic_uuid:       UUID_CONSOLE_LOG_CHAR,
            characteristic_value_len:  18,
            characteristic_properties: CharacteristicProperty::NOTIFY,
            // characteristic_properties: CharacteristicProperty::NOTIFY
            // | CharacteristicProperty::READ,
            security_permissions:      CharacteristicPermission::NONE,
            gatt_event_mask:           CharacteristicEvent::NONE,
            // gatt_event_mask: CharacteristicEvent::CONFIRM_READ,
            encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
            is_variable:               true,
            fw_version_before_v72:     true,
            // fw_version_before_v72:     false,
        };
        block!(self.add_characteristic(&params0))?;

        // let params1 = AddCharacteristicParameters {
        //     service_handle: service.service_handle,
        //     characteristic_uuid: UUID_CONSOLE_LOG_CHAR_WRITE,
        //     characteristic_value_len: 18,
        //     characteristic_properties: CharacteristicProperty::WRITE
        //         | CharacteristicProperty::WRITE_WITHOUT_RESPONSE,
        //     security_permissions: CharacteristicPermission::NONE,
        //     gatt_event_mask: CharacteristicEvent::NONE,
        //     encryption_key_size: EncryptionKeySize::with_value(7).unwrap(),
        //     is_variable: true,
        //     fw_version_before_v72: true,
        // };
        // block!(self.add_characteristic(&params1))?;

        // block!(self.read_event(uart))?;

        let c = match self.read_event_params_vendor(uart)? {
            VReturnParameters::GattAddCharacteristic(c) => c,
            _ => unimplemented!(),
        };

        uprintln!(uart, "c = {:?}", c);

        let logger = SvLogger {
            service_handle: service.service_handle,
            char_handle:    c.characteristic_handle,
        };

        self.services.logger = Some(logger);

        self.log_write_uart(uart, "wat".as_bytes()).unwrap();

        Ok(())
    }
}
