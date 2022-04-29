use bluetooth_hci::{event::command::ReturnParameters, Event};
use embedded_hal as hal;
use hal::digital::v2::{InputPin, OutputPin};

use stm32f4::stm32f401::{RCC, SPI1, TIM2};
use stm32f4xx_hal::{
    block,
    gpio::{Alternate, Input, Output, Pin, PA4, PA5, PA6, PA7, PB0, PB2},
    nb,
    prelude::*,
    spi::{Error as SpiError, NoMiso, Spi1},
    timer::{CounterMs, DelayMs, SysDelay},
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
        delay: bool,
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

        if delay {
            self.ignore_event_timeout(100.millis())?;
        } else {
            self.ignore_event()?;
        }

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

        self.read_event_uart(uart)?;

        Ok(())
    }

    pub fn init_log_service(
        &mut self,
        uart: &mut UART,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        self.wait_ms(25.millis());

        let params = AddServiceParameters {
            uuid:                  UUID_CONSOLE_LOG_SERVICE,
            service_type:          crate::bluetooth::gatt::ServiceType::Primary,
            max_attribute_records: 8,
        };

        // block!(self.add_service(&params))?;
        // self.wait_ms(25.millis());
        // // uprintln!(uart, "wat 0");
        // let service: GattService = match self.read_event_params_vendor(uart)? {
        //     VReturnParameters::GattAddService(service) => service,
        //     other => {
        //         panic!("other 0 = {:?}", other);
        //     }
        // };
        // uprintln!(uart, "service = {:?}", service);

        let service: GattService = loop {
            block!(self.add_service(&params))?;

            // let k = self.data_ready().unwrap();
            // uprintln!(uart, "0 = {:?}", k);
            // let k = self.data_ready().unwrap();
            // uprintln!(uart, "1 = {:?}", k);
            // let k = self.data_ready().unwrap();
            // uprintln!(uart, "2 = {:?}", k);
            // let k = self.data_ready().unwrap();
            // uprintln!(uart, "3 = {:?}", k);

            match self._read_event_timeout(25.millis(), uart) {
                // match self._read_event(uart) {
                Ok(Some(e)) => match e {
                    // Ok(e) => match e {
                    Event::CommandComplete(params) => match params.return_params {
                        ReturnParameters::Vendor(vs) => match vs {
                            VReturnParameters::GattAddService(service) => break service,
                            other => {
                                uprintln!(uart, "other 0 = {:?}", other);
                            }
                        },
                        other => {
                            uprintln!(uart, "other 1 = {:?}", other);
                        }
                    },
                    other => {
                        uprintln!(uart, "other 2 = {:?}", other);
                    }
                },
                Ok(None) => {
                    block!(self.add_service(&params))?;
                }
                Err(e) => panic!("init_log_service error = {:?}", e),
            }
        };
        uprintln!(uart, "service = {:?}", service);

        self.wait_ms(10.millis());

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
            // fw_version_before_v72:     true,
            fw_version_before_v72:     false,
        };
        // block!(self.add_characteristic(&params0))?;

        // let c = match self.read_event_params_vendor(uart)? {
        //     VReturnParameters::GattAddCharacteristic(c) => c,
        //     other => unimplemented!("other = {:?}", other),
        // };
        // uprintln!(uart, "c = {:?}", c);

        let c = loop {
            block!(self.add_characteristic(&params0))?;
            match self._read_event_timeout(25.millis(), uart) {
                Ok(Some(e)) => match e {
                    Event::CommandComplete(params) => match params.return_params {
                        ReturnParameters::Vendor(vs) => match vs {
                            VReturnParameters::GattAddCharacteristic(service) => {
                                break service
                            }
                            other => {
                                uprintln!(uart, "other 0 = {:?}", other);
                            }
                        },
                        other => {
                            uprintln!(uart, "other 1 = {:?}", other);
                        }
                    },
                    other => {
                        uprintln!(uart, "other 2 = {:?}", other);
                    }
                },
                Ok(None) => {
                    block!(self.add_service(&params))?;
                }
                Err(e) => panic!("init_log_service error = {:?}", e),
            }
        };
        uprintln!(uart, "c = {:?}", c);

        let logger = SvLogger {
            service_handle: service.service_handle,
            char_handle:    c.characteristic_handle,
        };

        self.services.logger = Some(logger);

        // self.log_write_uart(uart, "wat".as_bytes()).unwrap();

        Ok(())
    }
}
