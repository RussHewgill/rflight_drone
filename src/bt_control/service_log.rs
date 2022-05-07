use bluetooth_hci::{event::command::ReturnParameters, Event};
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

use super::TimeoutResult;

#[derive(Debug, Clone, Copy)]
pub struct SvLogger {
    // service_handle: ServiceHandle,
    // char_handle:    CharacteristicHandle,
    // XXX:
    pub service_handle: ServiceHandle,
    pub char_handle:    CharacteristicHandle,
}

// impl<'buf, SPI, CS, Reset, Input, GpioError> BluetoothSpi<'buf, SPI, CS, Reset, Input>
impl<SPI, CS, Reset, Input, GpioError> BluetoothSpi<SPI, CS, Reset, Input>
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
        timeout: bool,
        data: &[u8],
    ) -> nb::Result<bool, BTError<SpiError, GpioError>> {
        // if !self.state.is_connected() {
        //     uprintln!(uart, "not connected");
        //     return Ok(true);
        // }

        let logger = if let Some(logger) = self.services.logger {
            logger
        } else {
            panic!("no logger?");
        };

        rprintln!("log_write 0");

        let val = UpdateCharacteristicValueParameters {
            service_handle:        logger.service_handle,
            characteristic_handle: logger.char_handle,
            offset:                0,
            value:                 &data,
        };
        block!(self.update_characteristic_value(&val)).unwrap();

        rprintln!("log_write 1");

        if timeout {
            let timeout_duration = 1000.millis();

            // uprintln!(uart, "wat 0");
            let result = self.ignore_event_timeout(timeout_duration)?;
            // uprintln!(uart, "wat 1");

            if result == TimeoutResult::Timeout {
                Ok(false)
            } else {
                Ok(true)
            }
        } else {
            rprintln!("log_write 2");
            self.ignore_event()?;
            rprintln!("log_write 3");
            Ok(true)
        }
    }

    // #[cfg(feature = "nope")]
    /// block
    pub fn init_log_service(
        &mut self,
        // uart: &mut UART,
    ) -> Result<(), BTError<SpiError, GpioError>> {
        let params = AddServiceParameters {
            uuid:                  UUID_CONSOLE_LOG_SERVICE,
            service_type:          crate::bluetooth::gatt::ServiceType::Primary,
            max_attribute_records: 8,
        };
        block!(self.add_service(&params))?;
        rprintln!("sent service");

        // let service: GattService = match self.read_event_params_vendor(uart)? {
        //     VReturnParameters::GattAddService(service) => service,
        //     other => {
        //         panic!("other 0 = {:?}", other);
        //     }
        // };
        // rprintln!("service = {:?}", service);

        // while !self.data_ready().unwrap() {
        //     rprintln!("data not ready");
        // }

        let service = match self._read_event()? {
            Event::CommandComplete(params) => match params.return_params {
                ReturnParameters::Vendor(vs) => match vs {
                    VReturnParameters::GattAddService(service) => service,
                    other => {
                        panic!("event_params_vendor other 0 = {:?}", other);
                    }
                },
                other => {
                    panic!("event_params_vendor other 1 = {:?}", other);
                }
            },
            other => {
                panic!("event_params_vendor other 2 = {:?}", other);
            }
        };
        rprintln!("service = {:?}", defmt::Debug2Format(&service));

        let params0 = AddCharacteristicParameters {
            service_handle:            service.service_handle,
            characteristic_uuid:       UUID_CONSOLE_LOG_CHAR,
            characteristic_value_len:  18,
            characteristic_properties: CharacteristicProperty::NOTIFY,
            security_permissions:      CharacteristicPermission::NONE,
            gatt_event_mask:           CharacteristicEvent::NONE,
            encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
            is_variable:               true,
            fw_version_before_v72:     false,
        };

        // self.wait_ms(50.millis());
        block!(self.add_characteristic(&params0))?;
        rprintln!("sent c");

        let c = match self.read_event_params_vendor()? {
            VReturnParameters::GattAddCharacteristic(c) => c,
            other => unimplemented!("other = {:?}", other),
        };

        rprintln!("c = {:?}", defmt::Debug2Format(&c));

        let logger = SvLogger {
            service_handle: service.service_handle,
            char_handle:    c.characteristic_handle,
        };

        self.services.logger = Some(logger);

        Ok(())
    }

    #[cfg(feature = "nope")]
    /// Retry
    pub fn init_log_service(
        &mut self,
        uart: &mut UART,
    ) -> Result<(), BTError<SpiError, GpioError>> {
        self.wait_ms(25.millis());

        /// how long to wait before re-sending command
        let timeout = 1000.millis();

        let params = AddServiceParameters {
            uuid:                  UUID_CONSOLE_LOG_SERVICE,
            service_type:          crate::bluetooth::gatt::ServiceType::Primary,
            max_attribute_records: 8,
        };

        let service: GattService = loop {
            block!(self.add_service(&params))?;
            match self._read_event_timeout(timeout, uart) {
                Ok(Some(e)) => match e {
                    Event::CommandComplete(params) => match params.return_params {
                        ReturnParameters::Vendor(vs) => match vs {
                            VReturnParameters::GattAddService(service) => {
                                break service;
                            }
                            other => {
                                uprintln!(uart, "other a 0 = {:?}", other);
                            }
                        },
                        other => {
                            uprintln!(uart, "other a 1 = {:?}", other);
                        }
                    },
                    other => {
                        uprintln!(uart, "other a 2 = {:?}", other);
                    }
                },
                Ok(None) => {
                    uprintln!(uart, "init_log_service, re-sending add_service");
                    block!(self.add_service(&params))?;
                }
                Err(e) => panic!("init_log_service error = {:?}", e),
            }
        };
        uprintln!(uart, "service = {:?}", service);

        self.wait_ms(100.millis());

        let params0 = AddCharacteristicParameters {
            service_handle:            service.service_handle,
            characteristic_uuid:       UUID_CONSOLE_LOG_CHAR,
            characteristic_value_len:  18,
            characteristic_properties: CharacteristicProperty::NOTIFY,
            // characteristic_properties: CharacteristicProperty::NOTIFY
            // | CharacteristicProperty::READ,
            security_permissions:      CharacteristicPermission::NONE,
            gatt_event_mask:           CharacteristicEvent::NONE,
            encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
            is_variable:               true,
            // fw_version_before_v72:     true,
            fw_version_before_v72:     false,
        };

        let c = loop {
            block!(self.add_characteristic(&params0))?;
            match self._read_event_timeout(timeout, uart) {
                Ok(Some(e)) => match e {
                    Event::CommandComplete(params) => match params.return_params {
                        ReturnParameters::Vendor(vs) => match vs {
                            VReturnParameters::GattAddCharacteristic(service) => {
                                break service;
                            }
                            other => {
                                uprintln!(uart, "other b 0 = {:?}", other);
                            }
                        },
                        other => {
                            uprintln!(uart, "other b 1 = {:?}", other);
                        }
                    },
                    other => {
                        uprintln!(uart, "other b 2 = {:?}", other);
                    }
                },
                Ok(None) => {
                    uprintln!(uart, "init_log_service, re-sending add_characteristic");
                    block!(self.add_characteristic(&params0))?;
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
