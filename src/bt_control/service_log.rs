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

use crate::{
    bluetooth::gatt::{
        AddCharacteristicParameters, CharacteristicEvent, CharacteristicPermission,
        CharacteristicProperty, CharacteristicValue, Commands as GattCommands,
        EncryptionKeySize, UpdateCharacteristicValueParameters,
    },
    bluetooth::{
        gatt::UpdateLongCharacteristicValueParameters, hal_bt::Commands as HalCommands,
    },
    bt_control::{UUID_LOG_CHAR, UUID_LOG_SENS_CHAR},
    sensors::V3,
    uprint, uprintln,
};
use crate::{
    bluetooth::{ev_command::GattService, gap::Commands as GapCommands},
    sensors::UQuat,
};

use crate::{
    bluetooth::{
        ev_command::ReturnParameters as VReturnParameters,
        gatt::{AddServiceParameters, CharacteristicHandle, ServiceHandle},
        BTError, BluetoothSpi,
    },
    bt_control::UUID_LOG_SERVICE,
    uart::*,
};

use super::TimeoutResult;

#[derive(Debug, Clone, Copy)]
pub struct SvLogger {
    pub service_handle:   ServiceHandle,
    pub char_handle_quat: CharacteristicHandle,
    pub char_handle_sens: CharacteristicHandle,
}

/// write
impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    pub fn log_write_sens(
        &mut self,
        gyro: V3,
        acc: V3,
        mag: V3,
        // baro: f32,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        let logger = if let Some(logger) = self.services.logger {
            logger
        } else {
            panic!("no logger?");
        };

        // let mut data = [0u8; 40];
        let mut data = [0u8; 36];
        // let mut data = [0u8; 24];

        /// gyro
        data[0..4].copy_from_slice(&gyro.x.to_be_bytes());
        data[4..8].copy_from_slice(&gyro.y.to_be_bytes());
        data[8..12].copy_from_slice(&gyro.z.to_be_bytes());
        /// acc
        data[12..16].copy_from_slice(&acc.x.to_be_bytes());
        data[16..20].copy_from_slice(&acc.y.to_be_bytes());
        data[20..24].copy_from_slice(&acc.z.to_be_bytes());
        /// mag
        data[24..28].copy_from_slice(&mag.x.to_be_bytes());
        data[28..32].copy_from_slice(&mag.y.to_be_bytes());
        data[32..36].copy_from_slice(&mag.z.to_be_bytes());

        // data[36..40].copy_from_slice(&baro.to_be_bytes());

        let val = UpdateLongCharacteristicValueParameters {
            service_handle:        logger.service_handle,
            characteristic_handle: logger.char_handle_sens,
            update_type:           crate::bluetooth::gatt::UpdateType::NOTIFICATION,
            total_len:             data.len(),
            offset:                0,
            value:                 &data,
        };
        block!(self.update_long_characteristic_value(&val)).unwrap();

        let result = self.ignore_event_timeout(None)?;

        Ok(())
    }

    pub fn log_write_quat(
        &mut self,
        quat: &UQuat,
    ) -> nb::Result<(), BTError<SpiError, GpioError>> {
        let logger = if let Some(logger) = self.services.logger {
            logger
        } else {
            panic!("no logger?");
        };

        let qq = quat.coords;
        let mut data = [0u8; 16];

        data[0..4].copy_from_slice(&qq[0].to_be_bytes());
        data[4..8].copy_from_slice(&qq[1].to_be_bytes());
        data[8..12].copy_from_slice(&qq[2].to_be_bytes());
        data[12..16].copy_from_slice(&qq[3].to_be_bytes());

        let val = UpdateCharacteristicValueParameters {
            service_handle:        logger.service_handle,
            characteristic_handle: logger.char_handle_quat,
            offset:                0,
            value:                 &data,
        };
        block!(self.update_characteristic_value(&val)).unwrap();

        let result = self.ignore_event_timeout(None)?;

        Ok(())
    }

    #[cfg(feature = "nope")]
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

        // rprintln!("log_write 0");

        let val = UpdateCharacteristicValueParameters {
            service_handle:        logger.service_handle,
            characteristic_handle: logger.char_handle_quat,
            offset:                0,
            value:                 &data,
        };
        block!(self.update_characteristic_value(&val)).unwrap();

        // rprintln!("log_write 1");

        let timeout_duration = if timeout { Some(1000.millis()) } else { None };

        // rprintln!("log_write 2");
        let result = self.ignore_event_timeout(timeout_duration)?;
        // rprintln!("log_write 3");

        if result == TimeoutResult::Timeout {
            // rprintln!("log_write 4");
            Ok(false)
        } else {
            // rprintln!("log_write 5");
            Ok(true)
        }
    }
}

/// init
impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    pub fn init_log_service(&mut self) -> Result<(), BTError<SpiError, GpioError>> {
        let params = AddServiceParameters {
            uuid:                  UUID_LOG_SERVICE,
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
            characteristic_uuid:       UUID_LOG_CHAR,
            characteristic_value_len:  18,
            characteristic_properties: CharacteristicProperty::NOTIFY,
            security_permissions:      CharacteristicPermission::NONE,
            gatt_event_mask:           CharacteristicEvent::NONE,
            encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
            is_variable:               true,
            fw_version_before_v72:     false,
        };

        block!(self.add_characteristic(&params0))?;
        rprintln!("sent c 0");

        let c0 = match self.read_event_params_vendor()? {
            VReturnParameters::GattAddCharacteristic(c) => c,
            other => unimplemented!("other = {:?}", other),
        };
        rprintln!("c 0 = {:?}", defmt::Debug2Format(&c0));

        let params1 = AddCharacteristicParameters {
            service_handle:            service.service_handle,
            characteristic_uuid:       UUID_LOG_SENS_CHAR,
            characteristic_value_len:  48,
            // characteristic_properties: CharacteristicProperty::NOTIFY,
            characteristic_properties: CharacteristicProperty::NOTIFY
                | CharacteristicProperty::READ,
            security_permissions:      CharacteristicPermission::NONE,
            gatt_event_mask:           CharacteristicEvent::NONE,
            encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
            is_variable:               true,
            fw_version_before_v72:     false,
        };
        block!(self.add_characteristic(&params1))?;
        rprintln!("sent c 1");

        let c1 = match self.read_event_params_vendor()? {
            VReturnParameters::GattAddCharacteristic(c) => c,
            other => unimplemented!("other = {:?}", other),
        };
        rprintln!("c 1 = {:?}", defmt::Debug2Format(&c1));

        let logger = SvLogger {
            service_handle:   service.service_handle,
            char_handle_quat: c0.characteristic_handle,
            char_handle_sens: c1.characteristic_handle,
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
            uuid:                  UUID_LOG_SERVICE,
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
            characteristic_uuid:       UUID_LOG_CHAR,
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
