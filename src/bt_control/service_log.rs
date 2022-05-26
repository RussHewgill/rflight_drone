use bluetooth_hci_defmt::{event::command::ReturnParameters, Event};
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
    bt_control::{UUID_LOG_PID_CHAR, UUID_LOG_QUAT_CHAR, UUID_LOG_SENS_CHAR},
    flight_control::IdPID,
    pid::{PIDOutput, PID},
    sensors::V3,
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
};

use super::TimeoutResult;

#[derive(Debug, Clone, Copy)]
pub struct SvLogger {
    pub service_handle:   ServiceHandle,
    pub char_handle_quat: CharacteristicHandle,
    pub char_handle_sens: CharacteristicHandle,
    pub char_handle_pid:  CharacteristicHandle,
}

/// write
impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    fn log_write(
        &mut self,
        char_handle: CharacteristicHandle,
        data: &[u8],
        long: bool,
    ) -> Result<(), BTError<SpiError, GpioError>> {
        let logger = self.services.logger.expect("no logger?");

        if long {
            let val = UpdateLongCharacteristicValueParameters {
                service_handle:        logger.service_handle,
                characteristic_handle: char_handle,
                update_type:           crate::bluetooth::gatt::UpdateType::NOTIFICATION,
                total_len:             data.len(),
                offset:                0,
                value:                 &data,
            };
            block!(self.update_long_characteristic_value(&val)).unwrap();

            let result = self.ignore_event_timeout(None)?;
        } else {
            let val = UpdateCharacteristicValueParameters {
                service_handle:        logger.service_handle,
                characteristic_handle: char_handle,
                offset:                0,
                value:                 &data,
            };
            block!(self.update_characteristic_value(&val)).unwrap();

            let result = self.ignore_event_timeout(None)?;
        }

        Ok(())
    }

    pub fn log_write_pid(
        &mut self,
        pid_id: IdPID,
        pid: &PID,
    ) -> Result<(), BTError<SpiError, GpioError>> {
        let logger = self.services.logger.expect("no logger?");

        let PIDOutput { p, i, d, output } = pid.prev_output;

        let mut data = [0u8; 17];

        data[0..4].copy_from_slice(&output.to_be_bytes());
        data[4..8].copy_from_slice(&p.to_be_bytes());
        data[8..12].copy_from_slice(&i.to_be_bytes());
        data[12..16].copy_from_slice(&d.to_be_bytes());
        data[16] = pid_id as u8;

        self.log_write(logger.char_handle_pid, &data, false)?;

        Ok(())
    }

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

        // let mut data = [0u8; 36];
        // /// gyro
        // data[0..4].copy_from_slice(&gyro.x.to_be_bytes());
        // data[4..8].copy_from_slice(&gyro.y.to_be_bytes());
        // data[8..12].copy_from_slice(&gyro.z.to_be_bytes());
        // /// acc
        // data[12..16].copy_from_slice(&acc.x.to_be_bytes());
        // data[16..20].copy_from_slice(&acc.y.to_be_bytes());
        // data[20..24].copy_from_slice(&acc.z.to_be_bytes());
        // /// mag
        // data[24..28].copy_from_slice(&mag.x.to_be_bytes());
        // data[28..32].copy_from_slice(&mag.y.to_be_bytes());
        // data[32..36].copy_from_slice(&mag.z.to_be_bytes());

        let mut data = [0u8; 12];
        /// mag
        data[0..4].copy_from_slice(&mag.x.to_be_bytes());
        data[4..8].copy_from_slice(&mag.y.to_be_bytes());
        data[8..12].copy_from_slice(&mag.z.to_be_bytes());

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
        const NUM_SERVICES: usize = 3;
        // const NUM_RECORDS: usize = 1 + 3 * NUM_SERVICES;

        let params = AddServiceParameters {
            uuid:                  UUID_LOG_SERVICE,
            service_type:          crate::bluetooth::gatt::ServiceType::Primary,
            max_attribute_records: 1 + 3 * NUM_SERVICES,
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
        rprintln!("service = {:?}", service);

        // let params0 = AddCharacteristicParameters {
        //     service_handle:            service.service_handle,
        //     characteristic_uuid:       UUID_LOG_CHAR,
        //     characteristic_value_len:  18,
        //     characteristic_properties: CharacteristicProperty::NOTIFY,
        //     security_permissions:      CharacteristicPermission::NONE,
        //     gatt_event_mask:           CharacteristicEvent::NONE,
        //     encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
        //     is_variable:               true,
        //     fw_version_before_v72:     false,
        // };
        // block!(self.add_characteristic(&params0))?;
        // rprintln!("sent c 0");

        // let c0 = match self.read_event_params_vendor()? {
        //     VReturnParameters::GattAddCharacteristic(c) => c,
        //     other => unimplemented!("other = {:?}", other),
        // };
        // rprintln!("c 0 = {:?}", c0);

        let handle_quat = self.add_log_char(
            service.service_handle,
            UUID_LOG_QUAT_CHAR,
            18,
            CharacteristicProperty::NOTIFY,
            0,
        )?;

        let handle_sens = self.add_log_char(
            service.service_handle,
            UUID_LOG_SENS_CHAR,
            18,
            CharacteristicProperty::NOTIFY | CharacteristicProperty::READ,
            1,
        )?;

        let handle_pid = self.add_log_char(
            service.service_handle,
            UUID_LOG_PID_CHAR,
            18,
            CharacteristicProperty::NOTIFY,
            2,
        )?;

        // let params1 = AddCharacteristicParameters {
        //     service_handle:            service.service_handle,
        //     characteristic_uuid:       UUID_LOG_SENS_CHAR,
        //     characteristic_value_len:  48,
        //     // characteristic_properties: CharacteristicProperty::NOTIFY,
        //     characteristic_properties: CharacteristicProperty::NOTIFY
        //         | CharacteristicProperty::READ,
        //     security_permissions:      CharacteristicPermission::NONE,
        //     gatt_event_mask:           CharacteristicEvent::NONE,
        //     encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
        //     is_variable:               true,
        //     fw_version_before_v72:     false,
        // };
        // block!(self.add_characteristic(&params1))?;
        // rprintln!("sent c 1");
        // let c1 = match self.read_event_params_vendor()? {
        //     VReturnParameters::GattAddCharacteristic(c) => c,
        //     other => unimplemented!("other = {:?}", other),
        // };
        // rprintln!("c 1 = {:?}", c1);

        // let params2 = AddCharacteristicParameters {
        //     service_handle:            service.service_handle,
        //     characteristic_uuid:       UUID_LOG_PID_CHAR,
        //     characteristic_value_len:  18,
        //     // characteristic_properties: CharacteristicProperty::NOTIFY,
        //     characteristic_properties: CharacteristicProperty::NOTIFY,
        //     security_permissions:      CharacteristicPermission::NONE,
        //     gatt_event_mask:           CharacteristicEvent::NONE,
        //     encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
        //     is_variable:               true,
        //     fw_version_before_v72:     false,
        // };
        // block!(self.add_characteristic(&params2))?;
        // rprintln!("sent c 2");
        // let c2 = match self.read_event_params_vendor()? {
        //     VReturnParameters::GattAddCharacteristic(c) => c,
        //     other => unimplemented!("other = {:?}", other),
        // };
        // if c2.status != bluetooth_hci_defmt::Status::Success {
        //     panic!("c2 error: {:?}", c2);
        // }
        // rprintln!("c 2 = {:?}", c2);

        let logger = SvLogger {
            service_handle:   service.service_handle,
            // char_handle_quat: c0.characteristic_handle,
            // char_handle_sens: c1.characteristic_handle,
            // char_handle_pid:  c2.characteristic_handle,
            char_handle_quat: handle_quat,
            char_handle_sens: handle_sens,
            char_handle_pid:  handle_pid,
        };

        self.services.logger = Some(logger);

        Ok(())
    }

    fn add_log_char(
        &mut self,
        service: ServiceHandle,
        uuid: crate::bluetooth::gatt::Uuid,
        len: usize,
        props: CharacteristicProperty,
        n: u8,
    ) -> Result<CharacteristicHandle, BTError<SpiError, GpioError>> {
        let params = AddCharacteristicParameters {
            service_handle:            service,
            characteristic_uuid:       uuid,
            characteristic_value_len:  len,
            characteristic_properties: props,
            security_permissions:      CharacteristicPermission::NONE,
            gatt_event_mask:           CharacteristicEvent::NONE,
            encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
            is_variable:               true,
            fw_version_before_v72:     false,
        };
        block!(self.add_characteristic(&params))?;
        rprintln!("sent c {}", n);

        let c = match self.read_event_params_vendor()? {
            VReturnParameters::GattAddCharacteristic(c) => c,
            other => unimplemented!("other = {:?}", other),
        };

        if c.status != bluetooth_hci_defmt::Status::Success {
            panic!("c {} error: {:?}", n, c);
        }
        rprintln!("c {} = {:?}", n, c);

        Ok(c.characteristic_handle)
    }
}
