use embedded_hal as hal;
use hal::digital::v2::{InputPin, OutputPin};

use defmt::println as rprintln;

use stm32f4::stm32f401::{EXTI, RCC, SPI1, TIM2};
use stm32f4xx_hal::{
    block,
    gpio::{Alternate, Input, Output, Pin, PA4, PA5, PA6, PA7, PB0, PB2},
    nb,
    prelude::*,
    spi::{Error as SpiError, NoMiso, Spi1},
    timer::{DelayMs, SysDelay},
};

use crate::bluetooth::{
    gatt::{CharacteristicValue, UpdateCharacteristicValueParameters},
    hal_bt::Commands as HalCommands,
};
use crate::{
    bluetooth::gatt::{
        AddCharacteristicParameters, CharacteristicEvent, CharacteristicPermission,
        CharacteristicProperty, Commands as GattCommands, EncryptionKeySize,
    },
    uprint, uprintln,
};
use bluetooth_hci::{event::command::ReturnParameters, Event};

use crate::{
    bluetooth::{ev_command::GattService, gap::Commands as GapCommands},
    bluetooth::{
        ev_command::ReturnParameters as VReturnParameters,
        gatt::{AddServiceParameters, CharacteristicHandle, ServiceHandle},
        BTError, BluetoothSpi,
    },
    bt_control::{UUID_SENSOR_CHAR, UUID_SENSOR_SERVICE},
    sensors::V3,
    uart::*,
};

#[derive(Debug, Clone, Copy)]
pub struct SvSensors {
    sens_service_handle: ServiceHandle,
    sens_char_handle:    CharacteristicHandle,
}

impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    pub fn update_sensors(
        &mut self,
        gyro: V3,
        acc: V3,
        mag: V3,
        // baro: f32,
    ) -> Result<(), BTError<SpiError, GpioError>> {
        let service = if let Some(service) = self.services.sensors {
            service
        } else {
            panic!("no sensor service?");
        };

        // let mut data = [0u8; 40];
        let mut data = [0u8; 12];

        data[0..4].copy_from_slice(&gyro.x.to_be_bytes());
        data[4..8].copy_from_slice(&gyro.y.to_be_bytes());
        data[8..12].copy_from_slice(&gyro.z.to_be_bytes());

        // data[12..16].copy_from_slice(&acc.x.to_be_bytes());
        // data[16..20].copy_from_slice(&acc.y.to_be_bytes());
        // data[20..24].copy_from_slice(&acc.z.to_be_bytes());

        // data[24..28].copy_from_slice(&mag.x.to_be_bytes());
        // data[28..32].copy_from_slice(&mag.y.to_be_bytes());
        // data[32..36].copy_from_slice(&mag.z.to_be_bytes());

        // data[36..40].copy_from_slice(&baro.to_be_bytes());

        let val = UpdateCharacteristicValueParameters {
            service_handle:        service.sens_service_handle,
            characteristic_handle: service.sens_char_handle,
            offset:                0,
            value:                 &data,
        };
        block!(self.update_characteristic_value(&val)).unwrap();

        let result = self.ignore_event_timeout(None)?;

        Ok(())
    }

    /// block
    pub fn init_sensor_service(&mut self) -> Result<(), BTError<SpiError, GpioError>> {
        let params = AddServiceParameters {
            uuid:                  UUID_SENSOR_SERVICE,
            service_type:          crate::bluetooth::gatt::ServiceType::Primary,
            max_attribute_records: 8,
        };
        block!(self.add_service(&params))?;
        rprintln!("sent sensor service");

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
        rprintln!("sensor service = {:?}", service);

        let params0 = AddCharacteristicParameters {
            service_handle:            service.service_handle,
            characteristic_uuid:       UUID_SENSOR_CHAR,
            // characteristic_value_len:  48,
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
        rprintln!("sent sensor c");

        let c = match self.read_event_params_vendor()? {
            VReturnParameters::GattAddCharacteristic(c) => c,
            other => unimplemented!("other = {:?}", other),
        };
        rprintln!("sensor c = {:?}", c);

        let sensors = SvSensors {
            sens_service_handle: service.service_handle,
            sens_char_handle:    c.characteristic_handle,
        };

        self.services.sensors = Some(sensors);

        Ok(())
    }
}

// // impl<'buf, SPI, CS, Reset, Input, GpioError> BluetoothSpi<'buf, SPI, CS, Reset, Input>
// impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
// where
//     CS: OutputPin<Error = GpioError>,
//     Reset: OutputPin<Error = GpioError>,
//     Input: InputPin<Error = GpioError>,
//     GpioError: core::fmt::Debug,
// {
//     pub fn init_sensor_service(
//         &mut self,
//         uart: &mut UART,
//     ) -> nb::Result<(), BTError<SpiError, GpioError>> {
//         let params = AddServiceParameters {
//             uuid:                  UUID_CONSOLE_LOG_SERVICE,
//             service_type:          crate::bluetooth::gatt::ServiceType::Primary,
//             max_attribute_records: 8,
//         };
//         block!(self.add_service(&params))?;

//         let service: GattService = match self.read_event_params_vendor()? {
//             VReturnParameters::GattAddService(service) => service,
//             _ => unimplemented!(),
//         };
//         uprintln!(uart, "service = {:?}", service);

//         let params0 = AddCharacteristicParameters {
//             service_handle:            service.service_handle,
//             characteristic_uuid:       UUID_CONSOLE_LOG_CHAR,
//             characteristic_value_len:  18,
//             characteristic_properties: CharacteristicProperty::NOTIFY,
//             // characteristic_properties: CharacteristicProperty::NOTIFY
//             // | CharacteristicProperty::READ,
//             security_permissions:      CharacteristicPermission::NONE,
//             gatt_event_mask:           CharacteristicEvent::NONE,
//             // gatt_event_mask: CharacteristicEvent::CONFIRM_READ,
//             encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
//             is_variable:               true,
//             fw_version_before_v72:     true,
//         };
//         block!(self.add_characteristic(&params0))?;

//         // let params1 = AddCharacteristicParameters {
//         //     service_handle: service.service_handle,
//         //     characteristic_uuid: UUID_CONSOLE_LOG_CHAR_WRITE,
//         //     characteristic_value_len: 18,
//         //     characteristic_properties: CharacteristicProperty::WRITE
//         //         | CharacteristicProperty::WRITE_WITHOUT_RESPONSE,
//         //     security_permissions: CharacteristicPermission::NONE,
//         //     gatt_event_mask: CharacteristicEvent::NONE,
//         //     encryption_key_size: EncryptionKeySize::with_value(7).unwrap(),
//         //     is_variable: true,
//         //     fw_version_before_v72: true,
//         // };
//         // block!(self.add_characteristic(&params1))?;

//         // block!(self.read_event(uart))?;

//         let c = match self.read_event_params_vendor()? {
//             VReturnParameters::GattAddCharacteristic(c) => c,
//             _ => unimplemented!(),
//         };

//         uprintln!(uart, "c = {:?}", c);

//         let logger = SvSensors {
//             service_handle: service.service_handle,
//             char_handle:    c.characteristic_handle,
//         };

//         // self.services = Some(logger);

//         Ok(())
//     }
// }
