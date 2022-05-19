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
        gatt::{
            AccessPermission, AddDescriptorParameters, DescriptorPermission,
            UpdateLongCharacteristicValueParameters,
        },
        hal_bt::Commands as HalCommands,
    },
    bt_control::{UUID_INPUT_DESC_THROTTLE, UUID_LOG_CHAR, UUID_LOG_SENS_CHAR},
    flight_control::ControlInputs,
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
    bt_control::{UUID_INPUT_CHAR_THROTTLE, UUID_INPUT_SERVICE},
    uart::*,
};

#[derive(Debug, Clone, Copy)]
pub struct SvInput {
    pub input_service: ServiceHandle,
    pub input_char:    CharacteristicHandle,
    // pub roll_char:     CharacteristicHandle,
    // pub pitch_char:    CharacteristicHandle,
    // pub yaw_char:      CharacteristicHandle,
}

/// init
impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    pub fn init_input_service(&mut self) -> Result<(), BTError<SpiError, GpioError>> {
        let params = AddServiceParameters {
            uuid:                  UUID_INPUT_SERVICE,
            service_type:          crate::bluetooth::gatt::ServiceType::Primary,
            max_attribute_records: 8,
        };
        block!(self.add_service(&params))?;
        rprintln!("sent input service");

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
            characteristic_uuid:       UUID_INPUT_CHAR_THROTTLE,
            characteristic_value_len:  18,
            characteristic_properties: CharacteristicProperty::empty()
                // | CharacteristicProperty::WRITE
                | CharacteristicProperty::WRITE_WITHOUT_RESPONSE
                | CharacteristicProperty::NOTIFY
                | CharacteristicProperty::READ,
            security_permissions:      CharacteristicPermission::NONE,
            // gatt_event_mask:           CharacteristicEvent::NONE,
            gatt_event_mask:           CharacteristicEvent::ATTRIBUTE_WRITE,
            // gatt_event_mask:           CharacteristicEvent::ATTRIBUTE_WRITE
            //     | CharacteristicEvent::CONFIRM_READ
            //     | CharacteristicEvent::CONFIRM_WRITE,
            encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
            is_variable:               true,
            fw_version_before_v72:     false,
        };
        block!(self.add_characteristic(&params0))?;
        rprintln!("sent input c 0");

        let c0 = match self.read_event_params_vendor()? {
            VReturnParameters::GattAddCharacteristic(c) => c,
            other => unimplemented!("other = {:?}", other),
        };
        rprintln!("input c 0 = {:?}", defmt::Debug2Format(&c0));

        let input = SvInput {
            input_service: service.service_handle,
            input_char:    c0.characteristic_handle,
        };

        self.services.input = Some(input);

        Ok(())
    }
}
