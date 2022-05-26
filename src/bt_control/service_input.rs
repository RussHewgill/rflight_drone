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
        gatt::{
            AccessPermission, AddDescriptorParameters, DescriptorPermission,
            UpdateLongCharacteristicValueParameters,
        },
        hal_bt::Commands as HalCommands,
    },
    bt_control::{
        UUID_INPUT_PID_CFG_CHAR, UUID_LOG_QUAT_CHAR, UUID_LOG_SENS_CHAR, UUID_TEST0,
        UUID_TEST1,
    },
    flight_control::ControlInputs,
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
    bt_control::{UUID_INPUT_CHAR, UUID_INPUT_SERVICE},
};

#[derive(Debug, Clone, Copy)]
pub struct SvInput {
    pub input_service:      ServiceHandle,
    pub input_char:         CharacteristicHandle,
    pub input_pid_cfg_char: CharacteristicHandle,
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
        const NUM_SERVICES: usize = 2;
        // const NUM_RECORDS: usize = 1 + 3 * NUM_SERVICES;

        let params = AddServiceParameters {
            uuid:                  UUID_INPUT_SERVICE,
            service_type:          crate::bluetooth::gatt::ServiceType::Primary,
            max_attribute_records: 1 + 3 * NUM_SERVICES,
        };
        block!(self.add_service(&params))?;
        rprintln!("sent input service");

        let service = match self._read_event()? {
            Event::CommandComplete(params) => match params.return_params {
                ReturnParameters::Vendor(vs) => match vs {
                    VReturnParameters::GattAddService(service) => service,
                    other => {
                        rprintln!("other = {:?}", other);
                        panic!("event_params_vendor other 0");
                    }
                },
                other => {
                    rprintln!("other = {:?}", other);
                    panic!("event_params_vendor other 1");
                }
            },
            other => {
                rprintln!("other = {:?}", other);
                panic!("event_params_vendor other 2");
            }
        };
        rprintln!("service = {:?}", service);

        // let params0 = AddCharacteristicParameters {
        //     service_handle:            service.service_handle,
        //     characteristic_uuid:       UUID_INPUT_CHAR_THROTTLE,
        //     characteristic_value_len:  18,
        //     characteristic_properties: CharacteristicProperty::empty()
        //         // | CharacteristicProperty::WRITE
        //         | CharacteristicProperty::WRITE_WITHOUT_RESPONSE
        //         | CharacteristicProperty::NOTIFY
        //         | CharacteristicProperty::READ,
        //     security_permissions:      CharacteristicPermission::NONE,
        //     // gatt_event_mask:           CharacteristicEvent::NONE,
        //     gatt_event_mask:           CharacteristicEvent::ATTRIBUTE_WRITE,
        //     // gatt_event_mask:           CharacteristicEvent::ATTRIBUTE_WRITE
        //     //     | CharacteristicEvent::CONFIRM_READ
        //     //     | CharacteristicEvent::CONFIRM_WRITE,
        //     encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
        //     is_variable:               true,
        //     fw_version_before_v72:     false,
        // };
        // block!(self.add_characteristic(&params0))?;
        // rprintln!("sent input c 0");
        // let c0 = match self.read_event_params_vendor()? {
        //     VReturnParameters::GattAddCharacteristic(c) => c,
        //     other => unimplemented!("other = {:?}", other),
        // };
        // rprintln!("input c 0 = {:?}", c0);

        let handle_input = self.add_input_char(
            service.service_handle,
            UUID_INPUT_CHAR,
            18,
            CharacteristicProperty::NOTIFY
                | CharacteristicProperty::WRITE_WITHOUT_RESPONSE
                | CharacteristicProperty::READ,
            CharacteristicEvent::ATTRIBUTE_WRITE,
            3,
        )?;

        let handle_pid_cfg = self.add_input_char(
            service.service_handle,
            UUID_INPUT_PID_CFG_CHAR,
            18,
            CharacteristicProperty::NOTIFY
                | CharacteristicProperty::WRITE_WITHOUT_RESPONSE
                | CharacteristicProperty::READ,
            CharacteristicEvent::ATTRIBUTE_WRITE,
            4,
        )?;

        let input = SvInput {
            input_service:      service.service_handle,
            input_char:         handle_input,
            input_pid_cfg_char: handle_pid_cfg,
        };

        self.services.input = Some(input);

        Ok(())
    }

    fn add_input_char(
        &mut self,
        service: ServiceHandle,
        uuid: crate::bluetooth::gatt::Uuid,
        len: usize,
        props: CharacteristicProperty,
        event_mask: CharacteristicEvent,
        n: u8,
    ) -> Result<CharacteristicHandle, BTError<SpiError, GpioError>> {
        let params = AddCharacteristicParameters {
            service_handle:            service,
            characteristic_uuid:       uuid,
            characteristic_value_len:  len,
            characteristic_properties: props,
            security_permissions:      CharacteristicPermission::NONE,
            gatt_event_mask:           event_mask,
            encryption_key_size:       EncryptionKeySize::with_value(7).unwrap(),
            is_variable:               true,
            fw_version_before_v72:     false,
        };
        block!(self.add_characteristic(&params))?;
        rprintln!("sent c {}", n);

        let c = match self.read_event_params_vendor()? {
            VReturnParameters::GattAddCharacteristic(c) => c,
            other => {
                rprintln!("other = {:?}", other);
                unimplemented!("other");
            }
        };

        if c.status != bluetooth_hci_defmt::Status::Success {
            rprintln!("c {} error: {:?}", n, c);
            panic!();
        }
        rprintln!("c {} = {:?}", n, c);

        Ok(c.characteristic_handle)
    }
}
