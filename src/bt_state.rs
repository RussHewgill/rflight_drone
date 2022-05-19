use bluetooth_hci::ConnectionHandle;
use embedded_hal::digital::v2::{InputPin, OutputPin};

use crate::bt_control::BTEvent;

use crate::bluetooth::ev_command::ReturnParameters as VReturnParameters;
use crate::bluetooth::{events::BlueNRGEvent, BTError, BluetoothSpi};
use crate::flight_control::ControlInputs;
use crate::motors::MotorsPWM;

use bluetooth_hci::event::command::ReturnParameters;
use bluetooth_hci::event::{Event, VendorEvent};

use defmt::println as rprintln;

#[derive(Debug)]
pub enum BTState {
    Disconnected,
    // Connected(ConnectionHandle, BdAddrType, FixedConnectionInterval),
    Connected(ConnectionHandle),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConnectionChange {
    NewConnection(ConnectionHandle),
    Disconnect,
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

    pub fn handle_connect_disconnect(
        &mut self,
        event: &BTEvent,
    ) -> Option<ConnectionChange> {
        match event {
            Event::LeConnectionComplete(status) => {
                if status.status == bluetooth_hci::Status::Success {
                    rprintln!("new connection established 0");
                    *self = Self::Connected(status.conn_handle);
                    return Some(ConnectionChange::NewConnection(status.conn_handle));
                } else {
                    rprintln!(
                        "Connection Complete, but status = {:?}",
                        defmt::Debug2Format(&status.status)
                    );
                    return None;
                }
            }
            Event::ConnectionComplete(status) => {
                if status.status == bluetooth_hci::Status::Success {
                    rprintln!("new connection established 1");
                    *self = Self::Connected(status.conn_handle);
                    return Some(ConnectionChange::NewConnection(status.conn_handle));
                } else {
                    rprintln!(
                        "Connection Complete, but status = {:?}",
                        defmt::Debug2Format(&status.status)
                    );
                    return None;
                }
            }
            Event::DisconnectionComplete(status) => {
                *self = Self::Disconnected;
            }
            Event::Vendor(BlueNRGEvent::HalInitialized(reason)) => {
                rprintln!("bt restarted, reason = {:?}", defmt::Debug2Format(&reason));
            }
            Event::CommandComplete(params) => {
                rprintln!("command complete: {:?}", defmt::Debug2Format(&params));
            }
            _ => {
                rprintln!("unhandled event = {:?}", defmt::Debug2Format(&event));
            }
        }
        None
    }
}

impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    pub fn handle_input(
        &mut self,
        motors: &mut MotorsPWM,
        control_inputs: &mut ControlInputs,
        event: &BTEvent,
        //
    ) {
        match event {
            Event::Vendor(BlueNRGEvent::GattAttributeModified(att)) => {
                if let Some(input) = self.services.input {
                    /// attribute handle is 1 greater than characteristic handle ??
                    if att.attr_handle.0 + 1 == input.input_char.0 {
                        if let Some(ci) = ControlInputs::deserialize(att.data()) {
                            *control_inputs = ci;
                        }
                    }
                }
                // if Some(att.attr_handle) == self.services.input.map(|x| )
            }
            _ => {}
        }
    }
}
