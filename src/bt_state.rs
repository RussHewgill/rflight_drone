use bluetooth_hci_defmt::ConnectionHandle;
use embedded_hal::digital::v2::{InputPin, OutputPin};

use crate::bt_control::BTEvent;

use crate::bluetooth::ev_command::ReturnParameters as VReturnParameters;
use crate::bluetooth::{events::BlueNRGEvent, BTError, BluetoothSpi};
use crate::flight_control::{ControlInputs, DroneController, IdPID};
use crate::motors::MotorsPWM;
use crate::pid::PIDParam;
use crate::sensors::UQuat;

use bluetooth_hci_defmt::event::command::ReturnParameters;
use bluetooth_hci_defmt::event::{Event, VendorEvent};

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
}

/// handle_connect_disconnect
impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    // fn new_conn(self, st: bluetooth_hci_defmt::event::ConnectionComplete<_>) -> Self {
    //     Self::Connected(st.conn_handle)
    // }

    pub fn handle_connect_disconnect(
        &mut self,
        event: &BTEvent,
    ) -> Option<ConnectionChange> {
        match event {
            Event::LeConnectionComplete(status) => {
                if status.status == bluetooth_hci_defmt::Status::Success {
                    rprintln!("new connection established 0");
                    self.state = BTState::Connected(status.conn_handle);
                    return Some(ConnectionChange::NewConnection(status.conn_handle));
                } else {
                    rprintln!("Connection Complete, but status = {:?}", status.status);
                    return None;
                }
            }
            Event::ConnectionComplete(status) => {
                if status.status == bluetooth_hci_defmt::Status::Success {
                    rprintln!("new connection established 1");
                    self.state = BTState::Connected(status.conn_handle);
                    return Some(ConnectionChange::NewConnection(status.conn_handle));
                } else {
                    rprintln!("Connection Complete, but status = {:?}", status.status);
                    return None;
                }
            }
            Event::DisconnectionComplete(status) => {
                rprintln!("disconnected, reason = {:?}", status.reason);
                self.state = BTState::Disconnected;
                self.begin_advertise().unwrap();
            }
            Event::Vendor(BlueNRGEvent::HalInitialized(reason)) => {
                rprintln!("bt restarted, reason = {:?}", reason);
            }
            Event::CommandComplete(params) => {
                // rprintln!("command complete: {:?}", params);
            }
            _ => {
                // rprintln!("handle_connect_disconnect: unhandled event = {:?}", event);
            }
        }
        None
    }
}

/// Inputs
impl<CS, Reset, Input, GpioError> BluetoothSpi<CS, Reset, Input>
where
    CS: OutputPin<Error = GpioError>,
    Reset: OutputPin<Error = GpioError>,
    Input: InputPin<Error = GpioError>,
    GpioError: core::fmt::Debug,
{
    /// attribute handle is 1 greater than characteristic handle ??
    /// returns Some(true) when motors are set to armed
    pub fn handle_input(
        &mut self,
        motors: &mut MotorsPWM,
        control_inputs: &mut ControlInputs,
        controller: &mut DroneController,
        quat: UQuat,
        voltage: f32,
        event: &BTEvent,
    ) -> Option<bool> {
        let input = if let Some(input) = self.services.input {
            input
        } else {
            return None;
        };

        /// 24 = input att = input char + 1
        /// 27 = pid att = pid char + 1
        match event {
            Event::Vendor(BlueNRGEvent::GattAttributeModified(att)) => {
                let data = att.data();
                // rprintln!("att.attr_handle = {:?}", att.attr_handle.0);
                // rprintln!("data = {:?}", data);

                /// received input changed event
                if att.attr_handle.0 == input.input_char.0 + 1 {
                    // rprintln!("input update");
                    // rprintln!("data.len() = {:?}", data.len());

                    let mut out = false;
                    match control_inputs.update(controller, att.data()) {
                        Some(true) => {
                            rprintln!("Setting motors armed");
                            if motors.set_armed(true, &self.state, control_inputs, quat) {
                                controller.reset_integrals();
                                control_inputs.set_motors_armed(true);
                            }
                            out = true;
                        }
                        Some(false) => {
                            rprintln!("Setting motors disarmed");
                            motors.set_disarmed();
                            controller.reset_integrals();
                            control_inputs.set_motors_armed(false);
                            out = true;
                        }
                        None => {}
                    }
                    if out {
                        return Some(out);
                    }

                    // if let Some(ci) = ControlInputs::deserialize(att.data()) {
                    //     let mut out = false;
                    //     /// arm or disarm motors if changed
                    //     if !control_inputs.get_motors_armed() && ci.get_motors_armed() {
                    //         motors.set_armed(
                    //             ci.get_motors_armed(),
                    //             &self.state,
                    //             *control_inputs,
                    //             quat,
                    //         );
                    //         out = true;
                    //     } else if control_inputs.get_motors_armed()
                    //         && !ci.get_motors_armed()
                    //     {
                    //         motors.set_disarmed();
                    //         out = true;
                    //     }
                    //     /// set new inputs
                    //     *control_inputs = ci;
                    //     if out {
                    //         return Some(out);
                    //     }
                    // }
                }

                if att.attr_handle.0 == input.input_pid_cfg_char.0 + 1 {
                    // rprintln!("pid update");
                    // rprintln!("data.len() = {:?}", data.len());
                    let id = IdPID::from_u8(data[0]).unwrap();
                    let param = PIDParam::from_u8(data[1]).unwrap();
                    let val = f32::from_be_bytes(data[2..6].try_into().unwrap());

                    rprintln!("setting {:?} {:?} = {:?}", id, param, val);

                    // controller[id][param] = val;
                    controller[id].set_param(param, val);

                    // if param == PIDParam::Ki || param == PIDParam::KiLimit {
                    //     controller[id].reset_integral();
                    // }

                    // TODO:
                    // /// Pitch and Yaw should match
                    // match id {
                    //     IdPID::RollRate => {
                    //         controller[IdPID::PitchRate][param] = val;
                    //     }
                    //     IdPID::RollStab => {
                    //         controller[IdPID::PitchStab][param] = val;
                    //     }
                    //     IdPID::PitchRate => {
                    //         controller[IdPID::RollRate][param] = val;
                    //     }
                    //     IdPID::PitchStab => {
                    //         controller[IdPID::RollStab][param] = val;
                    //     }
                    //     _ => {}
                    // }

                    //
                }
            }
            _ => {}
        }

        None
    }
}
