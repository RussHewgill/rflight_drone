#!/bin/bash

sudo systemctl start bluetooth

# sudo rfkill list
# sudo rfkill unblock all

bluetoothctl power on

