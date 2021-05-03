// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

/**
A simple example of how to use the ICM20948 driver.
*/

import gpio
import serial.protocols.i2c as i2c
import icm20948

main:
  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  device := bus.device icm20948.I2C_ADDRESS_ALT

  sensor := icm20948.Driver device

  sensor.on
  sensor.configure_accel
  sensor.configure_gyro
  while true:
    print "Acceleration: $sensor.read_accel"
    print "Gyroscope: $sensor.read_gyro"
    sleep --ms=1000
  sensor.off
