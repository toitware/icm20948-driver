// Copyright (C) 2026 Toit Contributors.
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

/**
A simple example of how to use the ICM20948 driver.
*/

import gpio
import i2c
import icm20948

main:
  bus := i2c.Bus
    --sda=gpio.Pin 19
    --scl=gpio.Pin 20

  device := bus.device icm20948.I2C-ADDRESS-ALT

  sensor := icm20948.Driver device

  sensor.on
  sensor.configure-accel
  sensor.configure-gyro
  50.repeat:
    print " Acceleration: $sensor.read-accel"
    print " Gyroscope:    $sensor.read-gyro"
    sleep --ms=1000
  sensor.off
