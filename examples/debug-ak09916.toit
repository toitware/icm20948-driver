// Copyright (C) 2025 Toitware Contributors. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

/**
Uses I2C bypass mode to allow access to the AK09916 directly on the host
  I2C bus.  It requires the additional ak09916 driver package.

This is for testing purposes only.  The bypass configuration will remain until
  disabled, or until the next power on/reset.
*/

import gpio
import i2c
import icm20948
import ak09916

main:
  print

  bus := i2c.Bus
    --sda=gpio.Pin 8
    --scl=gpio.Pin 9
    --frequency=400_000

  bus-device-count := bus.scan.size
  if not (bus.test icm20948.Driver.AK09916-I2C-ADDRESS):
    print "Bus scan has $bus-device-count devices"

  device := bus.device icm20948.I2C-ADDRESS-ALT
  sensor := icm20948.Driver device
  sensor.on

  // Enable bypass:
  sensor.enable-i2c-bypass
  if not (bus.test icm20948.Driver.AK09916-I2C-ADDRESS):
    print "bus missing the device. stopping..."
    return

  ak-device := bus.device ak09916.Ak09916.I2C-ADDRESS
  ak-sensor := ak09916.Ak09916 ak-device

  print "Bus contains AK09916."
  print "Hardware ID: 0x$(%02x ak-sensor.get-hardware-id)"
  ak-sensor.set-operating-mode ak09916.Ak09916.OPMODE-CONT-MODE1
  sleep --ms=250
  print "Data Ready: $(ak-sensor.is-data-ready)"
  print "Magnetic Field: $ak-sensor.read-magnetic-field"
  print "Magnetic Bearing: $ak-sensor.read-bearing"
  print
  100.repeat:
    print " bearing $(%0.3f ak-sensor.read-bearing)"
    sleep --ms=100
