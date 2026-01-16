// Copyright (C) 2026 Toit Contributors.
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

/**
Uses I2C bypass mode to allow access to the AK09916 directly on the host
  I2C bus.  It requires the additional ak09916 driver package.

This is for testing purposes only.  The bypass configuration will remain until
  disabled, or until the next power on/reset.
*/

import gpio
import i2c
import icm20948
import ak0991x show *

main:
  print

  bus := i2c.Bus
      --sda=gpio.Pin 19
      --scl=gpio.Pin 20
      --frequency=400_000

  bus-device-count := bus.scan.size
  if not bus.test icm20948.Driver.AK09916-I2C-ADDRESS:
    print "ICM20948 not found."
    return

  print "Bus scan has $bus-device-count devices"
  device := bus.device icm20948.I2C-ADDRESS-ALT
  sensor := icm20948.Driver device
  sensor.on

  // Enable bypass:
  sensor.enable-i2c-bypass
  if not bus.test Ak0991x.I2C-ADDRESS:
    print "bus missing the device. stopping..."
    return

  new-bus-scan := bus.scan
  print "After bypass, bus scan has $new-bus-scan.size devices"

  ak-device := bus.device Ak0991x.I2C-ADDRESS
  ak-sensor := Ak0991x ak-device

  print "Bus contains AK09916."
  print "Hardware ID: 0x$(%02x ak-sensor.get-hardware-id)"
  ak-sensor.set-operating-mode Ak0991x.OPMODE-CONT-MODE1-10HZ
  sleep --ms=250
  print "Data Ready: $ak-sensor.is-data-ready"
  print "Magnetic Field: $ak-sensor.read-magnetic-field"
  print "Magnetic Bearing: $ak-sensor.read-bearing"
  print
  100.repeat:
    print " bearing $(%0.3f ak-sensor.read-bearing)"
    sleep --ms=100
