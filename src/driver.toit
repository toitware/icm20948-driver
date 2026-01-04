// Copyright (C) 2021 Toitware ApS. All rights reserved.

import io
import serial.device show Device
import serial.registers show Registers
import math

I2C-ADDRESS     ::= 0b1101000
I2C-ADDRESS-ALT ::= 0b1101001

ACCEL-SCALE-2G  ::= 0
ACCEL-SCALE-4G  ::= 1
ACCEL-SCALE-8G  ::= 2
ACCEL-SCALE-16G ::= 3

GYRO-SCALE-250DPS   ::= 0
GYRO-SCALE-500DPS   ::= 1
GYRO-SCALE-1000DPS  ::= 2
GYRO-SCALE-2000DPS  ::= 3

class Driver:

  static WHO-AM-I_ ::= 0xEA

  static ACCEL-SENSITIVITY_ ::= [16384.0, 8192.0, 4096.0, 2048.0]
  static GYRO-SENSITIVITY_ ::= [131.0, 65.5, 32.8, 16.4]

  static REGISTER-REG-BANK-SEL_   ::= 0x7F

  // Bank 0
  static REGISTER-WHO-AM-I_       ::= 0x00
  static REGISTER-LP-CONFIG_      ::= 0x05
  static REGISTER-PWR-MGMT-1_     ::= 0x06
  static REGISTER-PWR-MGMT-2_     ::= 0x07
  static REGISTER-INT-STATUS-1_   ::= 0x1A
  static REGISTER-ACCEL-XOUT-H_   ::= 0x2D
  static REGISTER-ACCEL-XOUT-L_   ::= 0x2E
  static REGISTER-ACCEL-YOUT-H_   ::= 0x2F
  static REGISTER-ACCEL-YOUT-L_   ::= 0x30
  static REGISTER-ACCEL-ZOUT-H_   ::= 0x31
  static REGISTER-ACCEL-ZOUT-L_   ::= 0x32
  static REGISTER-GYRO-XOUT-H_    ::= 0x33
  static REGISTER-GYRO-XOUT-L_    ::= 0x34
  static REGISTER-GYRO-YOUT-H_    ::= 0x35
  static REGISTER-GYRO-YOUT-L_    ::= 0x36
  static REGISTER-GYRO-ZOUT-H_    ::= 0x37
  static REGISTER-GYRO-ZOUT-L_    ::= 0x38

  // Bank 2
  static REGISTER-GYRO-SMPLRT-DIV_  ::= 0x0
  static REGISTER-GYRO-CONFIG-1_    ::= 0x1
  static REGISTER-GYRO-CONFIG-2_    ::= 0x2
  static REGISTER-ACCEL-CONFIG_     ::= 0x14
  static REGISTER-ACCEL-CONFIG-2_   ::= 0x15

  accel-sensitivity_/float := 0.0
  gyro-sensitivity_/float := 0.0

  reg_/Registers

  constructor dev/Device:
    reg_ = dev.registers

  on:
    tries := 5
    set-bank_ 0
    while (reg_.read-u8 REGISTER-WHO-AM-I_) != WHO-AM-I_:
      tries--
      if tries == 0: throw "INVALID_CHIP"
      sleep --ms=1

    reset_

    // Enable ACCEL and GYRO.
    set-bank_ 0
    // print ((reg_.read_u8 REGISTER_PWR_MGMT_1_).stringify 16)
    reg_.write-u8 REGISTER-PWR-MGMT-1_ 0b00000001
    reg_.write-u8 REGISTER-PWR-MGMT-2_ 0b00000000

  configure-accel --scale/int=ACCEL-SCALE-2G:
    r := reg_.read-u8 REGISTER-LP-CONFIG_
    reg_.write-u8 REGISTER-LP-CONFIG_ r & ~0b100000

    // Configure accel.
    cfg := 0b00111_00_1
    cfg |= scale << 1
    set-bank_ 2
    reg_.write-u8 REGISTER-ACCEL-CONFIG_ cfg
    set-bank_ 0

    accel-sensitivity_ = ACCEL-SENSITIVITY_[scale]

    sleep --ms=10

  configure-gyro --scale/int=GYRO-SCALE-250DPS:
    r := reg_.read-u8 REGISTER-LP-CONFIG_
    reg_.write-u8 REGISTER-LP-CONFIG_ r & ~0b10000

    set-bank_ 2

    //reg_.write_u8 REGISTER_GYRO_SMPLRT_DIV_ 0

    reg_.write-u8 REGISTER-GYRO-CONFIG-1_ 0b111_00_1 | scale << 1
    //reg_.write_u8 REGISTER_GYRO_CONFIG_2_ 0b1111

    set-bank_ 0

    gyro-sensitivity_ = GYRO-SENSITIVITY_[scale]

    sleep --ms=10

  off:

  read-point_ reg/int sensitivity/float -> math.Point3f:
    bytes := reg_.read-bytes reg 6
    x := io.BIG-ENDIAN.int16 bytes 0
    y := io.BIG-ENDIAN.int16 bytes 2
    z := io.BIG-ENDIAN.int16 bytes 4
    return math.Point3f
      x / sensitivity
      y / sensitivity
      z / sensitivity

  read-accel -> math.Point3f:
    while true:
      rdy := reg_.read-u8 REGISTER-INT-STATUS-1_
      if rdy == 1: break
      sleep --ms=1

    return read-point_ REGISTER-ACCEL-XOUT-H_ accel-sensitivity_

  read-gyro -> math.Point3f:
    while true:
      rdy := reg_.read-u8 REGISTER-INT-STATUS-1_
      if rdy == 1: break
      sleep --ms=1

    return read-point_ REGISTER-GYRO-XOUT-H_ gyro-sensitivity_

  reset_:
    set-bank_ 0
    reg_.write-u8 REGISTER-PWR-MGMT-1_ 0b10000001
    sleep --ms=5
    set-bank_ 0

  set-bank_ bank/int:
    reg_.write-u8 REGISTER-REG-BANK-SEL_ bank << 4
