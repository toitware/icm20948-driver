// Copyright (C) 2021 Toitware ApS. All rights reserved.

import io
import serial.device show Device
import serial.registers show Registers
import math

I2C_ADDRESS     ::= 0b1101000
I2C_ADDRESS_ALT ::= 0b1101001

ACCEL_SCALE_2G  ::= 0
ACCEL_SCALE_4G  ::= 1
ACCEL_SCALE_8G  ::= 2
ACCEL_SCALE_16G ::= 3

GYRO_SCALE_250DPS   ::= 0
GYRO_SCALE_500DPS   ::= 1
GYRO_SCALE_1000DPS  ::= 2
GYRO_SCALE_2000DPS  ::= 3

class Driver:

  static WHO_AM_I_ ::= 0xEA

  static ACCEL_SENSITIVITY_ ::= [16384.0, 8192.0, 4096.0, 2048.0]
  static GYRO_SENSITIVITY_ ::= [131.0, 65.5, 32.8, 16.4]

  static REGISTER_REG_BANK_SEL_   ::= 0x7F

  // Bank 0
  static REGISTER_WHO_AM_I_       ::= 0x00
  static REGISTER_LP_CONFIG_      ::= 0x05
  static REGISTER_PWR_MGMT_1_     ::= 0x06
  static REGISTER_PWR_MGMT_2_     ::= 0x07
  static REGISTER_INT_STATUS_1_   ::= 0x1A
  static REGISTER_ACCEL_XOUT_H_   ::= 0x2D
  static REGISTER_ACCEL_XOUT_L_   ::= 0x2E
  static REGISTER_ACCEL_YOUT_H_   ::= 0x2F
  static REGISTER_ACCEL_YOUT_L_   ::= 0x30
  static REGISTER_ACCEL_ZOUT_H_   ::= 0x31
  static REGISTER_ACCEL_ZOUT_L_   ::= 0x32
  static REGISTER_GYRO_XOUT_H_    ::= 0x33
  static REGISTER_GYRO_XOUT_L_    ::= 0x34
  static REGISTER_GYRO_YOUT_H_    ::= 0x35
  static REGISTER_GYRO_YOUT_L_    ::= 0x36
  static REGISTER_GYRO_ZOUT_H_    ::= 0x37
  static REGISTER_GYRO_ZOUT_L_    ::= 0x38

  // Bank 2
  static REGISTER_GYRO_SMPLRT_DIV_  ::= 0x0
  static REGISTER_GYRO_CONFIG_1_    ::= 0x1
  static REGISTER_GYRO_CONFIG_2_    ::= 0x2
  static REGISTER_ACCEL_CONFIG_     ::= 0x14
  static REGISTER_ACCEL_CONFIG_2_   ::= 0x15

  accel_sensitivity_/float := 0.0
  gyro_sensitivity_/float := 0.0

  reg_/Registers

  constructor dev/Device:
    reg_ = dev.registers

  on:
    tries := 5
    set_bank_ 0
    while (reg_.read_u8 REGISTER_WHO_AM_I_) != WHO_AM_I_:
      tries--
      if tries == 0: throw "INVALID_CHIP"
      sleep --ms=1

    reset_

    // Enable ACCEL and GYRO.
    set_bank_ 0
    // print ((reg_.read_u8 REGISTER_PWR_MGMT_1_).stringify 16)
    reg_.write_u8 REGISTER_PWR_MGMT_1_ 0b00000001
    reg_.write_u8 REGISTER_PWR_MGMT_2_ 0b00000000

  configure_accel --scale/int=ACCEL_SCALE_2G:
    r := reg_.read_u8 REGISTER_LP_CONFIG_
    reg_.write_u8 REGISTER_LP_CONFIG_ r & ~0b100000

    // Configure accel.
    cfg := 0b00111_00_1
    cfg |= scale << 1
    set_bank_ 2
    reg_.write_u8 REGISTER_ACCEL_CONFIG_ cfg
    set_bank_ 0

    accel_sensitivity_ = ACCEL_SENSITIVITY_[scale]

    sleep --ms=10

  configure_gyro --scale/int=GYRO_SCALE_250DPS:
    r := reg_.read_u8 REGISTER_LP_CONFIG_
    reg_.write_u8 REGISTER_LP_CONFIG_ r & ~0b10000

    set_bank_ 2

    //reg_.write_u8 REGISTER_GYRO_SMPLRT_DIV_ 0

    reg_.write_u8 REGISTER_GYRO_CONFIG_1_ 0b111_00_1 | scale << 1
    //reg_.write_u8 REGISTER_GYRO_CONFIG_2_ 0b1111

    set_bank_ 0

    gyro_sensitivity_ = GYRO_SENSITIVITY_[scale]

    sleep --ms=10

  off:

  read_point_ reg/int sensitivity/float -> math.Point3f:
    bytes := reg_.read_bytes reg 6
    x := io.BIG_ENDIAN.int16 bytes 0
    y := io.BIG_ENDIAN.int16 bytes 2
    z := io.BIG_ENDIAN.int16 bytes 4
    return math.Point3f
      x / sensitivity
      y / sensitivity
      z / sensitivity

  read_accel -> math.Point3f:
    while true:
      rdy := reg_.read_u8 REGISTER_INT_STATUS_1_
      if rdy == 1: break
      sleep --ms=1

    return read_point_ REGISTER_ACCEL_XOUT_H_ accel_sensitivity_

  read_gyro -> math.Point3f:
    while true:
      rdy := reg_.read_u8 REGISTER_INT_STATUS_1_
      if rdy == 1: break
      sleep --ms=1

    return read_point_ REGISTER_GYRO_XOUT_H_ gyro_sensitivity_

  reset_:
    set_bank_ 0
    reg_.write_u8 REGISTER_PWR_MGMT_1_ 0b10000001
    sleep --ms=5
    set_bank_ 0

  set_bank_ bank/int:
    reg_.write_u8 REGISTER_REG_BANK_SEL_ bank << 4
