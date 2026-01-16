// Copyright (C) 2021 Toitware ApS. All rights reserved.

import io
import serial.device show Device
import serial.registers show Registers
import math
import log

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

  // Bank 0.
  static REGISTER-WHO-AM-I_       ::= 0x00
  static REGISTER-USER-CTRL_      ::= 0x03
  static REGISTER-LP-CONFIG_      ::= 0x05
  static REGISTER-PWR-MGMT-1_     ::= 0x06
  static REGISTER-PWR-MGMT-2_     ::= 0x07
  static REGISTER-INT-PIN-CFG_    ::= 0x0F
  static REGISTER-INT-ENABLE_     ::= 0x10
  static REGISTER-INT-ENABLE-1_   ::= 0x11
  static REGISTER-INT-ENABLE-2_   ::= 0x12
  static REGISTER-INT-ENABLE-3_   ::= 0x13
  static REGISTER-I2C-MST-STATUS_ ::= 0x17
  static REGISTER-INT-STATUS_     ::= 0x19
  static REGISTER-INT-STATUS-1_   ::= 0x1A
  static REGISTER-INT-STATUS-2_   ::= 0x1B
  static REGISTER-INT-STATUS-3_   ::= 0x1C
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

  static REGISTER-EXT-SLV-DATA-00_ ::= 0x3b
  static REGISTER-EXT-SLV-DATA-01_ ::= 0x3c
  static REGISTER-EXT-SLV-DATA-02_ ::= 0x3d
  static REGISTER-EXT-SLV-DATA-23_ ::= 0x52

  static REGISTER-FIFO-EN-1_       ::= 0x66
  static REGISTER-FIFO-EN-2_       ::= 0x67
  static REGISTER-FIFO-RST_        ::= 0x68
  static REGISTER-FIFO-MODE_       ::= 0x69
  static REGISTER-FIFO-COUNT_      ::= 0x70
  static REGISTER-FIFO-R-W_        ::= 0x72
  static REGISTER-DATA-RDY-STATUS_ ::= 0x74
  static REGISTER-FIFO-CFG_        ::= 0x76

  // Masks: $REGISTER-USER-CTRL_.
  static USER-CTRL-DMP-EN_      ::= 0b10000000
  static USER-CTRL-FIFO-EN_     ::= 0b01000000
  static USER-CTRL-I2C-MST-EN_  ::= 0b00100000
  static USER-CTRL-I2C-IF-DIS_  ::= 0b00010000  // Reset I2C Slave module and put the serial interface in SPI mode only.
  static USER-CTRL-DMP-RST_     ::= 0b00001000  // Reset DMP. Asynchronous. Takes 1 clock cycle of 20 Mhz clock.
  static USER-CTRL-SRAM-RST_    ::= 0b00000100  // Reset SRAM. Asynchronous. Takes 1 clock cycle of 20 Mhz clock.
  static USER-CTRL-I2C-MST-RST_ ::= 0b00000010  // Reset I2C. Asynchronous. Takes 1 clock cycle of 20 Mhz clock. Could cause I2C slave to hang. See datasheet.

  // Masks: REGISTER-LP-CONFIG_.
  static LP-CONFIG-I2C-MST-CYCLE_    ::= 0b01000000
  static LP-CONFIG-I2C-ACCEL-CYCLE_  ::= 0b00100000
  static LP-CONFIG-I2C-GYRO-CYCLE_   ::= 0b00010000

  // Bank 2.
  static REGISTER-GYRO-SMPLRT-DIV_  ::= 0x0
  static REGISTER-GYRO-CONFIG-1_    ::= 0x1
  static REGISTER-GYRO-CONFIG-2_    ::= 0x2
  static REGISTER-ODR-ALIGN-EN_     ::= 0x9
  static REGISTER-ACCEL-CONFIG_     ::= 0x14
  static REGISTER-ACCEL-CONFIG-2_   ::= 0x15

  // Bank 3 - I2C Master Engine.
  static REGISTER-I2C-MST-ODR-CONFIG_ ::= 0x00
  static REGISTER-I2C-MST-CTRL_       ::= 0x01
  static REGISTER-I2C-MST-DELAY-CTRL_ ::= 0x02

  // Slave read/write engines:
  // SLV0–SLV3: continuous / automatic reads - Repeatedly read data from the
  //   external sensors and deposit it into Bank 0 $REGISTER-EXT-SLV-DATA-00_
  //   through to REGISTER-EXT-SLV-DATA-23_.
  // SLV4: one-shot command channel - Perform single, blocking I2C transactions
  //   (writes or reads).  Result goes into $REGISTER-I2C-SLV4-DI_.
  //   Must wait for 'DONE' from $REGISTER-I2C-MST-STATUS_.
  static REGISTER-I2C-SLV0-ADDR_      ::= 0x03  // R/W [7] and PHY address [0..6] of I2C Slave x.
  static REGISTER-I2C-SLV0-REG_       ::= 0x04  // I2C slave x register address from where to begin data transfer.
  static REGISTER-I2C-SLV0-CTRL_      ::= 0x05  // Bitmask of properties for the read.
  static REGISTER-I2C-SLV0-DO_        ::= 0x06  // Data out when slave x is set to write.

  static REGISTER-I2C-SLV1-ADDR_      ::= 0x07
  static REGISTER-I2C-SLV1-REG_       ::= 0x08
  static REGISTER-I2C-SLV1-CTRL_      ::= 0x09
  static REGISTER-I2C-SLV1-DO_        ::= 0x0A

  static REGISTER-I2C-SLV2-ADDR_      ::= 0x0B
  static REGISTER-I2C-SLV2-REG_       ::= 0x0C
  static REGISTER-I2C-SLV2-CTRL_      ::= 0x0D
  static REGISTER-I2C-SLV2-DO_        ::= 0x0E

  static REGISTER-I2C-SLV3-ADDR_      ::= 0x0F
  static REGISTER-I2C-SLV3-REG_       ::= 0x10
  static REGISTER-I2C-SLV3-CTRL_      ::= 0x11
  static REGISTER-I2C-SLV3-DO_        ::= 0x12

  static REGISTER-I2C-SLV4-ADDR_      ::= 0x13
  static REGISTER-I2C-SLV4-REG_       ::= 0x14
  static REGISTER-I2C-SLV4-CTRL_      ::= 0x15
  static REGISTER-I2C-SLV4-DO_        ::= 0x16  // Data OUT when slave 4 is set to write.
  static REGISTER-I2C-SLV4-DI_        ::= 0x17  // Data IN when slave 4.

  // Masks: $REGISTER-INT-PIN-CFG_.
  static INT-PIN-CFG-INT1-ACTL_             ::= 0b10000000
  static INT-PIN-CFG-INT1-OPEN_             ::= 0b01000000
  static INT-PIN-CFG-INT1-LATCH-EN_         ::= 0b00100000
  static INT-PIN-CFG-INT-INT-ANYRD-2CLEAR_  ::= 0b00010000
  static INT-PIN-CFG-ACTL-FSYNC_            ::= 0b00001000
  static INT-PIN-CFG-FSYNC-INT-MODE-EN_     ::= 0b00000100
  static INT-PIN-CFG-BYPASS-EN_             ::= 0b00000010

  // Masks: $REGISTER-I2C-MST-CTRL_.
  static I2C-MULT-MST-EN_ ::= 0b10000000
  static I2C-MST-P-NSR_   ::= 0b00010000
  static I2C-MST-CLK_     ::= 0b00001111 // To use 400 kHz, MAX, it is recommended to set I2C-MST-CLK_ to 7.

  // Masks: REGISTER-I2C-SLVX-ADDR_ [x=0..4].
  static I2C-SLVX-ADDR-R_      ::= 0b10000000  // 1 = transfer is R for slave x
  static I2C-SLVX-ADDR-I2C-ID_ ::= 0b01111111  // PHY address of I2C slave x

  // Masks: REGISTER-I2C-SLVX-CTRL_ [x=0..4].
  static I2C-SLVX-CTRL-EN_      ::= 0b10000000  // Enable reading data from this slave at the sample rate and storing data at the first available EXT_SENS_DATA register, which is always EXT_SENS_DATA_00 for I 2C slave 0
  static I2C-SLVX-CTRL-BYTE-SW_ ::= 0b01000000  // 1 – Swap bytes when reading both the low and high byte of a word.
  static I2C-SLVX-CTRL-REG-DIS_ ::= 0b00100000  // Disables writing the register value - when set it will only read/write data.
  static I2C-SLVX-CTRL-GRP_     ::= 0b00010000  // Whether 16 bit byte reads are 00..01 or 01..02.
  static I2C-SLVX-CTRL-LENG_    ::= 0b00001111  // Number of bytes to be read from I2C slave X.

  // Masks: REGISTER-I2C-MST-DELAY-CTRL_.
  static I2C-MST-DELAY-ES-SHADOW_ ::= 0b10000000
  static I2C-MST-DELAY-SLV4-EN_   ::= 0b00010000
  static I2C-MST-DELAY-SLV3-EN_   ::= 0b00001000
  static I2C-MST-DELAY-SLV2-EN_   ::= 0b00000100
  static I2C-MST-DELAY-SLV1-EN_   ::= 0b00000010
  static I2C-MST-DELAY-SLV0-EN_   ::= 0b00000001

  // Register Map for AK09916.
  static REG-AK09916-DEV-ID_    ::= 0x01  // R 1 Device ID.
  static REG-AK09916-STATUS-1_  ::= 0x10  // R 1 Data status.
  static REG-AK09916-X-AXIS_    ::= 0x11  // R 2 X Axis LSB (MSB 0x12).  Signed int.
  static REG-AK09916-Y-AXIS_    ::= 0x13  // R 2 Y Axis LSB (MSB 0x14).  Signed int.
  static REG-AK09916-Z-AXIS_    ::= 0x15  // R 2 Y Axis LSB (MSB 0x16).  Signed int.
  static REG-AK09916-STATUS-2_  ::= 0x18  // R 1 Data status.
  static REG-AK09916-CONTROL-1_ ::= 0x30  // R 1 Control Settings.
  static REG-AK09916-CONTROL-2_ ::= 0x31  // R 1 Control Settings.
  static REG-AK09916-CONTROL-3_ ::= 0x32  // R 1 Control Settings.

  static AK09916-DEV-ID_          ::= 0b00001001 // Device ID should always be this.
  static AK09916-STATUS-1-DOR_    ::= 0b00000010 // Data Overrun.
  static AK09916-STATUS-1-DRDY_   ::= 0b00000001 // New data is ready.
  static AK09916-STATUS-2-HOFL_   ::= 0b00001000 // Hardware Overflow.
  static AK09916-STATUS-2-RSV28_  ::= 0b00010000 // Reserved for AKM.
  static AK09916-STATUS-2-RSV29_  ::= 0b00100000 // Reserved for AKM.
  static AK09916-STATUS-2-RSV30_  ::= 0b01000000 // Reserved for AKM.
  static AK09916-CONTROL-2-STEST_ ::= 0b00010000 // Self Test Mode.
  static AK09916-CONTROL-2-MODE4_ ::= 0b00001000 // Continuous Mode 4.
  static AK09916-CONTROL-2-MODE3_ ::= 0b00000110 // Continuous Mode 3.
  static AK09916-CONTROL-2-MODE2_ ::= 0b00000100 // Continuous Mode 2.
  static AK09916-CONTROL-2-MODE1_ ::= 0b00000010 // Continuous Mode 1.
  static AK09916-CONTROL-2-MODE0_ ::= 0b00000001 // Single Measurement Mode.
  static AK09916-CONTROL-2-OFF_   ::= 0b00000000 // Power down mode.
  static AK09916-CONTROL-3-SRST_  ::= 0b00000001 // Software Reset.

  static AK09916-I2C-ADDRESS ::= 0x0C  // Magnetometer I2C address when configured for bus bridging.

  // $write-register_ statics for bit width.  All 16 bit read/writes are LE.
  static WIDTH-8_ ::= 8
  static WIDTH-16_ ::= 16
  static DEFAULT-REGISTER-WIDTH_ ::= WIDTH-8_

  accel-sensitivity_/float := 0.0
  gyro-sensitivity_/float := 0.0

  reg_/Registers := ?
  logger_/log.Logger := ?

  constructor dev/Device --logger/log.Logger=log.default:
    reg_ = dev.registers
    logger_ = logger.with-name "icm20948"

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


  /**
  Enables I2C bypass such that AK09916 appears and is reachable on external I2C bus.

  Requires the ICM20948 device to be turned $on.

  This is largely for testing/debugging the outputs of the AK09916.  It is
    not used for any other reasons.  As the MCU must manage timing there is no
    synchronization with accel/gyro. The bus will conflict if it is
    misconfigured.  The mode prevents DMP fusion. This mode also prevents access
    to AK09916 data over SPI.
  */
  enable-i2c-bypass -> none:
    set-bank_ 0
    // Disable I2C Master:
    write-register_ REGISTER-USER-CTRL_ 0 --mask=USER-CTRL-I2C-MST-EN_
    // Enable bypass mux:
    write-register_ REGISTER-INT-PIN-CFG_ 1 --mask=INT-PIN-CFG-BYPASS-EN_

  /**
  Reads and optionally masks/parses register data. (Big-endian.)
  */
  read-register_ -> int
      bank/int
      register/int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false:
    assert: 0 <= bank <= 3
    assert: (width == WIDTH-8_) or (width == WIDTH-16_)

    if not mask: mask = (width == 8) ? 0xFF : 0xFFFF
    if not offset: offset = mask.count-trailing-zeros

    assert:
      if width == WIDTH-8_: (mask & ~0xff) == 0
      else: (mask & ~0xffff) == 0
    assert: mask != 0

    full-width := (offset == 0) and ((width == WIDTH-8_ and mask == 0xFF) or (width == WIDTH-16_ and mask == 0xFFFF))
    if signed and not full-width:
      throw "masked signed read not supported (need sign-extension by field width)"

    register-value/int? := null
    bank-mutex_.do:
      set-bank-p_ bank
      if width == WIDTH-8_:
        register-value = signed ? reg_.read-i8 register : reg_.read-u8 register
      else:
        register-value = signed ? reg_.read-i16-be register : reg_.read-u16-be register

    if full-width:
      return register-value

    return (register-value & mask) >> offset

  /**
  Writes register data - either masked or full register writes. (Big-endian.)
  */
  write-register_ -> none
      bank/int
      register/int
      value/int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false:
    assert: 0 <= bank <= 3
    assert: (width == WIDTH-8_) or (width == WIDTH-16_)

    if not mask: mask = (width == WIDTH-8_) ? 0xff : 0xffff
    if not offset: offset = mask.count-trailing-zeros

    // Check mask fits register width:
    assert:
      if width == WIDTH-8_: (mask & ~0xff) == 0
      else: (mask & ~0xffff) == 0

    // Determine if write is full width:
    full-width := (offset == 0) and ((width == WIDTH-8_ and mask == 0xff) or (width == WIDTH-16_ and mask == 0xffff))

    // For now don't accept negative numbers as masked writes.
    if signed and not full-width:
      throw "masked signed write not supported (encode to field bits first)"

    // Mask must fit within the register width:
    field-mask/int := mask >> offset
    assert: field-mask != 0

    // Check an unsigned write is actually > 0:
    if not signed:
      assert: value >= 0 and value <= field-mask
    else:
      if width == 8: assert: -128 <= value and value <= 127
      else: assert: -32768 <= value and value <= 32767

    bank-mutex_.do:
      set-bank-p_ bank

      // Full-width direct write:
      if full-width:
        if width == WIDTH-8_:
          signed ? reg_.write-i8 register value : reg_.write-u8 register value
        else:
          signed ? reg_.write-i16-be register value : reg_.write-u16-be register value
        return

      // Read Reg for modification:
      old-value/int := (width == WIDTH-8_) ? reg_.read-u8 register : reg_.read-u16-be register
      reg-mask/int := (width == WIDTH-8_) ? 0xFF : 0xFFFF
      new-value/int := (old-value & ~mask) | ((value & field-mask) << offset) & reg-mask

      // Write modified value:
      if width == WIDTH-8_:
        reg_.write-u8 register new-value
      else:
        reg_.write-u16-be register new-value
