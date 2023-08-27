# Nautilus motor driver protocol

Communication with the `nautilus` board is done using SPI. A fixed-size frame of 8 bytes is required for a complete transaction:

Input frame:
 - Byte 0: command byte 0
 - Byte 1: command byte 1 (typically register address)
 - Byte 2-5: data
 - Byte 6: unused
 - Byte 7: checksum

Output frame:
 - Byte 0-1: secondary encoder reading
 - Byte 2: status byte
 - Byte 3-6: data
 - Byte 7: checksum

The checksum is simply the sum of the previous 6 bytes of the message.

**Note: for now, `nautilus` only supports SPI mode 1.**

## Available commands

### 0x01 <reg>: Register write

Write data to the register <reg>. If the register is non-writable, this has no effect

### 0x02 <reg>: Register read

Read data from register <reg>.

## 0x03 0x00: Perform calibration

Enter calibration mode: in voltage FOC, perform a full sweep of an electrical phase at the specified
velocity and power.

 - data[0-1]: uint16, duration of the sweep in ms.
 - data[2-3]: uint16, amplitude of the voltage in steps of 0.1%

## 0x03 0x01: Store calibration results

Store the calibration result into persistent memory.


## Register list

Address | Name                        | R/W | Persistent ? | Data type | Description
------- | ------------------          | --- | ------------ | --------- | -----------
0x00    | Current mode                | RW  | No           | uint32_t  | Current working mode, see details below
0x01    | Last error                  | RO  | No           | uint32_t  | Last error to have occured

0x10    | Measured position           | RO  | No           | float32_t | Measured position, rad
0x11    | Measured speed              | RO  | No           | float32_t | Measured speed, rad/s
0x12    | Measured quadrature current | RO  | No           | float32_t | Measured quadrature current, A
0x13    | Measured current in phase A | RO  | No           | float32_t | Measured current in phase A, A
0x14    | Measured current in phase B | RO  | No           | float32_t | Measured current in phase B, A
0x15    | Measured current in phase C | RO  | No           | float32_t | Measured current in phase C, A
0x16    | Measured voltage in phase A | RO  | No           | float32_t | Measured voltage in phase A, V
0x17    | Measured voltage in phase B | RO  | No           | float32_t | Measured voltage in phase B, V
0x18    | Measured voltage in phase C | RO  | No           | float32_t | Measured voltage in phase C, V
0x19    | Measured motor temperature  | RO  | No           | float32_t | Measured motor temperature, C
0x1A    | Measured mosfet temperature | RO  | No           | float32_t | Measured mosfet temperature, C
0x1B    | Measured battery voltage    | RO  | No           | float32_t | Battery voltage, V

0x20    | Target position             | RW  | No           | float32_t | Target position, rad
0x21    | Target speed                | RW  | No           | float32_t | Target speed, rad/s
0x22    | Target quadrature current   | RW  | No           | float32_t | Target quadrature current, A

0x30    | Raw encoder position        | RO  | No           | uint16_t | Raw encoder position, ticks
0x31    | Encoder orientation         | RW  | Yes          | uint8_t  | Encoder orientation with respect to motor
0x33    | Commutation offset          | RW  | Yes          | uint16_t | Commutation offset, ticks

0x40    | Current loop Kp             | RW  | Yes          | float32_t | Current loop Kp, V/A
0x41    | Current loop Ki             | RW  | Yes          | float32_t | Current loop Ki, Vs/A
0x42    | Current loop max integral   | RW  | Yes          | float32_t | Current loop max integral value, V

0x43    | Velocity loop Kp            | RW  | Yes          | float32_t | Velocity loop Kp, A/rad/s
0x44    | Velocity loop Ki            | RW  | Yes          | float32_t | Velocity loop Ki, As/rad/s
0x45    | Velocity loop max integral  | RW  | Yes          | float32_t | Velocity loop max integral value, A

0x50    | Motor max current           | RW  | Yes          | float32_t | Maximum motor current, A
0x51    | Motor max temperature       | RW  | Yes          | float32_t | Maximum motor temperature, deg C
0x52    | Mosfet max temperature      | RW  | Yes          | float32_t | Maximum mosfet temperature, deg C
0x53    | Communication timeout delay | RW  | Yes          | uint16_t  | Communication timeout limit, ms


## Register details

### 0x00: Current mode

List mode of operations:

 - 0x00: power off
 - 0x01: brake
 - 0x02: voltage FOC (commutation)
 - 0x03: current mode
 - 0x04: velocity mode

 ### 0x01: Last error

 List of errors: TODO