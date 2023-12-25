
# How to compile and flash the firmware

## Installing dependencies

```sudo dnf install patch openocd arm-none-eabi-binutils-cs```

## Building the firmware

The firmware can be built by the following command:

```./tools/bazel test --config=target //fw:nautilus```

The bootloader also needs to be built (typically only once):

```./tools/bazel test --config=target //fw:can_bootloader ```

## Flashing

## Setting STM32 option bytes

Pin PB8 (boot) is used for I2C, thus pulled up. Hence, for the firmware to start running, the STM32 option bytes must be changed. This requires the software STM32CubeProgrammer, and the following configuration: nSWBOOT0 unchecked, nBOOT0 checked.

## Flashing

Simply run the python script: `python fw/flash.py`
