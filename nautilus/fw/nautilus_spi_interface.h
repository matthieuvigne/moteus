// Copyright 2023 Matthieu Vigne

#ifndef NAUTILUS_SPI_INTERFACE_H
#define NAUTILUS_SPI_INTERFACE_H

#include <optional>

#include "mbed.h"
#include "hal/spi_api.h"

#include "fw/moteus_controller.h"
#include "fw/bldc_servo.h"
#include "fw/drv8323.h"

namespace nautilus {


enum SPIRegister {
    currentMode       = 0x00,       // uint8_t
    faultCode         = 0x01,       // uint32_t
    drvStatus         = 0x02,       // uint32_t
    drvConfigError    = 0x03,       // uint32_t

    measuredPosition  = 0x10,       // float32_t
    measuredVelocity  = 0x11,       // float32_t
    measuredIQ        = 0x12,       // float32_t
    measuredIPhaseA   = 0x13,       // float32_t
    measuredIPhaseB   = 0x14,       // float32_t
    measuredIPhaseC   = 0x15,       // float32_t
    measuredUPhaseA   = 0x16,       // float32_t
    measuredUPhaseB   = 0x17,       // float32_t
    measuredUPhaseC   = 0x18,       // float32_t
    measuredMotTemp   = 0x19,       // float32_t
    measuredDriveTemp = 0x1A,       // float32_t
    measuredUBat      = 0x1B,       // float32_t

    targetPosition     = 0x20,       // float32_t
    targetVelocity     = 0x21,       // float32_t
    targetIQ           = 0x22,       // float32_t

    rawEncoderPos      = 0x30,       // uint16_t
    encoderOrientation = 0x31,       // uint8_t
    commutationOffset  = 0x32,       // uint16_t
    nbrOfPoles         = 0x33,       // uint16_t

    currentLoopKp      = 0x40,       // float32_t
    currentLoopKi      = 0x41,       // float32_t
    currentLoopIntMax  = 0x42,       // float32_t

    velocityLoopKp      = 0x43,       // float32_t
    velocityLoopKi      = 0x44,       // float32_t
    velocityLoopIntMax  = 0x45,       // float32_t

    positionLoopKp      = 0x46,       // float32_t
    positionLoopKd      = 0x47,       // float32_t
    positionLoopKi      = 0x48,       // float32_t
    positionLoopIntMax  = 0x49,       // float32_t

    motorMaxCurrent      = 0x50,       // float32_t
    motorMaxTemperature  = 0x51,       // float32_t
    driverMaxTemperature = 0x52,       // float32_t
    commTimeout          = 0x53,       // uint16_t
};

enum SPICommand {
    regWrite = 0x01,
    regRead  = 0x02,
    commutation  = 0x03,
    storeToPersistentMemory  = 0x04,
    stop  = 0x05,
};

// Answer read query, return the 4 bytes to send.
uint32_t processReadCommand(uint8_t const& registerAddress);

// Handle write query.
void processWriteCommand(uint8_t const& registerAddress, uint32_t const& registerValue);

class NautilusSPIInterface {
 public:
    NautilusSPIInterface(moteus::BldcServo* bldc,
                         moteus::BldcServo::CommandData* command,
                         moteus::Drv8323* drv8323,
                         mjlib::micro::PersistentConfig* config);

    void setup();
    void poll();
};

}

#endif
