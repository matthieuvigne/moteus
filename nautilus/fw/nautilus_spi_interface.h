// Copyright 2023 Matthieu Vigne

#ifndef NAUTILUS_SPI_INTERFACE_H
#define NAUTILUS_SPI_INTERFACE_H

#include <optional>

#include "mbed.h"

// #include "stm32g4xx_hal_spi.h"
#include "hal/spi_api.h"

namespace nautilus {

#define MESSAGE_SIZE 2

class NautilusSPIInterface {
 public:
    NautilusSPIInterface();

    void setup();
    void poll();

private:
    spi_t spi_;
    SPI_HandleTypeDef hspi_;

    uint8_t txBuffer_[MESSAGE_SIZE];
    uint8_t rxBuffer_[MESSAGE_SIZE];
};

}

#endif
