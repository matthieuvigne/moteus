// Copyright 2023 Matthieu Vigne

#include "nautilus_spi_interface.h"
#include "fw/stm32g4_async_uart.h"
#include "PeripheralPins.h"



using namespace nautilus;

#define MESSAGE_LENGTH 8

uint8_t rxBuffer[MESSAGE_LENGTH] = {0};
uint8_t txBuffer[MESSAGE_LENGTH] = {0}; // Data that is about to be transmitted
uint8_t posInMessage = 0;   // Current position in the message



// Blink led - debug
USART_TypeDef* debug_uart_ = nullptr;
uint16_t timeCnt = 0;
static DigitalOut led1_{PA_5, 1};


NautilusSPIInterface::NautilusSPIInterface()
{
    // Empty on purpose
}

void NautilusSPIInterface::poll()
{

    debug_uart_->TDR = 0x20;

    for (int i = 0; i < posInMessage; i++)
        debug_uart_->TDR = rxBuffer[i];

    timeCnt++;
}

// Avoid C++ name-mangling
extern "C" {

void SPI3_IRQHandler(void)
{
    // SPI3->DR = txBuffer[posInMessage];

    // Read data
    rxBuffer[posInMessage] = SPI3->DR;
    if (posInMessage < MESSAGE_LENGTH - 1)
        posInMessage ++;

    if (posInMessage == 3)
    {
        // We have received enough info from the user to send the next 4 bytes
        txBuffer[3] = rxBuffer[0];
        txBuffer[4] = rxBuffer[1];
        txBuffer[5] = 0x04;
        txBuffer[6] = 0x02;
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[3];
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[4];
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[5];
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[6];
    }
    if (posInMessage == 4)
    {
        // Send last element: CRC
        uint8_t const crc = txBuffer[0] + txBuffer[1] + txBuffer[2] + txBuffer[3] +\
                            txBuffer[4] + txBuffer[5] + txBuffer[6];
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) crc;
    }
}


void EXTI15_10_IRQHandler(void)
{
    /* Clear interrupt flag*/
    EXTI->PR1 |= (1 << 15);

    // Falling edge of CS: a SPI transfer is about to start.



    // Reset position to 0
    posInMessage = 0;

    // Fill txBuffer - note that we need to send 4 bytes to clear the FIFO.
    // The last one will actually not be sent...

    // First two bytes: encoder
    // Third byte: status
    txBuffer[0] = (timeCnt >> 8) & 0xFF;
    txBuffer[1] = timeCnt & 0xFF;
    txBuffer[2] = 0x42;

    * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[0];
    * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[1];
    * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[2];
    // * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[3];

    debug_uart_->TDR = 0xbb;

    // Debug: we are in ISR
    // debug_uart_->TDR = 0xAA;

}

}


void NautilusSPIInterface::setup()
{

    const auto uart = pinmap_peripheral(
        PC_10_ALT0, PinMap_UART_TX);
    debug_uart_ = reinterpret_cast<USART_TypeDef*>(uart);


    led1_ = 0;
    spi_init(&spi_,
              PB_5_ALT0, // MOSI
              PB_4_ALT0, // MISO
              PB_3_ALT0, // SCK
              PA_15_ALT0); // CS

    spi_format(&spi_, 8, 0, 1); // 8 bits, mode 8, slave

    // SPI config: see https://www.youtube.com/watch?v=_RBXQLPGr7Q
    // Enable interrupt on SPI3 RX
    NVIC->ISER[1] |= (1u << 19);

    // Enable data recieved interrupt, work in 8-bits mode
    SPI3->CR2 |= (1u << 6);
    SPI3->CR2 &= ~(1u << 7);
    SPI3->CR2 &= ~(1u << 5);
    SPI3->CR2 &= ~(1u << 5);


    // Interrupt on rising edge of slave-select, to reset the counter.

    // Enable line 15
    EXTI->IMR1 |= (1u << 15);

    // Falling edge only
    EXTI->FTSR1 |= (1u << 15);
    EXTI->RTSR1 &= ~(1u << 15);

    // Line 15: use pin A15
    SYSCFG->EXTICR[3] &= ~(0b1111 << 8);

    // Enable EXTI15_10: NVIC 40
    NVIC->ISER[1] |= (1u << 8);

}
