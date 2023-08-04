// Copyright 2023 Matthieu Vigne

#include "nautilus_spi_interface.h"
#include "fw/stm32g4_async_uart.h"
#include "PeripheralPins.h"



using namespace nautilus;

#define MESSAGE_LENGTH 8

uint8_t rxBuffer[MESSAGE_LENGTH] = {0};
uint8_t txBuffer[MESSAGE_LENGTH] = {0}; // Data that is about to be transmitted
uint8_t posInMessage = 0;   // Current position in the message


// SPI3 configuration
// Slave mode 0
#define SPICR1 0b0000000001111000
// 8 bits, generate RXNEIE on half-full buffer (8 bits)
#define SPICR2 0b0001011101001000


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

    // Rising edge: SPI transfer done, reset SPI to clear TX buffer.
    if (GPIOA->IDR & 0x8000)  // Read pin A15
    {
        // Reconfigure SPI
        RCC->APB1RSTR1 |= (1 << 15);

        RCC->APB1RSTR1 &= ~(1 << 15);
        SPI3->CR1 = SPICR1;
        SPI3->CR2 = SPICR2;

    }
    else
    {
        // Falling edge of CS: a SPI transfer is about to start.

        // Reset position to 0
        posInMessage = 0;

        // Fill txBuffer with first three bytes
        // First two bytes: encoder
        // Third byte: status
        txBuffer[0] = (timeCnt >> 8) & 0xFF;
        txBuffer[1] = timeCnt & 0xFF;
        txBuffer[2] = 0x42;

        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[0];
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[1];
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[2];
    }
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

    SPI3->CR1 = SPICR1;
    SPI3->CR2 = SPICR2;
    // SPI config: see https://www.youtube.com/watch?v=_RBXQLPGr7Q
    // Enable interrupt on SPI3 RX
    NVIC->ISER[1] |= (1u << 19);



    // Interrupt on slave-select edges.

    // Enable line 15
    EXTI->IMR1 |= (1u << 15);

    // Rising and falling edge interrupts
    EXTI->FTSR1 |= (1u << 15);
    EXTI->RTSR1 |= (1u << 15);

    // Line 15: use pin A15
    SYSCFG->EXTICR[3] &= ~(0b1111 << 8);

    // Enable EXTI15_10: NVIC 40
    NVIC->ISER[1] |= (1u << 8);
}
