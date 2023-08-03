// Copyright 2023 Matthieu Vigne

#include "nautilus_spi_interface.h"
#include "fw/stm32g4_async_uart.h"
#include "PeripheralPins.h"



using namespace nautilus;

USART_TypeDef* debug_uart_ = nullptr;

#define MESSAGE_LENGTH 8

uint8_t rxBuffer[MESSAGE_LENGTH] = {0};
uint8_t txBuffer[MESSAGE_LENGTH] = {0}; // Data that is about to be transmitted
uint8_t posInMessage = 0;   // Current position in the message


uint16_t timeCnt = 0;

// // Blink led
// static int counter = 0;
// static bool isSet = false;
static DigitalOut led1_{PA_5, 1};
// counter ++;
// if (counter % 250 == 0)
// {
//     isSet = !isSet;
//     if (isSet)
//     led1_ = 1;
//     else
//     led1_ = 0;
// }

NautilusSPIInterface::NautilusSPIInterface()
{
    // Empty on purpose
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
    if (reinterpret_cast<uint32_t>(hspi->Instance) == SPI_3)
    {
        // led1_ = 1;
    }
    for (int i = 0; i < hspi->TxXferSize; i++)
    {
        hspi->pTxBuffPtr[i] = hspi->pRxBuffPtr[i];
        debug_uart_->TDR = hspi->pRxBuffPtr[i];
    }

    // Start next transaction
    HAL_SPI_TransmitReceive_IT(hspi, hspi->pTxBuffPtr, hspi->pRxBuffPtr, 2);

    debug_uart_->TDR = 0xaa;
}

void NautilusSPIInterface::poll()
{

    debug_uart_->TDR = 0x20;

    timeCnt++;
}

// Avoid C++ name-mangling
extern "C" {

void SPI3_IRQHandler(void)
{
    // Read data
    rxBuffer[posInMessage] = SPI3->DR;
    debug_uart_->TDR = posInMessage;
    debug_uart_->TDR = rxBuffer[posInMessage];
    // Reply with next byte from tx buffer
    SPI3->DR = txBuffer[posInMessage];
    if (posInMessage < MESSAGE_LENGTH - 1)
        posInMessage ++;
    debug_uart_->TDR = txBuffer[posInMessage];
}


void EXTI15_10_IRQHandler(void)
{
    // Falling edge of CS: a SPI transfer is about to start.

    // Reset position to 0
    posInMessage = 0;

    // Fill txBuffer
    txBuffer[0] = (timeCnt >> 8) & 0xFF;
    txBuffer[1] = timeCnt & 0xFF;

    // Send first byte
    // SPI3->DR = txBuffer[posInMessage];

    // Debug: we are in ISR
    // debug_uart_->TDR = 0xAA;
    /* Clear interrupt flag*/
    EXTI->PR1 |= (1 << 15);
}

}


void NautilusSPIInterface::setup()
{

    const auto uart = pinmap_peripheral(
        PC_10_ALT0, PinMap_UART_TX);
    debug_uart_ = reinterpret_cast<USART_TypeDef*>(uart);


    led1_ = 0;
    // __HAL_RCC_GPIOB_CLK_ENABLE();
    // __HAL_RCC_SPI3_CLK_ENABLE();
    spi_init(&spi_,
              PB_5_ALT0, // MOSI
              PB_4_ALT0, // MISO
              PB_3_ALT0, // SCK
              PA_15_ALT0); // CS

    spi_format(&spi_, 8, 0, 1); // 8 bits, mode 8, slave

    // SPI config: see https://www.youtube.com/watch?v=_RBXQLPGr7Q
    // Enable interrupt on SPI3 RX
    NVIC->ISER[1] |= (1u << 19);

    // Enable data recieved interrupt
    SPI3->CR2 |= (1u << 6);

    // Interrupt on rising edge of slave-select, to reset the counter.

    // Enable line 15
    EXTI->IMR1 |= (1u << 15);

    // Falling edge
    EXTI->FTSR1 |= (1u << 15);

    // Line 15: use pin A15
    SYSCFG->EXTICR[3] &= ~(0b1111 << 8);

    // Enable EXTI15_10: NVIC 40
    NVIC->ISER[1] |= (1u << 8);

}
