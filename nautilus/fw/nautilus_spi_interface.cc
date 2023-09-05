// Copyright 2023 Matthieu Vigne
#include "nautilus_spi_interface.h"
#include "fw/stm32g4_async_uart.h"
#include "PeripheralPins.h"

#define M_PI       3.14159265358979323846f

using namespace nautilus;

#define MESSAGE_LENGTH 8

uint8_t rxBuffer[MESSAGE_LENGTH] = {0};
uint8_t txBuffer[MESSAGE_LENGTH] = {0}; // Data that is about to be transmitted
uint8_t posInMessage = 0;   // Current position in the message


moteus::BldcServo::CommandData* command;
moteus::BldcServo* bldc;

// SPI3 configuration
// Slave mode 3
#define SPICR1 0b0000000001111010

// Mode 2: 10
// Mode 1: 00 - but doesn't work for read (1-bit shift)
// Mode 0/3 don't work for some reason.

// 8 bits, generate RXNEIE on half-full buffer (8 bits)
#define SPICR2 0b0001011101001000


// Blink led - debug
USART_TypeDef* debug_uart_ = nullptr;
uint16_t timeCnt = 0;




/*static DigitalOut led1_(PF_0, 1);
static bool isOn = true;

isOn = !isOn;
if (isOn)
    led1_ = 1;
else
    led1_ = 0;
*/

NautilusSPIInterface::NautilusSPIInterface(moteus::BldcServo* bldcIn,
                                           moteus::BldcServo::CommandData* commandIn)
{
    command = commandIn;
    bldc = bldcIn;
}

void NautilusSPIInterface::poll()
{
    timeCnt++;
}

uint32_t fToUInt(float const& f)
{
    union result{
        float f;
        uint32_t i;
    };
    union result res;
    res.f = f;
    return res.i;
}

uint32_t nautilus::processReadCommand(uint8_t const& registerAddress)
{

    switch ((registerAddress))
    {
    case SPIRegister::currentMode: return bldc->status().mode;
    case SPIRegister::faultCode: return static_cast<uint32_t>(bldc->status().fault);
    case SPIRegister::measuredPosition: return fToUInt(static_cast<float>(bldc->motor_position().sources[0].filtered_value / bldc->motor_position_config()->sources[0].cpr * 2 * M_PI));
    case SPIRegister::measuredVelocity: return bldc->motor_position().sources[0].raw;
    case SPIRegister::measuredIQ: return bldc->motor_position().sources[0].nonce;
    case SPIRegister::measuredIPhaseA: return 0;
    case SPIRegister::measuredIPhaseB: return 0;
    case SPIRegister::measuredIPhaseC: return 0;
    case SPIRegister::measuredUPhaseA: return 0;
    case SPIRegister::measuredUPhaseB: return 0;
    case SPIRegister::measuredUPhaseC: return 0;
    case SPIRegister::measuredMotTemp: return 0;
    case SPIRegister::measuredDriveTemp: return 0;
    case SPIRegister::measuredUBat: return 0;
    case SPIRegister::targetPosition: return 0;
    case SPIRegister::targetVelocity: return 0;
    case SPIRegister::targetIQ: return 0;
    case SPIRegister::rawEncoderPos: return 0;
    case SPIRegister::encoderOrientation: return 0;
    case SPIRegister::commutationOffset: return 0;
    case SPIRegister::currentLoopKp: return 0;
    case SPIRegister::currentLoopKI: return 0;
    case SPIRegister::currentLoopIntMax: return 0;
    case SPIRegister::velocityLoopKp: return 0;
    case SPIRegister::velocityLoopKI: return 0;
    case SPIRegister::velocityLoopIntMax: return 0;
    case SPIRegister::motorMaxCurrent: return 0;
    case SPIRegister::motorMaxTemperature: return 0;
    case SPIRegister::driverMaxTemperature: return 0;
    case SPIRegister::commTimeout: return 0;
    default: return 0;
    }
}

// Handle write query.
void nautilus::processWriteCommand(uint8_t const& registerAddress, uint32_t const& registerValue)
{
    // Empty for now
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
        // We have received enough info from the user to know what to do.
        uint32_t data = 0;
        if (rxBuffer[0] == static_cast<uint8_t>(SPICommand::regRead))
        {
            // Handle read request
            data = nautilus::processReadCommand(rxBuffer[1]);
        }

        // Send next 4 bytes.
        txBuffer[3] = (data >> 24) & 0xFF;
        txBuffer[4] = (data >> 16) & 0xFF;
        txBuffer[5] = (data >> 8) & 0xFF;
        txBuffer[6] = (data) & 0xFF;
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
        txBuffer[2] = bldc->status().mode;

        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[0];
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[1];
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[2];
    }
}

}


void NautilusSPIInterface::setup()
{
    // Clear oscillator config
    const auto uart = pinmap_peripheral(
        PC_10_ALT0, PinMap_UART_TX);
    debug_uart_ = reinterpret_cast<USART_TypeDef*>(uart);

    // led1_ = 0;
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
