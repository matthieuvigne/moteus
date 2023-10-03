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
moteus::Drv8323* drv8323;
mjlib::micro::PersistentConfig* persistent_config;

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
int errorCnt = 0;

uint32_t timeoutMs = 100;   // Default timeout
uint32_t timeoutCounter = 0;


static DigitalOut statusLed(PF_0, 1);
static DigitalOut errorLed(PF_1, 1);

/*static DigitalOut statusLed(PF_0, 1);
static bool isOn = true;

isOn = !isOn;
if (isOn)
    led1_ = 1;
else
    led1_ = 0;
*/

NautilusSPIInterface::NautilusSPIInterface(moteus::BldcServo* bldcIn,
                                           moteus::BldcServo::CommandData* commandIn,
                                           moteus::Drv8323* drv8323In,
                                           mjlib::micro::PersistentConfig* config)
{
    command = commandIn;
    bldc = bldcIn;
    drv8323 = drv8323In;
    persistent_config = config;
}

void NautilusSPIInterface::poll()
{
    // Led blink: blink while controller is not active, otherwise remain on.
    timeCnt++;
    if (timeCnt > 1000)
        timeCnt = 0;

    if (bldc->status().mode > 0)
        timeCnt = 1000;
    statusLed = !(timeCnt > 900);

     if (errorCnt > 0)
         errorCnt --;
     errorLed = !(errorCnt > 0);

     if (timeoutCounter > 0)
     {
        timeoutCounter--;
        if (timeoutCounter == 0)
        {
            command->mode = moteus::kStopped;
            bldc->Command(*command);
        }
     }
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

float UIntToF(uint32_t const& i)
{
    union result{
        float f;
        uint32_t i;
    };
    union result res;
    res.i = i;
    return res.f;
}

uint32_t nautilus::processReadCommand(uint8_t const& registerAddress)
{

    switch (registerAddress)
    {
    case SPIRegister::currentMode:          return bldc->status().mode;
    case SPIRegister::faultCode:            return static_cast<uint32_t>(bldc->status().fault);
    case SPIRegister::drvStatus:            return (static_cast<uint32_t>(drv8323->status()->fsr1) << 16) + static_cast<uint32_t>(drv8323->status()->fsr2);
    case SPIRegister::drvConfigError:       return static_cast<uint32_t>(drv8323->status()->fault_config);
    case SPIRegister::measuredPosition:     return fToUInt(static_cast<float>(bldc->motor_position().sources[0].filtered_value / bldc->motor_position_config()->sources[0].cpr * 2 * M_PI));
    case SPIRegister::measuredVelocity:     return fToUInt(static_cast<float>(bldc->motor_position().sources[0].velocity / bldc->motor_position_config()->sources[0].cpr * 2 * M_PI));
    case SPIRegister::measuredIQ:           return fToUInt(bldc->status().q_A);
    case SPIRegister::measuredIPhaseA:      return fToUInt(bldc->status().cur1_A);
    case SPIRegister::measuredIPhaseB:      return fToUInt(bldc->status().cur2_A);
    case SPIRegister::measuredIPhaseC:      return fToUInt(bldc->status().cur3_A);
    case SPIRegister::measuredUPhaseA:      return fToUInt(0.0);    // Not implemented for now
    case SPIRegister::measuredUPhaseB:      return fToUInt(0.0);    // Not implemented for now
    case SPIRegister::measuredUPhaseC:      return fToUInt(0.0);    // Not implemented for now
    case SPIRegister::measuredMotTemp:      return fToUInt(bldc->config().enable_motor_temperature ? bldc->status().motor_temp_C : -1);
    case SPIRegister::measuredDriveTemp:    return fToUInt(bldc->status().fet_temp_C);
    case SPIRegister::measuredUBat:         return fToUInt(bldc->status().bus_V);
    case SPIRegister::targetPosition:       return fToUInt(command->position);
    case SPIRegister::targetVelocity:       return fToUInt(command->velocity);
    case SPIRegister::targetIQ:             return fToUInt(command->i_q_A);
    case SPIRegister::rawEncoderPos:        return bldc->motor_position().sources[0].raw;
    case SPIRegister::encoderOrientation:   return bldc->motor()->phase_invert;
    case SPIRegister::commutationOffset:    return fToUInt(bldc->motor()->offset[0]);
    case SPIRegister::nbrOfPoles:           return bldc->motor()->poles;
    case SPIRegister::currentLoopKp:        return fToUInt(bldc->config().pid_dq.kp);
    case SPIRegister::currentLoopKi:        return fToUInt(bldc->config().pid_dq.ki);
    case SPIRegister::currentLoopIntMax:    return 0;   // Not available, this loop has no anti-windup term.
    case SPIRegister::velocityLoopKp:       return fToUInt(bldc->config().pid_velocity.kp);
    case SPIRegister::velocityLoopKi:       return fToUInt(bldc->config().pid_velocity.ki);
    case SPIRegister::velocityLoopIntMax:   return fToUInt(bldc->config().pid_velocity.ilimit);
    case SPIRegister::positionLoopKp:       return fToUInt(bldc->config().pid_position.kp);
    case SPIRegister::positionLoopKd:       return fToUInt(bldc->config().pid_position.kd);
    case SPIRegister::positionLoopKi:       return fToUInt(bldc->config().pid_position.ki);
    case SPIRegister::positionLoopIntMax:   return fToUInt(bldc->config().pid_position.ilimit);
    case SPIRegister::motorMaxCurrent:      return fToUInt(bldc->config().max_current_A);
    case SPIRegister::motorMaxTemperature:  return fToUInt(bldc->config().enable_motor_temperature ? bldc->config().motor_fault_temperature : -1);
    case SPIRegister::driverMaxTemperature: return fToUInt(bldc->config().fault_temperature);
    case SPIRegister::commTimeout:          return timeoutMs;
    default:                                return 0;
    }
}

// Handle write query.
void nautilus::processWriteCommand(uint8_t const& registerAddress, uint32_t const& registerValue)
{
    // Basic register settings
    float const fRegisterValue =  UIntToF(registerValue);
    switch (registerAddress)
    {
        case SPIRegister::currentLoopKp:        bldc->config().pid_dq.kp = fRegisterValue; break;
        case SPIRegister::currentLoopKi:        bldc->config().pid_dq.ki = fRegisterValue; break;
        case SPIRegister::velocityLoopKp:       bldc->config().pid_velocity.kp = fRegisterValue; break;
        case SPIRegister::velocityLoopKi:       bldc->config().pid_velocity.ki = fRegisterValue; break;
        case SPIRegister::velocityLoopIntMax:   bldc->config().pid_velocity.ilimit = fRegisterValue; break;
        case SPIRegister::positionLoopKp:       bldc->config().pid_position.kp = fRegisterValue; break;
        case SPIRegister::positionLoopKd:       bldc->config().pid_position.kd = fRegisterValue; break;
        case SPIRegister::positionLoopKi:       bldc->config().pid_position.ki = fRegisterValue; break;
        case SPIRegister::positionLoopIntMax:   bldc->config().pid_position.ilimit = fRegisterValue; break;
        case SPIRegister::commTimeout:          timeoutMs = registerValue; break;

        case SPIRegister::encoderOrientation:   bldc->motor()->phase_invert = registerValue & 0xFF; break;
        case SPIRegister::commutationOffset:
            for (int i = 0; i < 64; i++)
                bldc->motor()->offset[i] = fRegisterValue;
            break;
        case SPIRegister::targetPosition:
            command->mode = moteus::kPosition;
            command->position = fRegisterValue;

            timeoutCounter = timeoutMs;
            bldc->Command(*command);
            break;
        case SPIRegister::targetVelocity:
            command->mode = moteus::kVelocity;
            command->velocity = fRegisterValue;

            timeoutCounter = timeoutMs;
            bldc->Command(*command);
            break;
        case SPIRegister::targetIQ:
            command->mode = moteus::kCurrent;
            command->i_q_A = fRegisterValue;

            timeoutCounter = timeoutMs;
            bldc->Command(*command);
            break;

        default:
            break;
    }
}


// Avoid C++ name-mangling
extern "C" {

void SPI3_IRQHandler(void)
{
    // Read data
    rxBuffer[posInMessage] = SPI3->DR;

    if (posInMessage == MESSAGE_LENGTH - 1)
    {
        // Check CRC
        uint8_t const crc = rxBuffer[0] + rxBuffer[1] + rxBuffer[2] + rxBuffer[3] +\
                            rxBuffer[4] + rxBuffer[5] + rxBuffer[6];
        if (crc != rxBuffer[7])
        {
            errorCnt = 500;
            rxBuffer[0] = 0;
        }
        else
        {
            // Check if it's a write command.
            if (rxBuffer[0] == static_cast<uint8_t>(SPICommand::regWrite))
            {
                uint32_t const regValue = (rxBuffer[2] << 24) + (rxBuffer[3] << 16) + (rxBuffer[4] << 8) + rxBuffer[5];
                processWriteCommand(rxBuffer[1], regValue);
            }
            else if (rxBuffer[0] == static_cast<uint8_t>(SPICommand::commutation))
            {
                command->mode = moteus::kVoltageFoc;

                union result{
                    float f;
                    uint32_t i;
                };
                uint32_t const theta = ((rxBuffer[1] << 24) + (rxBuffer[2] << 16) + (rxBuffer[3] << 8) + (rxBuffer[4]));
                command->theta = UIntToF(theta);
                command->theta_rate = 0;
                uint16_t const voltageRatio = (rxBuffer[5] << 8) + rxBuffer[6];
                command->voltage = (static_cast<float_t>(voltageRatio) / 65535.0f) * bldc->status().bus_V;

                timeoutCounter = timeoutMs;
                bldc->Command(*command);
            }
            else if (rxBuffer[0] == static_cast<uint8_t>(SPICommand::storeToPersistentMemory))
            {
                mjlib::micro::CommandManager::Response response;
                // Note: for this to work, PersistentConfig.Write must be exposed.
                // TODO FIXME: calling this here cause the uC to hang. But the config is still saved...
                persistent_config->Write(response);
            }
            else if (rxBuffer[0] == static_cast<uint8_t>(SPICommand::stop))
            {
                command->mode = moteus::kStopped;
                bldc->Command(*command);
            }
        }
        // Clear buffer state to avoid a loop.
        rxBuffer[0] = 0;
    }

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
        uint16_t encVal = bldc->aux2().i2c.devices[0].value;
        // Check valid reading
        if (bldc->aux2().i2c.devices[0].ams_diag == 0x01)
            encVal |= 1 << 15;
        // Third byte: status
        txBuffer[0] = (encVal >> 8) & 0xFF;
        txBuffer[1] = encVal & 0xFF;
        txBuffer[2] = bldc->status().mode;

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

    // led1_ = 0;
    spi_t spi;
    spi_init(&spi,
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
