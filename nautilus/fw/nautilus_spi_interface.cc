// Copyright 2023 Matthieu Vigne
#include "nautilus_spi_interface.h"
#include "fw/stm32g4_async_uart.h"
#include "PeripheralPins.h"

using namespace nautilus;

#define FIRMWARE_VERSION 101

#define TWO_PI       6.283185307179586f

#define MESSAGE_LENGTH 8

uint8_t rxBuffer[MESSAGE_LENGTH] = {0};
uint8_t txBuffer[MESSAGE_LENGTH] = {0}; // Data that is about to be transmitted
uint8_t posInMessage = 0;   // Current position in the message


moteus::BldcServo::CommandData* command;
moteus::BldcServo* bldc;
moteus::Drv8323* drv8323;
// mjlib::micro::PersistentConfig* persistent_config;
mjlib::micro::EventQueue* config_queue;
mjlib::micro::AsyncStream* config_stream;

// SPI3 configuration
// Slave mode 3
#define SPICR1 0b0000000001111010

// Mode 2: 10
// Mode 1: 00 - but doesn't work for read (1-bit shift)
// Mode 0/3 don't work for some reason.

// 8 bits, generate RXNEIE on half-full buffer (8 bits)
#define SPICR2 0b0001011101001000

float const ZERO = 0.0;
float const MINUS_ONE = -1.0;

// Led control
uint16_t timeCnt = 0;
int errorCnt = 0;

uint32_t timeoutMs = 100;   // Default timeout
uint32_t timeoutCounter = 0;


static DigitalOut statusLed(PF_0, 1);
static DigitalOut errorLed (PF_1, 1);

static DigitalOut debugLed1(PC_14, 1);
static DigitalOut debugLed2(PC_15, 1);

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
                                           mjlib::micro::EventQueue* cqueue,
                                           mjlib::micro::AsyncStream* config)
{
    command = commandIn;
    bldc = bldcIn;
    drv8323 = drv8323In;
    config_queue = cqueue;
    config_stream = config;
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


void __attribute__ ((optimize("O3"))) nautilus::processReadCommand(uint8_t const& registerAddress, uint32_t&output)
{
    float res;
    switch (registerAddress)
    {
    case SPIRegister::currentMode:          output = bldc->status().mode; break;
    case SPIRegister::faultCode:            output = static_cast<uint32_t>(bldc->status().fault); break;
    case SPIRegister::drvStatus:            output = (static_cast<uint32_t>(drv8323->status()->fsr1) << 16) + static_cast<uint32_t>(drv8323->status()->fsr2); break;
    case SPIRegister::drvConfigError:       output = static_cast<uint32_t>(drv8323->status()->fault_config); break;
    case SPIRegister::measuredPosition:
        res = TWO_PI * bldc->motor_position().position;
        std::memcpy(&output, &res, 4);
        break;
    case SPIRegister::measuredVelocity:
        res = TWO_PI * bldc->motor_position().velocity;
        std::memcpy(&output, &res, 4);
        break;
    case SPIRegister::measuredIQ:           std::memcpy(&output, &bldc->status().q_A, 4); break;
    case SPIRegister::measuredIPhaseA:      std::memcpy(&output, &bldc->status().cur1_A, 4); break;
    case SPIRegister::measuredIPhaseB:      std::memcpy(&output, &bldc->status().cur2_A, 4); break;
    case SPIRegister::measuredIPhaseC:      std::memcpy(&output, &bldc->status().cur3_A, 4); break;
    case SPIRegister::measuredUPhaseA:      std::memcpy(&output, &ZERO, 4); break;    // Not implemented for now
    case SPIRegister::measuredUPhaseB:      std::memcpy(&output, &ZERO, 4); break;   // Not implemented for now
    case SPIRegister::measuredUPhaseC:      std::memcpy(&output, &ZERO, 4); break;    // Not implemented for now
    case SPIRegister::measuredMotTemp:      std::memcpy(&output, bldc->config().enable_motor_temperature ? &bldc->status().motor_temp_C : &MINUS_ONE, 4); break;
    case SPIRegister::measuredDriveTemp:    std::memcpy(&output, &bldc->status().fet_temp_C, 4); break;
    case SPIRegister::measuredUBat:         std::memcpy(&output, &bldc->status().bus_V, 4); break;
    case SPIRegister::targetPosition:       std::memcpy(&output, &command->position, 4); break;
    case SPIRegister::targetVelocity:       std::memcpy(&output, &command->velocity, 4); break;
    case SPIRegister::targetIQ:             std::memcpy(&output, &command->i_q_A, 4); break;
    case SPIRegister::rawEncoderPos:        output = bldc->motor_position().sources[0].raw; break;
    case SPIRegister::encoderOrientation:   output = bldc->motor()->phase_invert; break;
    case SPIRegister::commutationOffset:    std::memcpy(&output, &bldc->motor()->offset[0], 4); break;
    case SPIRegister::nbrOfPoles:           output = bldc->motor()->poles; break;
    case SPIRegister::currentLoopKp:        std::memcpy(&output, &bldc->config().pid_dq.kp, 4); break;
    case SPIRegister::currentLoopKi:        std::memcpy(&output, &bldc->config().pid_dq.ki, 4); break;
    case SPIRegister::currentLoopIntMax:    output = 0; break;   // Not available, this loop has no anti-windup term.
    case SPIRegister::velocityLoopKp:       std::memcpy(&output, &bldc->config().pid_velocity.kp, 4); break;
    case SPIRegister::velocityLoopKi:       std::memcpy(&output, &bldc->config().pid_velocity.ki, 4); break;
    case SPIRegister::velocityLoopIntMax:   std::memcpy(&output, &bldc->config().pid_velocity.ilimit, 4); break;
    case SPIRegister::positionLoopKp:       std::memcpy(&output, &bldc->config().pid_position.kp, 4); break;
    case SPIRegister::positionLoopKd:       std::memcpy(&output, &bldc->config().pid_position.kd, 4); break;
    case SPIRegister::positionLoopKi:       std::memcpy(&output, &bldc->config().pid_position.ki, 4); break;
    case SPIRegister::positionLoopIntMax:   std::memcpy(&output, &bldc->config().pid_position.ilimit, 4); break;
    case SPIRegister::motorMaxCurrent:      std::memcpy(&output, &bldc->config().max_current_A, 4); break;
    case SPIRegister::motorMaxTemperature:  std::memcpy(&output, &bldc->config().enable_motor_temperature ? &bldc->config().motor_fault_temperature : &MINUS_ONE, 4); break;
    case SPIRegister::driverMaxTemperature: std::memcpy(&output, &bldc->config().fault_temperature, 4); break;
    case SPIRegister::commTimeout:          output = timeoutMs; break;
    case SPIRegister::driveSourceCurrent:   output = drv8323->getConfig()->idrivep_hs_ma; break;
    case SPIRegister::driveSinkCurrent:     output = drv8323->getConfig()->idriven_hs_ma; break;
    case SPIRegister::firmwareVersion:      output = FIRMWARE_VERSION; break;
    default:                                output = 0; break;
    }
}

// Handle write query.
void __attribute__ ((optimize("O3"))) nautilus::processWriteCommand(uint8_t const& registerAddress, uint32_t const& registerValue)
{
    // Basic register settings
    float fRegisterValue;
    std::memcpy(&fRegisterValue, &registerValue, 4);
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
            command->position = fRegisterValue / TWO_PI;
            command->velocity = 0.0;

            timeoutCounter = timeoutMs;
            bldc->Command(*command);
            break;
        case SPIRegister::targetVelocity:
            command->mode = moteus::kVelocity;
            command->velocity = fRegisterValue / TWO_PI;

            timeoutCounter = timeoutMs;
            bldc->Command(*command);
            break;
        case SPIRegister::targetIQ:
            command->mode = moteus::kCurrent;
            command->i_q_A = fRegisterValue;

            timeoutCounter = timeoutMs;
            bldc->Command(*command);
            break;

        case SPIRegister::nbrOfPoles:           bldc->motor()->poles = registerValue & 0xFF; break;
        case SPIRegister::driveSourceCurrent:
            drv8323->getConfig()->idrivep_hs_ma = static_cast<uint16_t>(registerValue);
            drv8323->getConfig()->idrivep_ls_ma = static_cast<uint16_t>(registerValue);
            break;
        case SPIRegister::driveSinkCurrent:
            drv8323->getConfig()->idriven_hs_ma = static_cast<uint16_t>(registerValue);
            drv8323->getConfig()->idriven_ls_ma = static_cast<uint16_t>(registerValue);
            break;
        default:
            break;
    }
}


// Avoid C++ name-mangling
extern "C" {

void __attribute__ ((optimize("O3"))) SPI3_IRQHandler(void)
{
    // Interrput last typically <500ns, except when processing read commands
    // where it lasts up to 1500ns.
    // Read data
    rxBuffer[posInMessage] = SPI3->DR;

    if (posInMessage == MESSAGE_LENGTH - 1)
    {
        // Check CRC
        uint8_t const crc = ~(rxBuffer[0] + rxBuffer[1] + rxBuffer[2] + rxBuffer[3] +\
                              rxBuffer[4] + rxBuffer[5] + rxBuffer[6]);
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
                uint32_t const theta = ((rxBuffer[1] << 24) + (rxBuffer[2] << 16) + (rxBuffer[3] << 8) + (rxBuffer[4]));
                std::memcpy(&command->theta, &theta, 4);
                command->theta_rate = 0;
                uint16_t const voltageRatio = (rxBuffer[5] << 8) + rxBuffer[6];
                command->voltage = (static_cast<float_t>(voltageRatio) / 65535.0f) * bldc->status().bus_V;

                timeoutCounter = timeoutMs;
                bldc->Command(*command);
            }
            else if (rxBuffer[0] == static_cast<uint8_t>(SPICommand::storeToPersistentMemory))
            {
                // Note: for some reason this only works once, then the uC must be rebooted.
                // But this is good enough: writing the config is typically only needed for commutation,
                // so doing it once works well enough.
                mjlib::micro::ErrorCallback cb = [](mjlib::micro::error_code) {};
                mjlib::micro::AsyncWrite(*config_stream, "conf write\n", cb);
                config_queue->Poll();
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

    if (posInMessage == 2)
    {
        // We have received enough info from the user to know what to do.
        uint32_t data = 0;
        if (rxBuffer[0] == static_cast<uint8_t>(SPICommand::regRead))
        {
            // Handle read request
            nautilus::processReadCommand(rxBuffer[1], data);
        }

        // Send next byte.
        txBuffer[3] = (data >> 24) & 0xFF;
        txBuffer[4] = (data >> 16) & 0xFF;
        txBuffer[5] = (data >> 8) & 0xFF;
        txBuffer[6] = (data) & 0xFF;
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[3];
    }
    else if (posInMessage == 3)
    {
        // Send the rest of the data content.
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[4];
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[5];
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[6];
    }
    else if (posInMessage == 4)
    {
        // Send last element: CRC
        uint8_t const crc = ~(txBuffer[0] + txBuffer[1] + txBuffer[2] + txBuffer[3] +\
                              txBuffer[4] + txBuffer[5] + txBuffer[6]);
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) crc;
    }
}


void __attribute__ ((optimize("O3"))) EXTI15_10_IRQHandler(void)
{
    // Interrupt duration: 150ns (spi end), 400-600ns (SPI start)
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
        txBuffer[0] = (encVal >> 8) & 0xFF;
        txBuffer[1] = encVal & 0xFF;
        // Third byte: status
        txBuffer[2] = bldc->status().mode;

        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[0];
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[1];
        * ((__IO uint8_t *) & SPI3->DR) = (uint8_t) txBuffer[2];
    }
}

}


void NautilusSPIInterface::setup()
{
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
