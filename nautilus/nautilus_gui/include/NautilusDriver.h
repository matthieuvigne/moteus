/// \file NautilusDriver.h
/// \brief Linux driver for Nautilus (to be used for instance on Raspberry Pi)

#ifndef NAUTILUS_DRIVER_H
    #define NAUTILUS_DRIVER_H
    #include <unistd.h>
    #include <linux/spi/spidev.h>
    #include <string>

namespace nautilus
{

    struct NautilusRegister{
        uint8_t address;
        std::string name;
        bool isFloat;
        bool isWritable;

        NautilusRegister(uint8_t const& ad, std::string const& n, bool const& isF, bool const& isW):
            address(ad),
            name(n),
            isFloat(isF),
            isWritable(isW)
        {}
    };

    static NautilusRegister currentMode             = NautilusRegister(0x00, "Current mode",        false, false);

    static NautilusRegister faultCode               = NautilusRegister(0x01, "Fault code",          false, false);       // uint32_t
    static NautilusRegister measuredPosition        = NautilusRegister(0x10, "Measured position",   true, false);       // float32_t
    static NautilusRegister measuredVelocity        = NautilusRegister(0x11, "Measured velocity",   true, false);       // float32_t
    static NautilusRegister measuredIQ              = NautilusRegister(0x12, "Measured Iq",         true, false);       // float32_t
    static NautilusRegister measuredIPhaseA         = NautilusRegister(0x13, "Measured I phase A",  true, false);       // float32_t
    static NautilusRegister measuredIPhaseB         = NautilusRegister(0x14, "Measured I phase B",  true, false);       // float32_t
    static NautilusRegister measuredIPhaseC         = NautilusRegister(0x15, "Measured I phase c",  true, false);       // float32_t
    static NautilusRegister measuredUPhaseA         = NautilusRegister(0x16, "Measured U phase A",  true, false);       // float32_t
    static NautilusRegister measuredUPhaseB         = NautilusRegister(0x17, "Measured U phase B",  true, false);       // float32_t
    static NautilusRegister measuredUPhaseC         = NautilusRegister(0x18, "Measured U phase C",  true, false);       // float32_t
    static NautilusRegister measuredMotTemp         = NautilusRegister(0x19, "Measured mot temp",   true, false);       // float32_t
    static NautilusRegister measuredDriveTemp       = NautilusRegister(0x1A, "Measured fet temp",   true, false);       // float32_t
    static NautilusRegister measuredUBat            = NautilusRegister(0x1B, "Measured battery voltage",   true, false);       // float32_t

    static NautilusRegister targetPosition          = NautilusRegister(0x20, "Target position", true, true);       // float32_t
    static NautilusRegister targetVelocity          = NautilusRegister(0x21, "Target velocity", true, true);       // float32_t
    static NautilusRegister targetIQ                = NautilusRegister(0x22, "Target Iq",       true, true);       // float32_t

    static NautilusRegister rawEncoderPos           = NautilusRegister(0x30, "Raw position",        false, false);       // uint16_t
    static NautilusRegister encoderOrientation      = NautilusRegister(0x31, "Encoder orientation", false, true);       // uint8_t
    static NautilusRegister commutationOffset       = NautilusRegister(0x32, "Commutation offset",  false, true);       // uint16_t
    static NautilusRegister nbrOfPoles              = NautilusRegister(0x33, "Number of poles",     false, false);       // uint16_t

    static NautilusRegister currentLoopKp           = NautilusRegister(0x40, "Current loop Kp", true, true);       // float32_t
    static NautilusRegister currentLoopKI           = NautilusRegister(0x41, "Current loop Ki", true, true);       // float32_t
    static NautilusRegister currentLoopIntMax       = NautilusRegister(0x42, "Current loop max integral", true, true);       // float32_t
    static NautilusRegister velocityLoopKp          = NautilusRegister(0x43, "Velocity loop Kp", true, true);       // float32_t
    static NautilusRegister velocityLoopKI          = NautilusRegister(0x44, "Velocity loop Ki", true, true);       // float32_t
    static NautilusRegister velocityLoopIntMax      = NautilusRegister(0x45, "Velocity loop max integral", true, true);       // float32_t
    static NautilusRegister motorMaxCurrent         = NautilusRegister(0x50, "Motor max current", true, true);       // float32_t
    static NautilusRegister motorMaxTemperature     = NautilusRegister(0x51, "Motor max temperature", true, true);       // float32_t
    static NautilusRegister driverMaxTemperature    = NautilusRegister(0x52, "Driver max temperature", true, true);       // float32_t
    static NautilusRegister commTimeout             = NautilusRegister(0x53, "Communication timeout", true, true);       // float32_t

    static NautilusRegister allRegisters[] = {
        currentMode,
        faultCode,
        measuredPosition,
        measuredVelocity,
        measuredIQ,
        measuredIPhaseA,
        measuredIPhaseB,
        measuredIPhaseC,
        measuredUPhaseA,
        measuredUPhaseB,
        measuredUPhaseC,
        measuredMotTemp,
        measuredDriveTemp,
        measuredUBat,
        targetPosition,
        targetVelocity,
        targetIQ,
        rawEncoderPos,
        encoderOrientation,
        commutationOffset,
        nbrOfPoles,
        currentLoopKp,
        currentLoopKI,
        currentLoopIntMax,
        velocityLoopKp,
        velocityLoopKI,
        velocityLoopIntMax,
        motorMaxCurrent,
        motorMaxTemperature,
        driverMaxTemperature,
        commTimeout};

    struct NautilusReply{
        uint16_t encoderPosition = 0;
        bool validEncoder = false;
        uint8_t mode = 0;
        float data;
        bool isValid = true;
    };


    enum Command {
        regWrite = 0x01,
        regRead  = 0x02,
        commutation  = 0x03,
        stop  = 0x05,
    };

    class Nautilus{
        public:
            /// \brief Constructor.
            Nautilus(std::string const& portName, int const& frequency = 1000000);

            /// \brief Read a register
            NautilusReply readRegister(NautilusRegister const& reg);

            bool writeRegister(NautilusRegister const& reg, float const& value);
            bool writeRegister(NautilusRegister const& reg, uint32_t const& value);

            /// @brief Stop the motors
            NautilusReply stop();

            /// @brief Commutation command
            /// @param theta Angle, range 0-2 pi
            /// @param voltageRatio Ratio of max voltage, range 0-1
            NautilusReply commutation(float const& theta, float const& voltageRatio);

            int nSuccess = 0;
            int nFailed = 0;
        private:
            /// @brief Perform SPI communication
            /// @param buffer Data buffer, will be overwritten. Must contain 8 elements
            /// @return Formatted reply from drive.
            NautilusReply spiComm(uint8_t *buffer);

            /// @brief Helper for SPI communication, constructing the message.
            /// @param command
            /// @param address
            /// @param data
            /// @return
            NautilusReply spiComm(uint8_t command, uint8_t address, uint32_t data);

            std::string portName_;  ///< Name of the SPI port
            int frequency_; ///< Frequency, in Hz.
    };
}
#endif

