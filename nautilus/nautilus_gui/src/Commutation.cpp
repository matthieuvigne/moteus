#include "Commutation.h"
#include <iostream>
#include <iomanip>

#include <unistd.h>
#include <math.h>

typedef std::vector<nautilus::Register> rVector;


std::vector<float> getAverageRegisters(nautilus::Nautilus *nautilus,
                                       rVector const& registers,
                                       unsigned int nSamples,
                                       unsigned int samplesIntervalUs,
                                       std::vector<bool> isInteger)
{
    std::vector<float> results(registers.size(), 0.0);
    std::vector<int> successfulSamples(registers.size(), 0);
    for (unsigned int i = 0; i < nSamples; i++)
    {
        for (unsigned int j = 0; j < registers.size(); j++)
        {
            nautilus::NautilusReply reply = nautilus->readRegister(registers.at(j));
            if (reply.isValid)
            {
                results.at(j) += (isInteger.at(j) ? reinterpret_cast<uint32_t &>(reply.data) : reply.data);
                successfulSamples.at(j)++;
            }
        }
        usleep(samplesIntervalUs);
    }
    for (unsigned int j = 0; j < results.size(); j++)
    {
        if (successfulSamples.at(j) > 0)
            results.at(j) /= successfulSamples.at(j);
    }
    return results;
}

std::vector<float> getAverageRegisters(nautilus::Nautilus *nautilus,
                                       rVector const& registers,
                                       unsigned int nSamples = 10,
                                       unsigned int samplesIntervalUs = 1000,
                                       bool isInteger = false)
{
    return getAverageRegisters(nautilus, registers, nSamples, samplesIntervalUs, std::vector<bool>(registers.size(), false));
}


void performCommutation(nautilus::Nautilus *nautilus, double const& targetCurrent, bool* done)
{

    std::cout << "Performing commutation, target current: " << targetCurrent << std::endl;
    nautilus->writeRegister(nautilus::Register::encoderOrientation, static_cast<uint32_t>(0));
    nautilus->writeRegister(nautilus::Register::commutationOffset, static_cast<uint32_t>(0));

    // Identify resistance to find the correct current setting.
    float voltageRatio = 0.05;
    nautilus->commutation(0.0, voltageRatio);
    usleep(20000);
    std::vector<float> results = getAverageRegisters(nautilus, rVector({nautilus::Register::measuredUBat, nautilus::Register::measuredIPhaseA}));

    float res = voltageRatio * results.at(0) / std::abs(results.at(1));

    // Now that we have the resistance, let's check it with the desired user current.
    voltageRatio = res * targetCurrent / results.at(0);

    // Check that we get the desired current.
    nautilus->commutation(0.0, voltageRatio);
    usleep(20000);
    results = getAverageRegisters(nautilus, rVector({nautilus::Register::measuredIPhaseA}));

    if (std::abs(results[0] - targetCurrent) / std::abs(targetCurrent) > 0.20)
    {
        std::cout << "Error determining motor resistance." << std::endl;
        nautilus->stop();
        *done = true;
        return;
    }
    else
    {
        std::cout << "Identified motor resistance:" << std::setprecision(3) << res << std::endl;
    }

    // Move to zero position
    nautilus->commutation(0.0, voltageRatio);
    usleep(500000);

    results = getAverageRegisters(nautilus, rVector({nautilus::Register::rawEncoderPos, nautilus::Register::measuredPosition}), 20, 500, std::vector<bool>({true, false}));

    uint32_t const encoderZero = static_cast<uint32_t>(results.at(0));
    float zeroPos = results.at(1);
    std::cout << "Encoder zero position:" << zeroPos << "(" << encoderZero << ")" << std::endl;

    // Rotate by 90 electrical degree and check that the result makes sense.
    nautilus->commutation(M_PI_2, voltageRatio);
    usleep(500000);
    results = getAverageRegisters(nautilus, rVector({nautilus::Register::measuredPosition}), 20, 500);
    float orthogonalPos = results.at(0);
    std::cout << "Encoder 90deg position:" << orthogonalPos << std::endl;

    // Compute encoder direction and check number of poles.
    float encoderDelta = orthogonalPos - zeroPos;
    if (std::abs(encoderDelta) > M_PI)
    {
        if (orthogonalPos > zeroPos)
            zeroPos += 2 * M_PI;
        else
            orthogonalPos += 2 * M_PI;
        encoderDelta = orthogonalPos - encoderZero;
    }
    uint32_t isInverted =  encoderDelta > 0 ? 0 : 1;
    std::cout << "Encoder inverted: " << isInverted << std::endl;

    // Estimate number of poles
    std::cout << "Number of poles: " << 2 * M_PI / std::abs(encoderDelta * 4) << std::endl;

    // Compare with internal register setting
    // TODO

    // Save results
    nautilus->writeRegister(nautilus::Register::encoderOrientation, isInverted);
    nautilus->writeRegister(nautilus::Register::commutationOffset, encoderZero);


    // for (int i = 0; i < 10; i++)
    // {
    //     nautilus->commutation(0.0, voltageRatio);
    //     usleep(300000);
    //     nautilus->commutation(2.0, voltageRatio);
    //     usleep(300000);
    // }

    nautilus->stop();
    *done = true;
}
