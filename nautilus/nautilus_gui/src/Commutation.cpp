#include "Commutation.h"
#include <iostream>
#include <iomanip>

#include <unistd.h>
#include <math.h>

#include <sstream>

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


void performCommutation(nautilus::Nautilus *nautilus, ThreadStatus *status)
{
    std::stringstream sstream;
    sstream << "Performing commutation, target current: " << std::setprecision(2) << status->commutationCurrent;
    status->appendMessage(sstream.str());

    nautilus->writeRegister(nautilus::Register::encoderOrientation, static_cast<uint32_t>(0));
    nautilus->writeRegister(nautilus::Register::commutationOffset, static_cast<uint32_t>(0));

    // Identify resistance to find the correct current setting.
    float voltageRatio = 0.05;
    nautilus->commutation(0.0, voltageRatio);
    usleep(20000);
    std::vector<float> results = getAverageRegisters(nautilus, rVector({nautilus::Register::measuredUBat, nautilus::Register::measuredIPhaseA}));

    float res = voltageRatio * results.at(0) / std::abs(results.at(1));

    // Now that we have the resistance, let's check it with the desired user current.
    voltageRatio = res * status->commutationCurrent / results.at(0);

    // Check that we get the desired current.
    nautilus->commutation(0.0, voltageRatio);
    usleep(20000);
    results = getAverageRegisters(nautilus, rVector({nautilus::Register::measuredIPhaseA}));

    if (std::abs(results[0] - status->commutationCurrent) / std::abs(status->commutationCurrent) > 0.20)
    {
        status->appendMessage("Error determining motor resistance.");
        nautilus->stop();
        return;
    }
    else
    {
        sstream.str("");
        sstream << "Identified motor resistance:" << std::setprecision(3) << res;
        status->appendMessage(sstream.str());
    }

    // Perform full initial cycle.

    for (int i = 0; i < 50; i++)
    {
        nautilus->commutation(2 * M_PI * i / 50.0, voltageRatio);
        usleep(20000);
    }

    // Go back to zero.
    nautilus->commutation(0.0, voltageRatio);
    usleep(20000);

    results = getAverageRegisters(nautilus, rVector({nautilus::Register::rawEncoderPos, nautilus::Register::measuredPosition}), 20, 500, std::vector<bool>({true, false}));

    uint32_t const encoderZero = static_cast<uint32_t>(results.at(0));
    float zeroPos = results.at(1);
    sstream.str("");
    sstream << "Encoder zero position:" << zeroPos << "(" << encoderZero << ")";
    status->appendMessage(sstream.str());

    // Rotate by 90 electrical degree and check that the result makes sense.
    for (int i = 0; i < 10; i++)
    {
        nautilus->commutation(M_PI_2 * i / 10.0, voltageRatio);
        usleep(20000);
    }

    // for (int j = 0; j < 14; j++)
    // {
    //     for (int i = 0; i < 50; i++)
    //     {
    //         nautilus->commutation(2 * M_PI * i / 50.0, voltageRatio);
    //         usleep(20000);
    //     }
    //     usleep(500000);
    // }


    nautilus->commutation(M_PI_2, voltageRatio);
    usleep(20000);
    results = getAverageRegisters(nautilus, rVector({nautilus::Register::measuredPosition}), 20, 500);
    float orthogonalPos = results.at(0);
    sstream.str("");
    sstream << "Encoder 90deg position:" << orthogonalPos;
    status->appendMessage(sstream.str());

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
    sstream.str("");
    sstream << "Encoder inverted: " << isInverted;
    status->appendMessage(sstream.str());

    // Estimate number of poles
    sstream.str("");
    sstream << "Number of poles: " << 2 * M_PI / std::abs(encoderDelta * 4);
    status->appendMessage(sstream.str());

    // Compare with internal register setting
    uint32_t nPoles = static_cast<uint32_t>(2 * 2 * M_PI / std::abs(encoderDelta * 4));
    nautilus::NautilusReply rep;
    while (!rep.isValid)
        rep = nautilus->readRegister(nautilus::Register::nbrOfPoles);
    uint32_t expectedNPoles = reinterpret_cast<uint32_t &>(rep.data);
    if (nPoles != expectedNPoles)
    {
        sstream.str("");
        sstream << "Error: motor configured for " << expectedNPoles << " poles but measured " << nPoles ;
        status->appendMessage(sstream.str());

        nautilus->stop();
        // return;
    }

    // Convert zero to electrical angle.
    float offset = zeroPos * (nPoles / 2);
    while (offset > 2 * M_PI)
        offset -= 2 * M_PI;

    // Save results
    nautilus->writeRegister(nautilus::Register::encoderOrientation, isInverted);
    nautilus->writeRegister(nautilus::Register::commutationOffset, -offset);

    sstream.str("");
    sstream << "Computed commutation offset: " << -offset << nPoles ;
    status->appendMessage(sstream.str());
    status->appendMessage("Commutation completed");
    nautilus->stop();
}
