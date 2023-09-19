#include "Commutation.h"
#include <iostream>
#include <unistd.h>

void performCommutation(nautilus::Nautilus *nautilus, double const& targetCurrent, bool* done)
{

    std::cout << "Performing commutation, target current: " << targetCurrent << std::endl;
    nautilus->writeRegister(nautilus::encoderOrientation, static_cast<uint32_t>(0));
    nautilus->writeRegister(nautilus::commutationOffset, static_cast<uint32_t>(0));
    nautilus->commutation(0.0, 0.01);
    usleep(2000000);
    nautilus->stop();
    *done = true;
}
