#include "NautilusDriver.h"

#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>


#include <iostream>

using namespace nautilus;

int spiOpen(std::string const& portName, int const& frequency)
{
    int fd = open(portName.c_str(), O_RDWR) ;
    if(fd < 0)
    {
        #ifdef DEBUG
            std::cout << "SPI error opening bus " << fd << ": " << std::strerror(errno) << std::endl;
        #endif
        return -1;
    }

    // Setup the SPI bus, mode 3 (SPI_CPOL | SPI_CPHA).
    // See documentation of spi/spidev for details.
    int mode = SPI_MODE_2;
    if(ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0)
    {
        #ifdef DEBUG
            std::cout << "SPI error configuring fd " << fd << ": " << std::strerror(errno) << std::endl;
        #endif
        return -1;
    }

    int nBits = 8;
    if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &nBits) < 0)
    {
        #ifdef DEBUG
            std::cout << "SPI error configuring fd " << fd << ": " << std::strerror(errno) << std::endl;
        #endif
        return -1;
    }

    if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &frequency)   < 0)
    {
        #ifdef DEBUG
            std::cout << "SPI error configuring fd " << fd << ": " << std::strerror(errno) << std::endl;
        #endif
        return -1;
    }
    return fd;
}


void spiClose(int const& fd)
{
    if(fd > 0)
    {
        // Release CS - for some reason this is needed for the raspberry pi, as CS is not automatically released.
        int mode = SPI_MODE_0;
        if(ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0)
        {
            #ifdef DEBUG
            std::cout << "SPI error closing fd " << fd << ": " << std::strerror(errno) << std::endl;
            #endif
        }
        close(fd);
    }
}


Nautilus::Nautilus(std::string const& portName, int const& frequency):
    portName_(portName),
    frequency_(frequency)
{
}


NautilusReply Nautilus::readRegister(Register const& reg)
{
    return spiComm(Command::regRead, static_cast<uint8_t>(reg), 0);
}


bool Nautilus::writeRegister(Register const& reg, float const& value)
{
    spiComm(Command::regWrite, static_cast<uint8_t>(reg), reinterpret_cast<uint32_t const &>(value));
    return true;
}

bool Nautilus::writeRegister(Register const& reg, uint32_t const& value)
{
    spiComm(Command::regWrite, static_cast<uint8_t>(reg), value);
    return true;
}

NautilusReply Nautilus::stop()
{
    return spiComm(Command::stop, 0, 0);
}

NautilusReply Nautilus::commutation(float const& theta, float const& voltageRatio)
{
    uint8_t buffer[8];
    buffer[0] = Command::commutation;
    uint32_t const th = reinterpret_cast<uint32_t const &>(theta);
    buffer[1] = (th >> 24) & 0xFF;
    buffer[2] = (th >> 16) & 0xFF;
    buffer[3] = (th >> 8) & 0xFF;
    buffer[4] = th & 0xFF;
    uint16_t const ratio = static_cast<uint16_t>(std::min(1.0f, std::max(0.0f, voltageRatio)) * 65635);
    buffer[5] = (ratio >> 8) & 0xFF;
    buffer[6] = ratio & 0xFF;
    return spiComm(buffer);
}


NautilusReply Nautilus::spiComm(uint8_t command, uint8_t address, uint32_t data)
{
    uint8_t buffer[8];
    buffer[0] = command;
    buffer[1] = address;
    buffer[2] = (data >> 24) & 0xFF;
    buffer[3] = (data >> 16) & 0xFF;
    buffer[4] = (data >> 8) & 0xFF;
    buffer[5] = data & 0xFF;
    buffer[6] = 0;
    return spiComm(buffer);
}


NautilusReply Nautilus::spiComm(uint8_t *buffer)
{
    // Compute checksum
    buffer[7] = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6];

    static struct spi_ioc_transfer spiCtrl;
    spiCtrl.tx_buf = (unsigned long)&buffer[0];
    spiCtrl.rx_buf = (unsigned long)&buffer[0];
    spiCtrl.len = 8;
    spiCtrl.speed_hz = frequency_;
    spiCtrl.bits_per_word = 8;
    spiCtrl.delay_usecs = 1;
    spiCtrl.cs_change = true;

    int fd = spiOpen(portName_, frequency_);
    int res = ioctl(fd, SPI_IOC_MESSAGE(1), &spiCtrl);
    spiClose(fd);

    NautilusReply reply;
    reply.encoderPosition = (buffer[0] << 8) + buffer[1];
    reply.validEncoder = reply.encoderPosition & (1 << 15);
    reply.encoderPosition &= ~(1 << 15);
    reply.mode = buffer[2];
    uint32_t d = (uint32_t)(buffer[3]) << 24;
    d += (uint32_t)(buffer[4]) << 16;
    d += (uint32_t)(buffer[5]) << 8;
    d += (uint32_t)(buffer[6]);
    reply.data = reinterpret_cast<float &>(d);

    uint8_t checksum = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6];

    reply.isValid = res == 8 && buffer[7] == checksum;
    if (reply.isValid)
        nSuccess++;
    else
        nFailed++;

    return reply;
}
