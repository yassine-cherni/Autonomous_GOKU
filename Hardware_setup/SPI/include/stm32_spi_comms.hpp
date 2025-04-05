#ifndef DIFFDRIVE_STM32F4_STM32SPI_COMMS_HPP_
#define DIFFDRIVE_STM32F4_STM32SPI_COMMS_HPP_

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <iostream>

class STM32SPIComms
{
public:
    STM32SPIComms() : spi_fd_(-1), speed_hz_(1000000), mode_(0) {}

    void connect(const std::string &spi_device, int speed_hz, int mode)
    {
        speed_hz_ = speed_hz;
        mode_ = mode;
        spi_fd_ = open(spi_device.c_str(), O_RDWR);
        if (spi_fd_ < 0)
        {
            throw std::runtime_error("Failed to open SPI device: " + spi_device);
        }
        uint8_t mode_val = mode_;
        if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode_val) < 0)
        {
            throw std::runtime_error("Failed to set SPI mode");
        }
        uint32_t speed = speed_hz_;
        if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
        {
            throw std::runtime_error("Failed to set SPI speed");
        }
    }

    void disconnect()
    {
        if (spi_fd_ != -1)
        {
            close(spi_fd_);
            spi_fd_ = -1;
        }
    }

    bool connected() const
    {
        return spi_fd_ != -1;
    }

    void read_encoder_values(int &val_1, int &val_2)
    {
        uint8_t tx_cmd[1] = {'e'};
        uint8_t rx_cmd[1];
        struct spi_ioc_transfer tr_cmd = {
            .tx_buf = (unsigned long)tx_cmd,
            .rx_buf = (unsigned long)rx_cmd,
            .len = 1,
            .speed_hz = static_cast<uint32_t>(speed_hz_),
            .bits_per_word = 8,
        };
        if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr_cmd) < 0)
        {
            std::cerr << "Failed to send encoder command" << std::endl;
            return;
        }

        uint8_t tx_data[8] = {0}; // Dummy bytes to clock out response
        uint8_t rx_data[8];
        struct spi_ioc_transfer tr_data = {
            .tx_buf = (unsigned long)tx_data,
            .rx_buf = (unsigned long)rx_data,
            .len = 8,
            .speed_hz = static_cast<uint32_t>(speed_hz_),
            .bits_per_word = 8,
        };
        if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr_data) < 0)
        {
            std::cerr << "Failed to read encoder values" << std::endl;
            return;
        }

        val_1 = *(int32_t*)&rx_data[0];
        val_2 = *(int32_t*)&rx_data[4];
    }

    void set_motor_values(int val_1, int val_2)
    {
        uint8_t tx_data[9];
        tx_data[0] = 'm';
        std::memcpy(&tx_data[1], &val_1, 4);
        std::memcpy(&tx_data[5], &val_2, 4);
        uint8_t rx_data[9];
        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)tx_data,
            .rx_buf = (unsigned long)rx_data,
            .len = 9,
            .speed_hz = static_cast<uint32_t>(speed_hz_),
            .bits_per_word = 8,
        };
        if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) < 0)
        {
            std::cerr << "Failed to set motor values" << std::endl;
        }
    }

    void set_pid_values(int k_p, int k_d, int k_i, int k_o)
    {
        uint8_t tx_data[17];
        tx_data[0] = 'u';
        std::memcpy(&tx_data[1], &k_p, 4);
        std::memcpy(&tx_data[5], &k_d, 4);
        std::memcpy(&tx_data[9], &k_i, 4);
        std::memcpy(&tx_data[13], &k_o, 4);
        uint8_t rx_data[17];
        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)tx_data,
            .rx_buf = (unsigned long)rx_data,
            .len = 17,
            .speed_hz = static_cast<uint32_t>(speed_hz_),
            .bits_per_word = 8,
        };
        if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) < 0)
        {
            std::cerr << "Failed to set PID values" << std::endl;
        }
    }

private:
    int spi_fd_;
    int speed_hz_;
    int mode_;
};

#endif // DIFFDRIVE_STM32F4_STM32SPI_COMMS_HPP_
