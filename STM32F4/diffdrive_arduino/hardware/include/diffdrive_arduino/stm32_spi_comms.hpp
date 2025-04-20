#ifndef DIFFDRIVE_ARDUINO_SPI_COMMS_HPP
#define DIFFDRIVE_ARDUINO_SPI_COMMS_HPP

#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/spi/spidev.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

class SPIComms {
public:
  SPIComms() = default;

  void connect(const std::string& device, uint32_t speed_hz) {
    spi_fd_ = open(device.c_str(), O_RDWR);
    if (spi_fd_ < 0) {
      throw std::runtime_error("Failed to open SPI device " + device);
    }

    // Set SPI mode
    uint8_t mode = SPI_MODE_0;
    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0) {
      throw std::runtime_error("Failed to set SPI mode");
    }

    // Set bits per word
    uint8_t bits_per_word = 8;
    if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
      throw std::runtime_error("Failed to set bits per word");
    }

    // Set max speed
    speed_ = speed_hz;
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) < 0) {
      throw std::runtime_error("Failed to set max SPI speed");
    }

    std::cout << "SPI connection established at " << speed_ << " Hz" << std::endl;
  }

  void disconnect() {
    if (spi_fd_ >= 0) {
      close(spi_fd_);
      spi_fd_ = -1;
    }
  }

  bool connected() const {
    return spi_fd_ >= 0;
  }

  // Send a single character command, return true on success
  bool send_command(char cmd) {
    uint8_t tx_buf[1] = { static_cast<uint8_t>(cmd) };
    uint8_t rx_buf[1] = { 0 };

    struct spi_ioc_transfer tr;
    std::memset(&tr, 0, sizeof(tr));
    tr.tx_buf = reinterpret_cast<unsigned long>(tx_buf);
    tr.rx_buf = reinterpret_cast<unsigned long>(rx_buf);
    tr.len = 1;
    tr.speed_hz = speed_;
    tr.bits_per_word = 8;
    tr.delay_usecs = 0;

    int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
      std::cerr << "SPI transfer failed" << std::endl;
      return false;
    }

    // Optionally print received byte:
    std::cout << "Sent '" << cmd << "', received: " << static_cast<int>(rx_buf[0]) << std::endl;
    return true;
  }

  // Motor ON/OFF interface (L298N without PWM)
  void set_motor_state(bool left_on, bool right_on) {
    // You can define your own command protocol here.
    // For example:
    // 'A' = left on, 'a' = left off
    // 'B' = right on, 'b' = right off
    send_command(left_on ? 'A' : 'a');
    send_command(right_on ? 'B' : 'b');
  }

  // Read encoder values from STM32 (for example, encoder data in 2 bytes)
  void read_encoder_values(int &left_encoder, int &right_encoder) {
    uint8_t cmd_left = 0x01; // Command to read left encoder (example)
    uint8_t cmd_right = 0x02; // Command to read right encoder (example)

    uint8_t rx_buf_left[2] = {0};  // 2 bytes for encoder value
    uint8_t rx_buf_right[2] = {0}; // 2 bytes for encoder value

    struct spi_ioc_transfer tr_left;
    std::memset(&tr_left, 0, sizeof(tr_left));
    tr_left.tx_buf = reinterpret_cast<unsigned long>(&cmd_left);
    tr_left.rx_buf = reinterpret_cast<unsigned long>(&rx_buf_left);
    tr_left.len = 2; // Receive 2 bytes for left encoder value
    tr_left.speed_hz = speed_;
    tr_left.bits_per_word = 8;
    tr_left.delay_usecs = 0;

    struct spi_ioc_transfer tr_right;
    std::memset(&tr_right, 0, sizeof(tr_right));
    tr_right.tx_buf = reinterpret_cast<unsigned long>(&cmd_right);
    tr_right.rx_buf = reinterpret_cast<unsigned long>(&rx_buf_right);
    tr_right.len = 2; // Receive 2 bytes for right encoder value
    tr_right.speed_hz = speed_;
    tr_right.bits_per_word = 8;
    tr_right.delay_usecs = 0;

    // Send and receive data
    int ret_left = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr_left);
    int ret_right = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr_right);

    if (ret_left < 0 || ret_right < 0) {
      std::cerr << "SPI transfer failed" << std::endl;
      left_encoder = 0;
      right_encoder = 0;
      return;
    }

    // Combine the two bytes to form the encoder values
    left_encoder = (rx_buf_left[0] << 8) | rx_buf_left[1]; // Assuming big-endian
    right_encoder = (rx_buf_right[0] << 8) | rx_buf_right[1]; // Assuming big-endian

    std::cout << "Left encoder: " << left_encoder << ", Right encoder: " << right_encoder << std::endl;
  }

  // Set motor speed/position (example: send motor values for left and right motors)
  void set_motor_values(int left_motor_value, int right_motor_value) {
    uint8_t cmd_left[3] = {0x10, static_cast<uint8_t>(left_motor_value >> 8), static_cast<uint8_t>(left_motor_value & 0xFF)};
    uint8_t cmd_right[3] = {0x11, static_cast<uint8_t>(right_motor_value >> 8), static_cast<uint8_t>(right_motor_value & 0xFF)};

    // Send left motor command
    struct spi_ioc_transfer tr_left;
    std::memset(&tr_left, 0, sizeof(tr_left));
    tr_left.tx_buf = reinterpret_cast<unsigned long>(cmd_left);
    tr_left.rx_buf = reinterpret_cast<unsigned long>(&cmd_left);
    tr_left.len = 3;  // Send 3 bytes (command + 2 bytes for motor value)
    tr_left.speed_hz = speed_;
    tr_left.bits_per_word = 8;
    tr_left.delay_usecs = 0;

    // Send right motor command
    struct spi_ioc_transfer tr_right;
    std::memset(&tr_right, 0, sizeof(tr_right));
    tr_right.tx_buf = reinterpret_cast<unsigned long>(cmd_right);
    tr_right.rx_buf = reinterpret_cast<unsigned long>(&cmd_right);
    tr_right.len = 3;  // Send 3 bytes (command + 2 bytes for motor value)
    tr_right.speed_hz = speed_;
    tr_right.bits_per_word = 8;
    tr_right.delay_usecs = 0;

    // Perform SPI communication
    int ret_left = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr_left);
    int ret_right = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr_right);

    if (ret_left < 0 || ret_right < 0) {
      std::cerr << "SPI transfer failed" << std::endl;
      return;
    }

    std::cout << "Sent motor values: Left = " << left_motor_value << ", Right = " << right_motor_value << std::endl;
  }

private:
  int spi_fd_ = -1;
  uint32_t speed_ = 500000; // default 500 kHz
};

#endif  // DIFFDRIVE_ARDUINO_SPI_COMMS_HPP
