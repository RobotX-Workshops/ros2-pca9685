#include "pca9685/pca9685_driver.hpp"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cmath>
#include <stdexcept>
#include <cstdint>

// PCA9685 Registers
constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t PRE_SCALE = 0xFE;
constexpr uint8_t LED0_ON_L = 0x06;

namespace pca9685
{

  PCA9685Driver::PCA9685Driver(int bus_number, int device_address)
  {
    std::string i2c_bus = "/dev/i2c-" + std::to_string(bus_number);
    i2c_fd_ = open(i2c_bus.c_str(), O_RDWR);
    if (i2c_fd_ < 0)
    {
      throw std::runtime_error("Failed to open I2C bus: " + i2c_bus);
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, device_address) < 0)
    {
      throw std::runtime_error("Failed to acquire bus access and/or talk to slave.");
    }

    // Reset and wake up the device
    write_byte(MODE1, 0x00); // Wake up
    usleep(5000);            // Wait for oscillator
  }

  PCA9685Driver::~PCA9685Driver()
  {
    if (i2c_fd_ >= 0)
    {
      close(i2c_fd_);
    }
  }

  void PCA9685Driver::set_pwm_frequency(int frequency)
  {
    frequency_ = frequency;
    pwm_scale_ = static_cast<double>(frequency) / 60.0; // Assuming 60Hz is default

    uint8_t old_mode = read_byte(MODE1);
    uint8_t new_mode = (old_mode & 0x7F) | 0x10; // Sleep
    write_byte(MODE1, new_mode);

    // Calculate prescale value
    int prescale = static_cast<int>(std::round(25000000.0 / (4096.0 * frequency)) - 1);
    write_byte(PRE_SCALE, static_cast<uint8_t>(prescale));

    write_byte(MODE1, old_mode);
    usleep(5000);
    write_byte(MODE1, old_mode | 0xA0); // Restart
  }

  void PCA9685Driver::set_pulse(int channel, int pulse)
  {
    if (channel < 0 || channel > 15)
    {
      throw std::out_of_range("Channel must be between 0 and 15.");
    }
    int scaled_pulse = static_cast<int>(pulse * pwm_scale_);
    if (scaled_pulse > 4095)
      scaled_pulse = 4095;
    if (scaled_pulse < 0)
      scaled_pulse = 0;

    // Set PWM, ON=0, OFF=pulse
    write_byte(LED0_ON_L + 4 * channel, 0 & 0xFF);
    write_byte(LED0_ON_L + 4 * channel + 1, 0 >> 8);
    write_byte(LED0_ON_L + 4 * channel + 2, scaled_pulse & 0xFF);
    write_byte(LED0_ON_L + 4 * channel + 3, scaled_pulse >> 8);
  }

  void PCA9685Driver::write_byte(uint8_t reg, uint8_t value)
  {
    uint8_t buffer[2] = {reg, value};
    if (write(i2c_fd_, buffer, 2) != 2)
    {
      throw std::runtime_error("Failed to write to I2C device.");
    }
  }

  uint8_t PCA9685Driver::read_byte(uint8_t reg)
  {
    uint8_t buffer = reg;
    if (write(i2c_fd_, &buffer, 1) != 1)
    {
      throw std::runtime_error("Failed to write register for reading.");
    }
    if (read(i2c_fd_, &buffer, 1) != 1)
    {
      throw std::runtime_error("Failed to read from I2C device.");
    }
    return buffer;
  }

} // namespace pca9685
