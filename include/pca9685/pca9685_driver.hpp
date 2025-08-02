#ifndef PCA9685__PCA9685_DRIVER_HPP_
#define PCA9685__PCA9685_DRIVER_HPP_

#include <string>

namespace pca9685
{
  class PCA9685Driver
  {
  public:
    PCA9685Driver(int bus_number, int device_address);
    ~PCA9685Driver();

    void set_pwm_frequency(int frequency);
    void set_pulse(int channel, int pulse);

  private:
    void write_byte(uint8_t reg, uint8_t value);
    uint8_t read_byte(uint8_t reg);

    int i2c_fd_{-1}; // File descriptor for the I2C device
    int frequency_{60};
    double pwm_scale_{1.0};
  };
} // namespace pca9685

#endif // PCA9685__PCA9685_DRIVER_HPP_
