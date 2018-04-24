
#ifndef OCTOPUS_H
#define OCTOPUS_H

#include <Adafruit_BME680.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_NeoPixel.h>

class Octopus
{
  public:
    Octopus();
    ~Octopus();

    bool Begin();

    bool NeoPixel0UpdateColor(uint32_t color);
    bool NeoPixel1UpdateColor(uint32_t color);
    bool NeoPixelUpdateColors(uint32_t color_neo_pixel_0, uint32_t color_neo_pixel_1);

    bool ReadBme680(float *temperature_celsius,
                    float *pressure_pascal,
                    float *humidity_percent,
                    float *gas_resistance_ohms);

    bool ReadBno055(imu::Vector<3> *accelerometer,
                    imu::Vector<3> *magnetometer,
                    imu::Vector<3> *gyroscope,
                    imu::Vector<3> *euler,
                    imu::Vector<3> *linear_acceleration,
                    imu::Vector<3> *gravity,
                    int8_t *temperature_celsius);
    bool ReadBno055Calibration(uint8_t *system,
                               uint8_t *gyroscope,
                               uint8_t *accelerometer,
                               uint8_t *magnetometer);
    bool Bno055IsCalibrated();

    const Adafruit_NeoPixel *neo_pixel() { return neo_pixel_; }
    const Adafruit_BME680 *bme680() { return bme680_; }
    const Adafruit_BNO055 *bno055() { return bno055_; }

  private:
    Adafruit_NeoPixel *neo_pixel_;
    Adafruit_BME680 *bme680_;
    Adafruit_BNO055 *bno055_;
};

#endif  // OCTOPUS_H