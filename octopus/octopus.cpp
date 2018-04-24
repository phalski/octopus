#include "octopus.h"

namespace
{
static const float kTemparatureOffset = -8;

float readBme680TemperatureCelsius(const Adafruit_BME680 &bme680)
{
    return bme680.temperature + kTemparatureOffset;
}

float readBme680PressurePascal(const Adafruit_BME680 &bme680)
{
    return bme680.pressure;
}

float readBme680HumidityPercent(const Adafruit_BME680 &bme680)
{
    return bme680.humidity;
}

float readBme680GasResistanceOhm(const Adafruit_BME680 &bme680)
{
    return bme680.gas_resistance;
}

void updatePixelColor(Adafruit_NeoPixel &neo_pixel, uint16_t n, uint32_t c)
{
    neo_pixel.setPixelColor(n, c);
    neo_pixel.show();
}

bool setupBme680(Adafruit_BME680 &bme680)
{
    if (!bme680.begin(0x76))
        return false;

    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150);  // 320*C for 150 ms

    return true;
}

bool setupBno055(Adafruit_BNO055 &bno055)
{
    if (!bno055.begin())
        return false;

    bno055.setExtCrystalUse(true);

    return true;
}

bool setupNeoPixel(Adafruit_NeoPixel &neo_pixel)
{
    neo_pixel.begin();
    neo_pixel.setBrightness(50);  // 50%

    for (uint16_t i = 0; i < neo_pixel.numPixels(); ++i)
    {
        neo_pixel.show();
    }

    return true;
}
}

Octopus::Octopus()
    : neo_pixel_(new Adafruit_NeoPixel(2, 13, NEO_GRBW + NEO_KHZ800)),
      bme680_(new Adafruit_BME680()),
      bno055_(new Adafruit_BNO055(55))
{
}

Octopus::~Octopus()
{
    delete neo_pixel_;
    delete bme680_;
    delete bno055_;
}

bool Octopus::Begin()
{
    bool ok = true;

    if (NULL != neo_pixel_)
    {
        if (!setupNeoPixel(*neo_pixel_))
        {
            Serial.println("NeoPixel setup failed");
            ok = false;
        }
    }

    if (NULL != bme680_)
    {
        if (!setupBme680(*bme680_))
        {
            Serial.println("BME680 setup failed");
            ok = false;
        }
    }

    if (NULL != bno055_)
    {
        if (!setupBno055(*bno055_))
        {
            Serial.println("BNO055 setup failed");
            ok = false;
        }
    }

    return ok;
}

bool Octopus::Bno055IsCalibrated()
{
    uint8_t system_calibration;
    return ReadBno055Calibration(&system_calibration, NULL, NULL, NULL) && system_calibration;
}

bool Octopus::NeoPixel0UpdateColor(uint32_t color)
{
    if (NULL == neo_pixel_)
    {
        return false;
    }

    updatePixelColor(*neo_pixel_, 0, color);

    return true;
}

bool Octopus::NeoPixel1UpdateColor(uint32_t color)
{
    if (NULL == neo_pixel_)
    {
        return false;
    }

    updatePixelColor(*neo_pixel_, 1, color);

    return true;
}

bool Octopus::NeoPixelUpdateColors(uint32_t color_neo_pixel_0, uint32_t color_neo_pixel_1)
{
    if (NULL == neo_pixel_)
    {
        return false;
    }

    neo_pixel_->setPixelColor(0, color_neo_pixel_0);
    neo_pixel_->setPixelColor(1, color_neo_pixel_1);
    neo_pixel_->show();

    return true;
}

bool Octopus::ReadBme680(float *temperature_celsius,
                         float *pressure_pascal,
                         float *humidity_percent,
                         float *gas_resistance_ohms)
{
    if (NULL == bme680_ || !bme680_->performReading())
    {
        return false;
    }

    if (NULL != temperature_celsius)
    {
        *temperature_celsius = bme680_->temperature;
    }

    if (NULL != pressure_pascal)
    {
        *pressure_pascal = bme680_->pressure;
    }

    if (NULL != humidity_percent)
    {
        *humidity_percent = bme680_->humidity;
    }

    if (NULL != gas_resistance_ohms)
    {
        *gas_resistance_ohms = bme680_->gas_resistance;
    }

    return true;
}

bool Octopus::ReadBno055(imu::Vector<3> *accelerometer,
                         imu::Vector<3> *magnetometer,
                         imu::Vector<3> *gyroscope,
                         imu::Vector<3> *euler,
                         imu::Vector<3> *linear_acceleration,
                         imu::Vector<3> *gravity,
                         int8_t *temperature_celsius)
{
    if (!Bno055IsCalibrated())
    {
        return false;
    }

    if (NULL != accelerometer)
    {
        *accelerometer = bno055_->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    }

    if (NULL != magnetometer)
    {
        *magnetometer = bno055_->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    }

    if (NULL != gyroscope)
    {
        *gyroscope = bno055_->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    }

    if (NULL != euler)
    {
        *euler = bno055_->getVector(Adafruit_BNO055::VECTOR_EULER);
    }

    if (NULL != linear_acceleration)
    {
        *linear_acceleration = bno055_->getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    }

    if (NULL != gravity)
    {
        *gravity = bno055_->getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    }

    if (NULL != temperature_celsius)
    {
        *temperature_celsius = bno055_->getTemp();
    }

    return true;
}

bool Octopus::ReadBno055Calibration(uint8_t *system,
                                    uint8_t *gyroscope,
                                    uint8_t *accelerometer,
                                    uint8_t *magnetometer)
{
    if (NULL == bno055_)
    {
        return false;
    }

    bno055_->getCalibration(system, gyroscope, accelerometer, magnetometer);
    return true;
}