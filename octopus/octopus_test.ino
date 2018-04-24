#include <ESP8266WiFi.h>

#include "octopus.h"

Octopus octopus = Octopus();

void setup()
{
    delay(2000);
    pinMode(0, OUTPUT);
    Serial.begin(115200);
    Serial.println("Initializing");
    if (!octopus.Begin())
    {
        Serial.println("Setup failed reboot in 2s");
        delay(2000);
        digitalWrite(0, LOW);
        while (1);
    }
    
    delay(1000);
    
}

void loop()
{
    
    float temperature;
    float pressure;
    float humidity;
    float gas_resistance;

    if (octopus.ReadBme680(&temperature, &pressure, &humidity, &gas_resistance))
    {
        Serial.printf("%4.2f°  %6.3fhPa  %6.3f%%  %6.3fKΩ\n", temperature, pressure / 100, humidity,
                      gas_resistance / 1000);
    }

    uint8_t system_calibration;
    uint8_t gyroscope_calibration;
    uint8_t accelerometer_calibration;
    uint8_t magnetometer_calibration;
    if (octopus.ReadBno055Calibration(&system_calibration, &gyroscope_calibration, &accelerometer_calibration, &magnetometer_calibration))
    {
        Serial.printf("calibration: sys=%d  gyro=%d  accel=%d  mag=%d\n", system_calibration, gyroscope_calibration,
                      accelerometer_calibration, magnetometer_calibration);
    }
    else
    {
        Serial.println("Cant read BNO055 calibration");
    }

    imu::Vector<3> accelerometer;
    imu::Vector<3> magnetometer;
    imu::Vector<3> gyroscope;
    imu::Vector<3> euler;
    imu::Vector<3> linear_acceleration;
    imu::Vector<3> gravity;
    int8_t temperature2;
    if (octopus.ReadBno055(&accelerometer, &magnetometer, &gyroscope, &euler, &linear_acceleration, &gravity, &temperature2))
    {
      
      Serial.printf("      accelerometer: %6.2f / %6.2f / %6.2f\n", accelerometer.x(), accelerometer.y(), accelerometer.z());
      Serial.printf("       magnetometer: %6.2f / %6.2f / %6.2f\n", magnetometer.x(), magnetometer.y(), magnetometer.z());
      Serial.printf("          gyroscope: %6.2f / %6.2f / %6.2f\n", gyroscope.x(), gyroscope.y(), gyroscope.z());
      Serial.printf("              euler: %6.2f / %6.2f / %6.2f\n", euler.x(), euler.y(), euler.z());
      Serial.printf("linear_acceleration: %6.2f / %6.2f / %6.2f\n", linear_acceleration.x(), linear_acceleration.y(), linear_acceleration.z());
      Serial.printf("            gravity: %6.2f / %6.2f / %6.2f\n", gravity.x(), gravity.y(), gravity.z());
      Serial.printf("        temperature: %d\n", temperature2);

      euler.normalize();
      magnetometer.normalize();

      octopus.NeoPixel0UpdateColor(Adafruit_NeoPixel::Color((euler.x()+1)/2.0*255, (euler.y()+1)/2.0*255, (euler.z()+1)/2.0*255));
      octopus.NeoPixel1UpdateColor(Adafruit_NeoPixel::Color((magnetometer.x()+1)/2.0*255, (magnetometer.y()+1)/2.0*255, (magnetometer.z()+1)/2.0*255));

    }
    else
    {
      Serial.println("Can't read from BNO055");
      digitalWrite(0, LOW);
    }

    // octopus.NeoPixelUpdateColors(Adafruit_NeoPixel::Color(255, 0, 0), Adafruit_NeoPixel::Color(0, 255, 0));

    Serial.println("loop");
    delay(100);
    digitalWrite(0, HIGH);
}