#include "Arduino.h"
#include "RGBLed.h"
#include <Wire.h>

// RGB LED Pins and Type
#define RGBLED_PIN_B 2
#define RGBLED_PIN_G 3
#define RGBLED_PIN_R 4
#define rgbLed_TYPE COMMON_ANODE
RGBLed rgbLed(RGBLED_PIN_R, RGBLED_PIN_G, RGBLED_PIN_B, rgbLed_TYPE);

// MPU-9250 I2C Address and Registers
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up MPU-9250
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); // Wake up
  Wire.endTransmission(true);

  // Flash LED white for 500ms
  rgbLed.setRGB(255, 255, 255);
  delay(500);
  rgbLed.turnOff();

  Serial.println("MPU-9250 RGB LED Accelerometer Visualizer Initialized");
}

void loop() {
  int16_t ax, ay, az;
  readAcceleration(ax, ay, az);

  // Normalize to g units (assuming ±2g range)
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  // Compute acceleration magnitude
  float a_mag = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  // Print to serial
  Serial.print("Acceleration Magnitude: ");
  Serial.print(a_mag, 2);
  Serial.println(" g");

  // Update LED color
  setColorFromAccel(a_mag);

  delay(100);
}

void readAcceleration(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
}

void setColorFromAccel(float a_mag) {
  a_mag = constrain(a_mag, 0.0, 3.0);

  int r = 0, g = 0, b = 0;

  if (a_mag < 1.5) {
    float t = a_mag / 1.5;
    r = 0;
    g = int(255 * t);
    b = int(255 * (1 - t));
  } else {
    float t = (a_mag - 1.5) / 1.5;
    r = int(255 * t);
    g = int(255 * (1 - t));
    b = 0;
  }

  rgbLed.setRGB(r, g, b);
}
