#ifndef KX134_H
#define KX134_H

#include <Arduino.h>
#include <cstdint>

// Lightweight, dependency-free driver for the Kionix KX134-1211 64g accelerometer.
class Kx134 {
 public:
  explicit Kx134(uint8_t i2cAddr = 0x1F);

  // Initializes the sensor, verifies WHO_AM_I, performs soft-reset, and sets 64g high-res mode.
  bool begin();

  // Reads 3-axis acceleration in mm/s^2.
  bool readAccelMmS2(int32_t& accelX, int32_t& accelY, int32_t& accelZ);

  // Quick helper for Z-axis acceleration in mm/s^2.
  int32_t readZAccelMmS2();

 private:
  uint8_t m_i2cAddr;
  bool m_initialized;
};

#endif  // KX134_H
