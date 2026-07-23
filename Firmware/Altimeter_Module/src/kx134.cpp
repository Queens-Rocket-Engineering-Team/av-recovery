// kx134.cpp - Kionix KX134-1211 Tri-axis High-G Accelerometer Driver
// Reference: Kionix KX134-1211 Technical Reference Manual & Datasheet

#include "kx134.h"
#include <Wire.h>

namespace {
// Register Map (KX134 Datasheet Section 8)
constexpr uint8_t kRegWhoAmI  = 0x13U;  // Hardware ID register (returns 0x46)
constexpr uint8_t kRegXoutL   = 0x06U;  // Output data start register (XOUT_L)
constexpr uint8_t kRegCntl1   = 0x1BU;  // Control Register 1 (PC1, RES, GSEL bits)
constexpr uint8_t kRegCntl2   = 0x1EU;  // Control Register 2 (SRST bit)

// Hardware Identification (KX134 Datasheet Table 6)
constexpr uint8_t kExpectedWhoAmI = 0x46U;

// Control Register Bitmasks (KX134 Datasheet Section 8)
constexpr uint8_t kBitCntl2Srst    = 0x80U;  // SRST bit: 1 = Software reset
constexpr uint8_t kBitCntl1Pc1     = 0x80U;  // PC1 bit: 1 = Active operating mode, 0 = Stand-by
constexpr uint8_t kBitCntl1Res     = 0x40U;  // RES bit: 1 = 16-bit high resolution, 0 = 8-bit
constexpr uint8_t kBitCntl1Gsel64g = 0x18U;  // GSEL[1:0] bits (4:3): 11 = +/-64g dynamic range

// Active Configuration: 16-bit High Resolution + 64g Range + Active Mode (0xD8)
constexpr uint8_t kCntl1Config = kBitCntl1Pc1 | kBitCntl1Res | kBitCntl1Gsel64g;

constexpr uint32_t kResetDelayMs   = 20U;  // Post-reset settling time in ms
constexpr uint8_t  kReadBytes3Axis = 6U;   // 2 bytes per axis * 3 axes (X, Y, Z)

// Sensitivity Scale Factor at +/-64g range: 512 LSB/g (KX134 Datasheet Table 1).
// Standard gravity 1g = 9.80665 m/s^2 = 9,807 mm/s^2.
// Scale formula: acceleration (mm/s^2) = rawCount * 9807 / 512
constexpr int32_t kScaleNumerator   = 9807L;
constexpr int32_t kScaleDenominator = 512L;
}  // namespace

Kx134::Kx134(uint8_t i2cAddr) : m_i2cAddr(i2cAddr), m_initialized(false) {}

bool Kx134::begin() {
  Wire.beginTransmission(m_i2cAddr);
  Wire.write(kRegWhoAmI);
  if (Wire.endTransmission(false) != 0U) {
    m_initialized = false;
    return false;
  }

  if (Wire.requestFrom(m_i2cAddr, static_cast<uint8_t>(1U)) != 1U) {
    m_initialized = false;
    return false;
  }

  const uint8_t who = Wire.read();
  if (who != kExpectedWhoAmI) {
    m_initialized = false;
    return false;
  }

  // Issue software reset
  Wire.beginTransmission(m_i2cAddr);
  Wire.write(kRegCntl2);
  Wire.write(kBitCntl2Srst);
  Wire.endTransmission();
  delay(kResetDelayMs);

  // Configure operating mode, resolution, and range
  Wire.beginTransmission(m_i2cAddr);
  Wire.write(kRegCntl1);
  Wire.write(kCntl1Config);
  if (Wire.endTransmission() != 0U) {
    m_initialized = false;
    return false;
  }

  m_initialized = true;
  return true;
}

bool Kx134::readAccelMmS2(int32_t& accelX, int32_t& accelY, int32_t& accelZ) {
  if (!m_initialized) return false;

  Wire.beginTransmission(m_i2cAddr);
  Wire.write(kRegXoutL);
  if (Wire.endTransmission(false) != 0U) return false;

  if (Wire.requestFrom(m_i2cAddr, kReadBytes3Axis) != kReadBytes3Axis) return false;

  const uint8_t xl = Wire.read();
  const uint8_t xh = Wire.read();
  const uint8_t yl = Wire.read();
  const uint8_t yh = Wire.read();
  const uint8_t zl = Wire.read();
  const uint8_t zh = Wire.read();

  const int16_t rawX = static_cast<int16_t>((xh << 8) | xl);
  const int16_t rawY = static_cast<int16_t>((yh << 8) | yl);
  const int16_t rawZ = static_cast<int16_t>((zh << 8) | zl);

  accelX = (static_cast<int32_t>(rawX) * kScaleNumerator) / kScaleDenominator;
  accelY = (static_cast<int32_t>(rawY) * kScaleNumerator) / kScaleDenominator;
  accelZ = (static_cast<int32_t>(rawZ) * kScaleNumerator) / kScaleDenominator;

  return true;
}

int32_t Kx134::readZAccelMmS2() {
  int32_t x = 0, y = 0, z = 0;
  if (readAccelMmS2(x, y, z)) {
    return z;
  }
  return 0;
}
