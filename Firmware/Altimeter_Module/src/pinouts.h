#pragma once
#include <Arduino.h>   // STM32 variant pin macros (PB8, PA1, ...)
#include <cstdint>

// pinouts.h — Altimeter Module (STM32F103CB) pin map.
// Values are MCU-native STM32 Arduino pin macros (e.g. PB9 = 25).
// Seeded from the legacy Altimeter_Module_old/pinouts.h — verify against the
// current board revision before relying on it.
namespace pins {

// --- CAN ---
constexpr uint8_t kCanRx = PB8;
constexpr uint8_t kCanTx = PB9;

// --- Sensor I2C (barometer + IMUs) ---
constexpr uint8_t kI2cScl     = PB10;
constexpr uint8_t kI2cSda     = PB11;
constexpr uint8_t kMs5611Addr  = 0x76;   // barometer
constexpr uint8_t kMpu6050Addr = 0x69;   // IMU
constexpr uint8_t kKx134Addr   = 0x1F;   // accelerometer

// --- IMU interrupts ---
constexpr uint8_t kAccelInt1 = PB5;
constexpr uint8_t kAccelInt2 = PB6;
constexpr uint8_t kGyroInt   = PB7;

// --- Flash (SPI) ---
constexpr uint8_t kFlashCs    = PB12;
constexpr uint8_t kSpiSclk    = PB13;
constexpr uint8_t kSpiMiso    = PB14;
constexpr uint8_t kSpiMosi    = PB15;
constexpr uint8_t kFlashReset = PA11;

// --- Recovery (pyro fire + continuity) ---
constexpr uint8_t kFireDrogue = PA1;
constexpr uint8_t kContDrogue = PA2;
constexpr uint8_t kFireMain   = PA3;
constexpr uint8_t kContMain   = PA4;

// --- Power sensing ---
constexpr uint8_t kBattSense = PB0;
constexpr uint8_t kCurrSense = PB1;

// --- Buzzer ---
constexpr uint8_t kBuzzerA = PA8;
constexpr uint8_t kBuzzerB = PA7;

// --- LEDs ---
constexpr uint8_t kRgbData   = PB3;
constexpr uint8_t kStatusLed = PA15;

// --- Serial (USART1) ---
constexpr uint8_t kSerialTx = PA9;
constexpr uint8_t kSerialRx = PA10;

// --- SWD debug ---
constexpr uint8_t kSwdio = PA13;
constexpr uint8_t kSwclk = PA14;

// --- Misc ---
constexpr uint8_t kBoot1 = PB2;

}  // namespace pins
