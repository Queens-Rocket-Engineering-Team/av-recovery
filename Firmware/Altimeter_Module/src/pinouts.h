#ifndef PINOUTS_H
#define PINOUTS_H

#include <Arduino.h> // needed for PB/A# pin assignment
#include <cstdint>

// pinouts.h - STINGER Altimeter Module V2.0 Pin Mapping (STM32F103CB)
// Reference: STINGER V2.0 Hardware KiCad Schematic & PCB Layout (av-recovery/Schematics)
namespace pins {

// --- Serial (USB-UART Bridge; TX/RX are swapped on the PCB) ---
//  !!! Fix in next board spin !!!
//  Use software serial
constexpr uint8_t kSerialTx = PA9;  // USART_TX -> USB_RX
constexpr uint8_t kSerialRx = PA10; // USART_RX <- USB_TX

// --- CAN ---
constexpr uint8_t kCanRx = PB8;
constexpr uint8_t kCanTx = PB9;

// --- Flash (SPI) ---
constexpr uint8_t kFlashReset = PA11;
constexpr uint8_t kFlashCs    = PB12;
constexpr uint8_t kSpiSclk    = PB13;
constexpr uint8_t kSpiMiso    = PB14;
constexpr uint8_t kSpiMosi    = PB15;

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

// --- Recovery (pyro fire + continuity) ---
constexpr uint8_t kFireDrogue = PA1;
constexpr uint8_t kContDrogue = PA2;
constexpr uint8_t kFireMain   = PA3;
constexpr uint8_t kContMain   = PA4;

// --- Power sensing ---
constexpr uint8_t kBattSense = PB0;
constexpr uint8_t kCurrSense = PB1;

// --- Buzzer (TIM1 complementary PWM pair) ---
constexpr uint8_t kBuzzerB = PA7;
constexpr uint8_t kBuzzerA = PA8;

// --- LEDs ---
constexpr uint8_t kRgbData  = PB3;   // JTAG pin, requires JTAG disable/remap
constexpr uint8_t kDebugLed = PA15;  // JTAG pin, requires JTAG disable/remap

}

#endif // PINOUTS_H

