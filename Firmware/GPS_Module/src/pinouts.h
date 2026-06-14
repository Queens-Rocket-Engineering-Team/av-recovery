#pragma once

#include <Arduino.h>  // STM32 variant pin macros (PA9, PB6, ...)
#include <cstdint>

// pinouts.h - GPS Module (STM32F103CBT6) pin map.
// Values are MCU-native STM32 Arduino pin macros.
namespace pins {

// --- USB-UART bridge (USART1, names are from the MCU perspective) ---
//   PA9  / USART1_TX -> USB_TX -> bridge RXD
//   PA10 / USART1_RX <- USB_RX <- bridge TXD
constexpr uint8_t kSerialTx = PA9;
constexpr uint8_t kSerialRx = PA10;

// --- GPS interface (I2C + control) ---
constexpr uint8_t kGpsReset  = PB4;  // RST_GPS; JTAG pin, requires JTAG disable/remap
constexpr uint8_t kGpsExtInt = PB5;  // EXTINT_GPS
constexpr uint8_t kGpsScl    = PB6;  // SCL_GPS
constexpr uint8_t kGpsSda    = PB7;  // SDA_GPS
constexpr uint8_t kGpsAddr   = 0x42;

// --- Flash (SPI) ---
constexpr uint8_t kFlashReset = PA11;  // RESET_FL
constexpr uint8_t kFlashCs    = PB12;  // CS_FL
constexpr uint8_t kSpiSclk    = PB13;  // SCK_FL
constexpr uint8_t kSpiMiso    = PB14;  // MISO_FL
constexpr uint8_t kSpiMosi    = PB15;  // MOSI_FL

// --- CAN bus ---
constexpr uint8_t kCanRx = PB8;
constexpr uint8_t kCanTx = PB9;

// --- Buzzer (TIM1 complementary PWM pair) ---
constexpr uint8_t kBuzzerB = PA7;  // TIM1_CH1N
constexpr uint8_t kBuzzerA = PA8;  // TIM1_CH1

// --- LEDs ---
constexpr uint8_t kRgbData  = PB3;   // JTAG pin, requires JTAG disable/remap
constexpr uint8_t kDebugLed = PA15;  // JTAG pin, requires JTAG disable/remap

// --- Boot and SWD debug ---
constexpr uint8_t kBoot1 = PB2;
constexpr uint8_t kSwdio = PA13;
constexpr uint8_t kSwclk = PA14;

// --- Test points ---
constexpr uint8_t kTp10 = PA2;
constexpr uint8_t kTp11 = PA3;
constexpr uint8_t kTp4  = PA4;

}  // namespace pins
