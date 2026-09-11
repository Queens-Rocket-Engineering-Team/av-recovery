#ifndef PINOUTS_H
#define PINOUTS_H

#include <Arduino.h> // needed for PB/A# pin assignment
#include <cstdint>

// pinouts.h - MASANOV GPS Module V2.0 2024/2025 (STM32F103CB) pin map.
namespace pins {

// --- Serial (USB-UART Bridge; TX/RX are swapped on the PCB) ---
//  !!! Fix in next board spin !!!
//  Use software serial
constexpr uint8_t kSerialTx = PA9;  // USART_TX -> USB_RX
constexpr uint8_t kSerialRx = PA10; // USART_RX <- USB_TX

// --- CAN bus ---
constexpr uint8_t kCanRx = PB8;
constexpr uint8_t kCanTx = PB9;

// --- Flash (SPI) ---
constexpr uint8_t kFlashReset = PA11;
constexpr uint8_t kFlashCs    = PB12;
constexpr uint8_t kSpiSclk    = PB13;
constexpr uint8_t kSpiMiso    = PB14;
constexpr uint8_t kSpiMosi    = PB15;

// --- GPS interface (I2C + control) ---
constexpr uint8_t kGpsReset  = PB4;  // RST_GPS; JTAG pin, requires JTAG disable/remap
constexpr uint8_t kGpsExtInt = PB5;  // EXTINT_GPS
constexpr uint8_t kGpsScl    = PB6;  // SCL_GPS
constexpr uint8_t kGpsSda    = PB7;  // SDA_GPS
constexpr uint8_t kGpsAddr   = 0x42;

// --- Buzzer (TIM1 complementary PWM pair) ---
constexpr uint8_t kBuzzerB = PA7;
constexpr uint8_t kBuzzerA = PA8;

// --- LEDs ---
constexpr uint8_t kRgbData  = PB3;   // JTAG pin, requires JTAG disable/remap
constexpr uint8_t kDebugLed = PA15;  // JTAG pin, requires JTAG disable/remap

// --- Test points ---
constexpr uint8_t kTp10 = PA2;
constexpr uint8_t kTp11 = PA3;
constexpr uint8_t kTp4  = PA4;

}

#endif // PINOUTS_H
