#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstdint>

#include <aim_can_driver.h>
#include <aim_network.h>
#include <aim_safety.h>

// Board-level identity and interface configuration lives in this file.
#define NODE_ORIGIN AIM_ORG_GPS
#define NODE_NAME "GPS_MODULE"

#define NODE_CAN_BAUD 500000U

#define NODE_CAN_BUS CAN1

#define NODE_SERIAL_RX_PIN PA10
#define NODE_SERIAL_TX_PIN PA9

#define NODE_SERIAL_BAUD 38400U

// Flash debug storage configuration (SPI flash on PB12-PB15).
#define NODE_FLASH_CS_PIN PB12
#define NODE_FLASH_SCK_PIN PB13
#define NODE_FLASH_MISO_PIN PB14
#define NODE_FLASH_MOSI_PIN PB15

// Number of columns stored in each flash table row.
#define NODE_FLASH_TABLE_COLS 1U
// Interval, in rows, between origin/metadata refresh operations.
#define NODE_FLASH_ORIGIN_REFRESH_INT 64U
// Total size, in bytes, reserved for the flash table.
#define NODE_FLASH_TABLE_SIZE 65536U
// Flash table instance index used by this node.
#define NODE_FLASH_TABLE_NUM 0U
// Scratch/data buffer size, in bytes, allocated in STM32 MCU RAM for flash table operations.
#define NODE_MCU_BUFFER_SIZE 256U

#define NODE_GPS_SCL_PIN PB6
#define NODE_GPS_SDA_PIN PB7
#define NODE_GPS_ADDR 0x42U

enum NodeState : uint8_t {
  INIT = 0U,
  OPERATIONAL = 1U,
  DEBUG_CONSOLE = 2U,
  FLASH_DUMP = 3U,
  SAFE_MODE = 4U,
  LOW_POWER = 5U,
  FAULT = 6U
};

struct GpsDebugSnapshot {
  bool parserTimeValid;
  bool parserLocationValid;
  bool parserSatellitesValid;
  bool hasValidTime;
  bool hasValidLocation;
  uint32_t timeOfDayMs;
  int64_t longitudeNano;
  int64_t latitudeNano;
  uint32_t satellites;
  uint32_t charsProcessed;
  uint32_t sentencesWithFix;
  uint32_t failedChecksum;
  uint32_t passedChecksum;
};

bool nodeGetGpsDebugSnapshot(GpsDebugSnapshot* out);

// Add board-specific periodic behavior in board_update().
void board_update(void);

#endif  // NODE_H
