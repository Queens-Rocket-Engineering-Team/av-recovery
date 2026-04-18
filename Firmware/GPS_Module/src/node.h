#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstdint>

#include <aim_can_driver.h>
#include <aim_network.h>
#include <aim_safety.h>

// Board-level identity and interface configuration lives in this file.
#define NODE_ORIGIN AIM_ORG_GPS

#define NODE_CAN_BAUD 500000U

#if defined(CAN1)
#define NODE_CAN_BUS CAN1
#else
#error "Define NODE_CAN_BUS for this STM32 target."
#endif

#define NODE_SERIAL_RX_PIN PA10
#define NODE_SERIAL_TX_PIN PA9

#define NODE_SERIAL_BAUD 38400U

#define NODE_GPS_SCL_PIN PB6
#define NODE_GPS_SDA_PIN PB7
#define NODE_GPS_ADDR 0x42U

enum NodeState : uint8_t {
  INIT = 0U,
  OPERATIONAL = 1U,
  DEGRADED = 2U,
  SAFE_MODE = 3U,
  FAULT = 4U
};

void board_init(void);
// Add board-specific periodic behavior in board_update().
void board_update(NodeState state);

#endif  // NODE_H
