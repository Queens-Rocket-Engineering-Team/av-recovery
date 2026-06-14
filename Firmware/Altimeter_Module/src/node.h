#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstdint>

#include <aim_can_driver.h>
#include <aim_network.h>
#include <aim_safety.h>

#include "pinouts.h"

// Node-level identity and interface configuration lives in this file.
namespace node {
constexpr char     kName[]     = "ALTIMETER_MODULE";
constexpr uint32_t kCanBaud    = 500000U;
constexpr uint32_t kSerialBaud = 38400U;
}  // namespace node

// Application logic entry points. Bodies are stubs until the altimeter's
// barometer/IMU sensing and pyro/recovery control are implemented.
void nodeInit(uint32_t nowMs);
void nodeUpdate(uint32_t schedulerNowMs);
void nodeServiceCanTx(uint32_t schedulerNowMs, AimNetwork& aim);
void nodeOnRx(const aim::Msg& m, uint32_t nowMs);

#endif  // NODE_H
