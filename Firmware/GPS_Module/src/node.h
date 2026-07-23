#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstdint>

#include <aim_network.h>

#include "pinouts.h"

class AimFlightRecorder;

// Node-level identity and interface configuration lives in this file.
namespace node {
constexpr char kName[] = "GPS_MODULE";
constexpr aim::Source kSource = aim::Source::Gps;
constexpr uint32_t kCanBaud = 1000000U;
constexpr uint32_t kSerialBaud = 38400U;
}  // namespace node

// Add node-specific periodic behavior in nodeUpdate().
void nodeInit();
void nodeUpdate(uint32_t nowMs);
void nodeServiceLog(uint32_t nowMs, AimFlightRecorder& recorder);
void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim);
void nodeOnRx(const aim::Msg& m, uint32_t nowMs);

// Dynamic data rate management (broadcasting TelemetryMode over CAN).
void nodeSetTelemetryMode(bool active, AimNetwork& aim);

aim::NodeState nodeCurrentState();
uint16_t nodeErrorBits();

#ifndef FLIGHT_BUILD
#include <aim_console.h>
const AimConsoleHook* nodeConsoleHooks(uint8_t& count);
#endif

#endif  // NODE_H
