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
constexpr char     kName[]     = "GPS_MODULE";
constexpr uint32_t kCanBaud    = 500000U;
constexpr uint32_t kSerialBaud = 38400U;
}  // namespace node

struct GpsDebugSnapshot {
  bool parserTimeValid;
  bool parserLocationValid;
  bool parserSatellitesValid;
  bool hasValidTime;
  bool hasValidLocation;
  uint32_t timeOfDayMs;
  char readableTimeStr[20];
  int64_t longitudeNano;
  int64_t latitudeNano;
  uint32_t satellites;
  uint32_t charsProcessed;
  uint32_t sentencesWithFix;
  uint32_t failedChecksum;
  uint32_t passedChecksum;
};

bool nodeGetGpsDebugSnapshot(GpsDebugSnapshot* out);

// Add node-specific periodic behavior in nodeUpdate().
void nodeInit(uint32_t nowMs);
void nodeUpdate(uint32_t schedulerNowMs);
void nodeServiceCanTx(uint32_t schedulerNowMs, AimNetwork& aim);
void nodeOnRx(const aim::Msg& m, uint32_t nowMs);

#endif  // NODE_H
