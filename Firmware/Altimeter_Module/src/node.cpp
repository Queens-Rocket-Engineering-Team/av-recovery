#include "node.h"

#include <logger.h>

// Scaffold only: this node heartbeats and consumes TimeSync. The bodies below
// fill in as the altimeter's drivers land — keep application logic here, not in
// main.cpp (see the firmware node conventions in the workspace CLAUDE.md).

void nodeInit(uint32_t nowMs) {
  // TODO: bring up the barometer (MS5611), IMUs (MPU6050 / KX134), and pyro
  // continuity sensing once their drivers exist.
  (void)nowMs;
}

void nodeUpdate(uint32_t schedulerNowMs) {
  // TODO: read sensors and run the apogee / recovery state machine.
  (void)schedulerNowMs;
}

void nodeServiceCanTx(uint32_t schedulerNowMs, AimNetwork& aim) {
  // TODO: publish altitude / acceleration Sensor frames and recovery Events.
  (void)schedulerNowMs;
  (void)aim;
}

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  // TODO: handle cross-node events (e.g. power state) once subjects are defined.
  (void)m;
  (void)nowMs;
}
