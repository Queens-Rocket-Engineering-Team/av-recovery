#include "node.h"

#include <logger.h>

enum class FlightPhase : uint8_t {
  Pad,
  Powered,
  Coast,
  Descent,
  Landed
};

static FlightPhase s_phase = FlightPhase::Pad;
static bool s_lowPower = false;
static bool s_sensorInitOk = true; // Stubbed sensor status check

void nodeInit(uint32_t nowMs) {
  // TODO: bring up the barometer (MS5611), IMUs (MPU6050 / KX134), and pyro
  // continuity sensing once their drivers exist.
  (void)nowMs;
  LOG_INFO("Altimeter initialization completed");
}

[[maybe_unused]] static void transitionTo(FlightPhase newPhase, AimNetwork& aim) {
  if (s_phase == newPhase) return;

  LOG_INFO("Altimeter phase transition: %d -> %d", static_cast<int>(s_phase), static_cast<int>(newPhase));
  s_phase = newPhase;

  if (s_phase == FlightPhase::Powered) {
    aim::Msg detectMsg = {};
    detectMsg.cls = aim::Class::Event;
    detectMsg.subject = aim::subject::LaunchDetect;
    detectMsg.b[0] = 1U;
    if (aim.send(detectMsg)) {
      LOG_INFO("LaunchDetect event published successfully");
    } else {
      LOG_ERROR("LaunchDetect event publication failed");
    }
  }
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
  (void)nowMs;
  if (m.cls == aim::Class::Event && m.subject == aim::subject::LowPower) {
    s_lowPower = (m.b[0] == 1U);
    LOG_INFO("Altimeter low power state updated: %d", s_lowPower);
  }
}

aim::NodeState nodeCurrentState() {
  if (!s_sensorInitOk) {
    return aim::NodeState::Fault;
  }
  return aim::NodeState::Nominal;
}

uint16_t nodeErrorBits() {
  uint16_t bits = 0U;
  if (!s_sensorInitOk) {
    bits |= (1U << 0);
  }
  return bits;
}
