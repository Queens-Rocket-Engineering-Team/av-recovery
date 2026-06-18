#include "node.h"

#include <IWatchdog.h>
#include <logger.h>
#include <SoftwareSerial.h>

static constexpr uint32_t kWatchdogTimeoutUs  = 2000000U;
static constexpr uint8_t  kMaxRxFramesPerLoop = 8U;

static AimCanDriver g_canHw(node::kCanBaud, CAN1);
static AimNetwork g_aim(&g_canHw, aim::Source::Altimeter);
static SoftwareSerial g_serial(pins::kSerialRx, pins::kSerialTx);
static Logger g_log(g_serial, static_cast<uint8_t>(aim::Source::Altimeter), LogLevel::INFO);

static void serviceCanRx(void) {
  const uint32_t nowMs = millis();
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aim::Msg m = {};
    if (!g_aim.receive(m)) break;
    nodeOnRx(m, nowMs);
  }
}

void setup(void) {
  g_serial.begin(node::kSerialBaud);
  g_logger = &g_log;
  LOG_INFO("Boot %s source=%u", node::kName, static_cast<unsigned>(aim::Source::Altimeter));
  IWatchdog.begin(kWatchdogTimeoutUs);

  // Altimeter is a Sensor/Event publisher and TimeSync consumer.
  if (!g_aim.begin(aim::classBit(aim::Class::Time) |
                   aim::classBit(aim::Class::Heartbeat))) {
    LOG_ERROR("CAN init failed");
  }

  nodeInit(millis());
}

void loop(void) {
  const uint32_t schedulerNowMs = millis();

  // Core work runs every loop. Flight recorder + console get added with the
  // flash stack once sensor logging exists (see GPS_Module for the pattern).
  serviceCanRx();
  nodeUpdate(schedulerNowMs);
  nodeServiceCanTx(schedulerNowMs, g_aim);
  g_aim.service(schedulerNowMs, nodeCurrentState(), nodeErrorBits());   // heartbeat fills bus silence

  IWatchdog.reload();
}
