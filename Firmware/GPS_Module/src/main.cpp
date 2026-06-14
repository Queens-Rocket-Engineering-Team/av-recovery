#include "node.h"

#include <IWatchdog.h>
#include <logger.h>
#include <SoftwareSerial.h>
#include <SPI.h>

#include <aim_file_system.h>
#include <aim_flight_recorder.h>
#ifndef FLIGHT_BUILD
#include <aim_console.h>
#endif

static constexpr uint32_t kWatchdogTimeoutUs = 2000000U;
static constexpr uint8_t kMaxRxFramesPerLoop = 8U;

// Flight-recorder geometry. No telemetry rows are written yet; the recorder
// exists so the console can dump/erase. Headers must have static lifetime.
static constexpr uint8_t  kLogCols           = 1U;
static constexpr uint16_t kLogOriginRefresh  = 64U;
static constexpr uint32_t kLogMaxSize        = 1UL * 1024UL * 1024UL;
static const char* const  kLogHeaders[kLogCols] = {"time"};

static AimCanDriver g_canHw(node::kCanBaud, CAN1);
static AimNetwork g_aim(&g_canHw, aim::Source::Gps);
static SoftwareSerial g_serial(pins::kSerialRx, pins::kSerialTx);
static Logger g_log(g_serial, static_cast<uint8_t>(aim::Source::Gps), LogLevel::INFO);

// Flash on SPI2: MOSI=PB15, MISO=PB14, SCLK=PB13, CS=PB12 (see pinouts.h).
static SPIClass g_flashSpi(pins::kSpiMosi, pins::kSpiMiso, pins::kSpiSclk);
static SpiNorFlashDriver g_flashDriver(pins::kFlashCs, g_flashSpi);
static AimFileSystem g_fs(&g_flashDriver);
static AimFlightRecorder g_recorder(g_fs, kLogCols, kLogOriginRefresh, kLogMaxSize, kLogHeaders);

static void serviceCanRx(void) {
  const uint32_t nowMs = millis();
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aim::Msg m = {};
    if (!g_aim.receive(m)) break;
    nodeOnRx(m, nowMs);
  }
}

#ifndef FLIGHT_BUILD
static void hookStatus(Stream& out) {
  out.print("name=");
  out.print(node::kName);
  out.print(" logMask=0x");
  out.print(static_cast<unsigned>(g_log.filterMask()), HEX);
  out.print(" syncedMs=");
  out.print(static_cast<unsigned long>(g_aim.syncedMillis()));
  out.print(" version=");
  out.print(aim::kNetworkVersionString);
  out.print(" schema=");
  out.print(static_cast<unsigned>(aim::kSchemaVersion));
  out.print(" build=");
  out.print(__DATE__);
  out.print(" ");
  out.println(__TIME__);
}

static void hookGpsSnapshot(Stream& out) {
  GpsDebugSnapshot gps = {};
  if (!nodeGetGpsDebugSnapshot(&gps)) {
    out.println("gps snapshot unavailable");
    return;
  }

  out.print("gps timeValid(parser/state)=");
  out.print(static_cast<unsigned>(gps.parserTimeValid ? 1U : 0U));
  out.print("/");
  out.println(static_cast<unsigned>(gps.hasValidTime ? 1U : 0U));

  out.print("gps locValid(parser/state)=");
  out.print(static_cast<unsigned>(gps.parserLocationValid ? 1U : 0U));
  out.print("/");
  out.println(static_cast<unsigned>(gps.hasValidLocation ? 1U : 0U));

  out.print("gps sats(valid/count)=");
  out.print(static_cast<unsigned>(gps.parserSatellitesValid ? 1U : 0U));
  out.print("/");
  out.println(static_cast<unsigned long>(gps.satellites));

  out.print("timeOfDayMs=");
  out.println(static_cast<unsigned long>(gps.timeOfDayMs));

  out.print("lonNano=");
  out.println(static_cast<long long>(gps.longitudeNano));
  out.print("latNano=");
  out.println(static_cast<long long>(gps.latitudeNano));
}

static void hookGpsParserStats(Stream& out) {
  GpsDebugSnapshot gps = {};
  if (!nodeGetGpsDebugSnapshot(&gps)) {
    out.println("gps parser stats unavailable");
    return;
  }

  out.print("chars=");
  out.println(static_cast<unsigned long>(gps.charsProcessed));
  out.print("sentencesWithFix=");
  out.println(static_cast<unsigned long>(gps.sentencesWithFix));
  out.print("checksum pass/fail=");
  out.print(static_cast<unsigned long>(gps.passedChecksum));
  out.print("/");
  out.println(static_cast<unsigned long>(gps.failedChecksum));
}

static const AimConsoleHook kConsoleHooks[] = {
  {'s', "status", hookStatus},
  {'g', "gps snapshot", hookGpsSnapshot},
  {'p', "gps parser stats", hookGpsParserStats},
};
#endif  // FLIGHT_BUILD

void setup(void) {
  g_serial.begin(node::kSerialBaud);
  g_logger = &g_log;
  LOG_INFO("Boot %s source=%u", node::kName, static_cast<unsigned>(aim::Source::Gps));
  IWatchdog.begin(kWatchdogTimeoutUs);
  LOG_INFO("Watchdog ready");

  // GPS is a TimeSync consumer; it accepts Time (to discipline its clock) and
  // Heartbeat. It publishes Sensor frames but does not need to receive them.
  if (!g_aim.begin(aim::classBit(aim::Class::Time) |
                   aim::classBit(aim::Class::Heartbeat))) {
    LOG_ERROR("CAN init failed");
  }

  if (!g_fs.begin()) {
    LOG_WARN("Filesystem mount failed");
  } else if (!g_recorder.begin()) {
    LOG_WARN("Recorder init failed");
  } else {
    LOG_INFO("Flash ready");
  }

#ifndef FLIGHT_BUILD
  aimConsoleInit(g_serial, g_fs, g_recorder, node::kName, kConsoleHooks,
                 static_cast<uint8_t>(sizeof(kConsoleHooks) / sizeof(kConsoleHooks[0])));
#endif

  nodeInit(millis());
#ifndef FLIGHT_BUILD
  g_serial.println("Console ready. d=enter debug");
#endif
}

void loop(void) {
  const uint32_t schedulerNowMs = millis();

  // Core work runs every loop, even while the console is active.
  serviceCanRx();
  nodeUpdate(schedulerNowMs);                   // GPS I2C read + parse
  nodeServiceCanTx(schedulerNowMs, g_aim);      // GPS position fix, 1 Hz
  g_aim.service(nodeCurrentState(), nodeErrorBits());   // heartbeat fills bus silence

#ifndef FLIGHT_BUILD
  aimConsoleService();                           // owns console + flash dump/erase
#endif

  IWatchdog.reload();
}
