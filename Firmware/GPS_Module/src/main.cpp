#include "node.h"
#include "console.h"

#include <IWatchdog.h>
#include <logger.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <flash_table.h>
#include <Wire.h>
#include <TinyGPS++.h>

static constexpr uint32_t kTimeSyncTxIntervalMs = 250U;
static constexpr uint32_t kGpsCoordTxIntervalMs = 1000U;
static constexpr uint32_t kWatchdogTimeoutUs = 2000000U;
static constexpr uint8_t kMaxRxFramesPerLoop = 8U;
static constexpr uint8_t kGpsReadChunkBytes = 64U;
static constexpr uint32_t kGpsNoDataWarnAfterMs = 5000U;
static constexpr uint32_t kGpsI2cErrorLogIntervalMs = 5000U;
static constexpr int64_t kGpsDegreesToNanoScale = 1000000000LL;

struct GpsState {
  TinyGPSPlus parser;
  bool hasValidTime = false;
  bool hasValidLocation = false;
  uint32_t timeOfDayMs = 0U;
  int64_t longitudeNano = 0LL;
  int64_t latitudeNano = 0LL;
  uint32_t lastI2cErrorLogMs = 0U;
  bool loggedNoDataWarning = false;
  bool loggedNoGpsTimeWarning = false;
  bool loggedNoGpsLocationWarning = false;
};

struct NodeSchedulerState {
  NodeState value = INIT;
  uint32_t lastHeartbeatTxMs = 0U;
  uint32_t lastTimeSyncTxMs = 0U;
  uint32_t lastGpsCoordTxMs = 0U;
};

static AimCanDriver g_canHw(NODE_ORIGIN, NODE_CAN_BAUD, NODE_CAN_BUS);
static AimNetwork g_aim(&g_canHw, NODE_ORIGIN);
static SoftwareSerial g_serial(NODE_SERIAL_RX_PIN, NODE_SERIAL_TX_PIN);
static Logger g_log(g_serial, NODE_ORIGIN, LogLevel::INFO);
static uint32_t g_flashLastVals[NODE_FLASH_TABLE_COLS];
static uint8_t g_flashIoBuffer[NODE_MCU_BUFFER_SIZE];
static FlashTable g_flashTable(
  &SerialFlash,
  NODE_FLASH_TABLE_COLS,
  NODE_FLASH_ORIGIN_REFRESH_INT,
  NODE_FLASH_TABLE_SIZE,
  NODE_FLASH_TABLE_NUM,
  NODE_MCU_BUFFER_SIZE,
  g_flashLastVals,
  g_flashIoBuffer);
static NodeSchedulerState g_schedulerState = {};
static GpsState g_gpsState = {};

bool nodeGetGpsDebugSnapshot(GpsDebugSnapshot* out) {
  if (out == nullptr) {
    return false;
  }

  out->parserTimeValid = g_gpsState.parser.time.isValid();
  out->parserLocationValid = g_gpsState.parser.location.isValid();
  out->parserSatellitesValid = g_gpsState.parser.satellites.isValid();
  out->hasValidTime = g_gpsState.hasValidTime;
  out->hasValidLocation = g_gpsState.hasValidLocation;
  out->timeOfDayMs = g_gpsState.timeOfDayMs;
  out->longitudeNano = g_gpsState.longitudeNano;
  out->latitudeNano = g_gpsState.latitudeNano;
  out->satellites = g_gpsState.parser.satellites.value();
  out->charsProcessed = g_gpsState.parser.charsProcessed();
  out->sentencesWithFix = g_gpsState.parser.sentencesWithFix();
  out->failedChecksum = g_gpsState.parser.failedChecksum();
  out->passedChecksum = g_gpsState.parser.passedChecksum();
  return true;
}

void service_can_rx(uint32_t networkNowMs) {
  // Handle incoming bus messages and custom packet branches here.
  (void)networkNowMs;
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aimPkt pkt = {};
    if (!g_aim.readPkt(pkt)) {
      break;
    }
  }
}

uint32_t gpsTimeToCentiseconds(TinyGPSTime& gpsTime) {
  const uint32_t hh = static_cast<uint32_t>(gpsTime.hour());
  const uint32_t mm = static_cast<uint32_t>(gpsTime.minute());
  const uint32_t ss = static_cast<uint32_t>(gpsTime.second());
  const uint32_t cs = static_cast<uint32_t>(gpsTime.centisecond());

  return ((((hh * 60U) + mm) * 60U) + ss) * 100U + cs;
}

int64_t rawDegreesToNano(const RawDegrees& raw) {
  int64_t nano = static_cast<int64_t>(raw.deg) * kGpsDegreesToNanoScale;
  nano += static_cast<int64_t>(raw.billionths);
  if (raw.negative) {
    nano = -nano;
  }

  return nano;
}

void poll_gps_i2c(void) {
  // The GPS module exposes a single data register at 0xFF.
  Wire.beginTransmission(static_cast<uint8_t>(NODE_GPS_ADDR));
  Wire.write(0xFFU);
  const uint8_t txStatus = Wire.endTransmission(false);
  if (txStatus != 0U) {
    const uint32_t nowMs = millis();
    if ((nowMs - g_gpsState.lastI2cErrorLogMs) >= kGpsI2cErrorLogIntervalMs) {
      LOG_WARN("GPS I2C request failed status=%u", static_cast<unsigned>(txStatus));
      g_gpsState.lastI2cErrorLogMs = nowMs;
    }
  } else {
    (void)Wire.requestFrom(static_cast<uint8_t>(NODE_GPS_ADDR), kGpsReadChunkBytes);
    for (uint8_t i = 0U; (i < kGpsReadChunkBytes) && (Wire.available() > 0); i++) {
      (void)g_gpsState.parser.encode(static_cast<char>(Wire.read()));
    }
  }

  if (!g_gpsState.loggedNoDataWarning &&
      (millis() > kGpsNoDataWarnAfterMs) &&
      (g_gpsState.parser.charsProcessed() < 10UL)) {
    LOG_WARN("No GPS NMEA data detected yet");
    g_gpsState.loggedNoDataWarning = true;
  }
}

void update_network_time_from_gps(void) {
  if (!g_gpsState.parser.time.isValid()) {
    g_gpsState.hasValidTime = false;
    return;
  }

  const uint32_t currentTimeOfDayCs = gpsTimeToCentiseconds(g_gpsState.parser.time);
  g_gpsState.timeOfDayMs = currentTimeOfDayCs * 10U;
  if (!g_gpsState.hasValidTime) {
    g_gpsState.hasValidTime = true;
    LOG_INFO(
        "GPS time lock acquired %02u:%02u:%02u.%02u",
        static_cast<unsigned>(g_gpsState.parser.time.hour()),
        static_cast<unsigned>(g_gpsState.parser.time.minute()),
        static_cast<unsigned>(g_gpsState.parser.time.second()),
        static_cast<unsigned>(g_gpsState.parser.time.centisecond()));
  }
}

void update_network_coords_from_gps(void) {
  if (!g_gpsState.parser.location.isValid()) {
    g_gpsState.hasValidLocation = false;
    return;
  }

  const RawDegrees rawLng = g_gpsState.parser.location.rawLng();
  const RawDegrees rawLat = g_gpsState.parser.location.rawLat();

  g_gpsState.longitudeNano = rawDegreesToNano(rawLng);
  g_gpsState.latitudeNano = rawDegreesToNano(rawLat);
  g_gpsState.hasValidLocation = true;
}

uint32_t get_network_now_ms(void) {
  if (!g_gpsState.hasValidTime) {
    return 0U;
  }

  return g_gpsState.timeOfDayMs;
}

void service_can_tx(uint32_t networkNowMs) {
  // Add periodic transmit-side behavior in this service pattern.
  const uint32_t scheduleNowMs = millis();

  // TX SECTION 1: node heartbeat.
  if ((scheduleNowMs - g_schedulerState.lastHeartbeatTxMs) >= AIM_HEARTBEAT_TX_INTERVAL_DEFAULT_MS) {
    g_schedulerState.lastHeartbeatTxMs = scheduleNowMs;
    const uint32_t payload = static_cast<uint32_t>(g_schedulerState.value);
    const bool heartbeatSent = g_aim.sendPkt32(networkNowMs, payload, AIM_DEST_BROADCAST, AIM_TYP_HEARTBEAT);
    if (!heartbeatSent) {
      LOG_ERROR("Heartbeat TX failed");
    } else {
      LOG_DEBUG("Heartbeat TX ok");
    }
  }

  // TX SECTION 2: strict GPS time-of-day sync in 64-bit payload.
  if ((scheduleNowMs - g_schedulerState.lastTimeSyncTxMs) >= kTimeSyncTxIntervalMs) {
    g_schedulerState.lastTimeSyncTxMs = scheduleNowMs;

    if (!g_gpsState.hasValidTime) {
      if (!g_gpsState.loggedNoGpsTimeWarning) {
        LOG_WARN("Time sync TX paused until GPS time is valid");
        g_gpsState.loggedNoGpsTimeWarning = true;
      }
    } else {
      g_gpsState.loggedNoGpsTimeWarning = false;
      const bool syncSent = g_aim.sendPkt64(static_cast<uint64_t>(g_gpsState.timeOfDayMs), AIM_DEST_BROADCAST, AIM_TYP_TIME);
      if (!syncSent) {
        LOG_ERROR("Time sync TX failed");
      }
    }
  }

  // TX SECTION 3: COMMs GPS coordinates, long then lat, each as signed nano-degrees in 64-bit payload.
  if ((scheduleNowMs - g_schedulerState.lastGpsCoordTxMs) >= kGpsCoordTxIntervalMs) {
    g_schedulerState.lastGpsCoordTxMs = scheduleNowMs;

    if (!g_gpsState.hasValidLocation) {
      if (!g_gpsState.loggedNoGpsLocationWarning) {
        LOG_WARN("GPS coordinate TX paused until location is valid");
        g_gpsState.loggedNoGpsLocationWarning = true;
      }
    } else {
      g_gpsState.loggedNoGpsLocationWarning = false;

      const bool longSent = g_aim.sendPkt64(static_cast<uint64_t>(g_gpsState.longitudeNano), AIM_DEST_COMMS, AIM_TYP_GPS_LONG);
      const bool latSent = g_aim.sendPkt64(static_cast<uint64_t>(g_gpsState.latitudeNano), AIM_DEST_COMMS, AIM_TYP_GPS_LAT);
      if (!longSent || !latSent) {
        LOG_ERROR("GPS coord TX failed (long=%u lat=%u)",
                  static_cast<unsigned>(longSent ? 1U : 0U),
                  static_cast<unsigned>(latSent ? 1U : 0U));
      }
    }
  }
}

void run_state_machine(uint32_t networkNowMs) {
  AIM_ASSERT(g_schedulerState.value <= FAULT);  // precondition: corrupted state -> reset

  switch (g_schedulerState.value) {
    case OPERATIONAL:
#ifndef FLIGHT_BUILD
      if (consoleCheckEntry() == CONSOLE_ACTION_ENTER) {
        g_schedulerState.value = DEBUG_CONSOLE;
      }
#endif
      break;

#ifndef FLIGHT_BUILD
    case DEBUG_CONSOLE: {
      const ConsoleAction act = consoleService(static_cast<uint8_t>(g_schedulerState.value), networkNowMs);
      if (act == CONSOLE_ACTION_EXIT) {
        g_schedulerState.value = OPERATIONAL;
      } else if (act == CONSOLE_ACTION_FLASH_DUMP) {
        g_schedulerState.value = FLASH_DUMP;
      }
      break;
    }

    case FLASH_DUMP:
      if (consoleServiceFlashDump() == CONSOLE_ACTION_DUMP_DONE) {
        g_schedulerState.value = DEBUG_CONSOLE;
      }
      break;
#endif

    case SAFE_MODE:
    case LOW_POWER:
    case FAULT:
      break;

    default:
      AIM_ASSERT(false);
      break;
  }

  board_update();
  service_can_tx(networkNowMs);
}

void board_update(void) {
  // BOARD EXTENSION POINT: add recurring board logic here.

  poll_gps_i2c();
  update_network_time_from_gps();
  update_network_coords_from_gps();
}

void setup(void) {
  AIM_ASSERT(NODE_ORIGIN <= AIM_ORG_ADDR_MAX);
  g_serial.begin(NODE_SERIAL_BAUD);
  g_logger = &g_log;
  LOG_INFO("Boot node origin=%u", static_cast<unsigned>(NODE_ORIGIN));
  IWatchdog.begin(kWatchdogTimeoutUs);
  LOG_INFO("Watchdog ready");
  g_aim.begin();
#ifndef FLIGHT_BUILD
  consoleInit(g_serial, g_aim, g_log, g_flashTable);
#endif

  SPI.setSCLK(NODE_FLASH_SCK_PIN);
  SPI.setMISO(NODE_FLASH_MISO_PIN);
  SPI.setMOSI(NODE_FLASH_MOSI_PIN);
  SPI.begin();
  if (SerialFlash.begin(NODE_FLASH_CS_PIN)) {
    g_flashTable.init(&g_serial);
  }
  if (g_flashTable.isReady()) {
    LOG_INFO("Flash ready");
  } else {
    LOG_WARN("Flash init failed");
  }

  Wire.setSCL(NODE_GPS_SCL_PIN);
  Wire.setSDA(NODE_GPS_SDA_PIN);
  Wire.begin();
  LOG_INFO("GPS I2C ready addr=0x%02X", static_cast<unsigned>(NODE_GPS_ADDR));
#ifndef FLIGHT_BUILD
  g_serial.println("Console ready. d=enter debug");
#endif
  g_schedulerState.lastHeartbeatTxMs = millis();
  g_schedulerState.lastTimeSyncTxMs = millis();
  g_schedulerState.lastGpsCoordTxMs = millis();
  g_schedulerState.value = OPERATIONAL;
}

void loop(void) {
  const uint32_t networkNowMs = get_network_now_ms();

  // Main scheduler order: RX, state machine, watchdog.
  service_can_rx(networkNowMs);
  run_state_machine(networkNowMs);

  IWatchdog.reload();
}