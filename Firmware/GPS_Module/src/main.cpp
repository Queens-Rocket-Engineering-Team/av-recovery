#include "node.h"

#include <IWatchdog.h>
#include <logger.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPS++.h>

static constexpr uint32_t kHeartbeatTxIntervalMs = AIM_HEARTBEAT_TX_INTERVAL_DEFAULT_MS;
static constexpr uint32_t kTimeSyncTxIntervalMs = 250U;
static constexpr uint32_t kGpsCoordTxIntervalMs = 1000U;
static constexpr uint32_t kWatchdogTimeoutUs = 2000000U;
static constexpr uint8_t kMaxRxFramesPerLoop = 8U;
static constexpr uint8_t kGpsReadChunkBytes = 64U;
static constexpr uint8_t kGpsI2cDataRegisterRequest = 0xFFU;
static constexpr uint32_t kGpsNoDataWarnAfterMs = 5000U;
static constexpr uint32_t kGpsI2cErrorLogIntervalMs = 5000U;
static constexpr uint32_t kMillisecondsPerCentisecond = 10U;
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
static NodeSchedulerState g_schedulerState = {};
static GpsState g_gpsState = {};

void service_can_rx(void) {
  // Handle incoming bus messages and custom packet branches here.
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
  Wire.beginTransmission(static_cast<uint8_t>(NODE_GPS_ADDR));
  Wire.write(kGpsI2cDataRegisterRequest);
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
  g_gpsState.timeOfDayMs = currentTimeOfDayCs * kMillisecondsPerCentisecond;
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

void service_tx(uint32_t networkNowMs) {
  // Add periodic transmit-side behavior in this service pattern.
  const uint32_t scheduleNowMs = millis();

  // TX SECTION 1: node heartbeat.
  if ((scheduleNowMs - g_schedulerState.lastHeartbeatTxMs) >= kHeartbeatTxIntervalMs) {
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
  if (g_schedulerState.value > FAULT) {
    g_schedulerState.value = FAULT;
  }

  AIM_ASSERT(g_schedulerState.value <= FAULT);
  if (g_schedulerState.value == INIT) {
    board_init();
    const uint32_t scheduleNowMs = millis();
    g_schedulerState.lastHeartbeatTxMs = scheduleNowMs;
    g_schedulerState.lastTimeSyncTxMs = scheduleNowMs;
    g_schedulerState.lastGpsCoordTxMs = scheduleNowMs;
    g_schedulerState.value = OPERATIONAL;
    LOG_INFO("State transition INIT -> OPERATIONAL");
    return;
  }

  board_update(g_schedulerState.value);
  service_tx(networkNowMs);
}

void board_init(void) {
  // BOARD EXTENSION POINT: add one-time board setup here.
  AIM_ASSERT(NODE_ORIGIN <= AIM_ORG_ADDR_MAX);

  Wire.setSCL(NODE_GPS_SCL_PIN);
  Wire.setSDA(NODE_GPS_SDA_PIN);
  Wire.begin();
  LOG_INFO("GPS I2C ready addr=0x%02X", static_cast<unsigned>(NODE_GPS_ADDR));
}

void board_update(NodeState state) {
  // BOARD EXTENSION POINT: add recurring board logic here.
  AIM_ASSERT(state <= FAULT);

  poll_gps_i2c();
  update_network_time_from_gps();
  update_network_coords_from_gps();

  (void)state;
}

void setup(void) {
  AIM_ASSERT(NODE_ORIGIN <= AIM_ORG_ADDR_MAX);
  g_serial.begin(NODE_SERIAL_BAUD);
  g_logger = &g_log;
  LOG_INFO("Boot node origin=%u", static_cast<unsigned>(NODE_ORIGIN));
  IWatchdog.begin(kWatchdogTimeoutUs);
  LOG_INFO("Watchdog ready");
  g_aim.begin();
  g_schedulerState.value = INIT;
}

void loop(void) {
  AIM_ASSERT(g_schedulerState.value <= FAULT);
  const uint32_t networkNowMs = get_network_now_ms();

  // Main scheduler order: RX, state machine, watchdog.
  service_can_rx();
  run_state_machine(networkNowMs);

  IWatchdog.reload();
}