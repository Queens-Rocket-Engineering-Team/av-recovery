#include "node.h"

#include <logger.h>
#include <Wire.h>
#include <TinyGPS++.h>

static constexpr uint32_t kTimeSyncTxIntervalMs = 250U;
static constexpr uint32_t kGpsCoordTxIntervalMs = 1000U;
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
  uint32_t lastTimeSyncTxMs = 0U;
  uint32_t lastGpsCoordTxMs = 0U;
  bool loggedNoDataWarning = false;
  bool loggedNoGpsTimeWarning = false;
  bool loggedNoGpsLocationWarning = false;
};

static GpsState g_gpsState = {};

static uint32_t gpsTimeToCentiseconds(TinyGPSTime& gpsTime) {
  const uint32_t hh = static_cast<uint32_t>(gpsTime.hour());
  const uint32_t mm = static_cast<uint32_t>(gpsTime.minute());
  const uint32_t ss = static_cast<uint32_t>(gpsTime.second());
  const uint32_t cs = static_cast<uint32_t>(gpsTime.centisecond());

  return ((((hh * 60U) + mm) * 60U) + ss) * 100U + cs;
}

static int64_t rawDegreesToNano(const RawDegrees& raw) {
  int64_t nano = static_cast<int64_t>(raw.deg) * kGpsDegreesToNanoScale;
  nano += static_cast<int64_t>(raw.billionths);
  if (raw.negative) {
    nano = -nano;
  }

  return nano;
}

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

void nodeInit(uint32_t nowMs) {
  Wire.setSCL(NODE_GPS_SCL_PIN);
  Wire.setSDA(NODE_GPS_SDA_PIN);
  Wire.begin();
  LOG_INFO("GPS I2C ready addr=0x%02X", static_cast<unsigned>(NODE_GPS_ADDR));

  g_gpsState.lastTimeSyncTxMs = nowMs;
  g_gpsState.lastGpsCoordTxMs = nowMs;
}

uint32_t nodeGetNetworkNowMs(uint32_t schedulerNowMs) {
  if (g_gpsState.hasValidTime) {
    return g_gpsState.timeOfDayMs;
  }

  return schedulerNowMs;
}

void nodeServiceCanTx(uint32_t schedulerNowMs, uint32_t networkNowMs, AimNetwork& aim) {
  (void)networkNowMs;

  // TX SECTION 2: strict GPS time-of-day sync in 64-bit payload.
  if ((schedulerNowMs - g_gpsState.lastTimeSyncTxMs) >= kTimeSyncTxIntervalMs) {
    g_gpsState.lastTimeSyncTxMs = schedulerNowMs;

    if (!g_gpsState.hasValidTime) {
      if (!g_gpsState.loggedNoGpsTimeWarning) {
        LOG_WARN("Time sync TX paused until GPS time is valid");
        g_gpsState.loggedNoGpsTimeWarning = true;
      }
    } else {
      g_gpsState.loggedNoGpsTimeWarning = false;
      const bool syncSent = aim.sendPkt64(static_cast<uint64_t>(g_gpsState.timeOfDayMs), AIM_DEST_BROADCAST, AIM_TYP_TIME);
      if (!syncSent) {
        LOG_ERROR("Time sync TX failed");
      }
    }
  }

  // TX SECTION 3: COMMs GPS coordinates, long then lat, each as signed nano-degrees in 64-bit payload.
  if ((schedulerNowMs - g_gpsState.lastGpsCoordTxMs) >= kGpsCoordTxIntervalMs) {
    g_gpsState.lastGpsCoordTxMs = schedulerNowMs;

    if (!g_gpsState.hasValidLocation) {
      if (!g_gpsState.loggedNoGpsLocationWarning) {
        LOG_WARN("GPS coordinate TX paused until location is valid");
        g_gpsState.loggedNoGpsLocationWarning = true;
      }
    } else {
      g_gpsState.loggedNoGpsLocationWarning = false;

      const bool longSent = aim.sendPkt64(static_cast<uint64_t>(g_gpsState.longitudeNano), AIM_DEST_COMMS, AIM_TYP_GPS_LONG);
      const bool latSent = aim.sendPkt64(static_cast<uint64_t>(g_gpsState.latitudeNano), AIM_DEST_COMMS, AIM_TYP_GPS_LAT);
      if (!longSent || !latSent) {
        LOG_ERROR("GPS coord TX failed (long=%u lat=%u)",
                  static_cast<unsigned>(longSent ? 1U : 0U),
                  static_cast<unsigned>(latSent ? 1U : 0U));
      }
    }
  }
}

void nodeUpdate(uint32_t schedulerNowMs) {
  // Retrieve and parse GPS NMEA sentences over I2C, and update network time base.

  // Pull GPS data over I2C
  // The GPS module exposes a single data register at 0xFF.
  Wire.beginTransmission(static_cast<uint8_t>(NODE_GPS_ADDR));
  Wire.write(0xFFU);
  const uint8_t txStatus = Wire.endTransmission(false);
  if (txStatus != 0U) {
    if ((schedulerNowMs - g_gpsState.lastI2cErrorLogMs) >= kGpsI2cErrorLogIntervalMs) {
      LOG_WARN("GPS I2C request failed status=%u", static_cast<unsigned>(txStatus));
      g_gpsState.lastI2cErrorLogMs = schedulerNowMs;
    }
  } else {
    (void)Wire.requestFrom(static_cast<uint8_t>(NODE_GPS_ADDR), kGpsReadChunkBytes);
    for (uint8_t i = 0U; (i < kGpsReadChunkBytes) && (Wire.available() > 0); i++) {
      (void)g_gpsState.parser.encode(static_cast<char>(Wire.read()));
    }
  }

  if (!g_gpsState.loggedNoDataWarning &&
      (schedulerNowMs > kGpsNoDataWarnAfterMs) &&
      (g_gpsState.parser.charsProcessed() < 10UL)) {
    LOG_WARN("No GPS NMEA data detected yet");
    g_gpsState.loggedNoDataWarning = true;
  }

  // Update network time from GPS
  if (!g_gpsState.parser.time.isValid()) {
    g_gpsState.hasValidTime = false;
  } else {
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

  // Update GPS coordinates from GPS
  if (!g_gpsState.parser.location.isValid()) {
    g_gpsState.hasValidLocation = false;
  } else {
    const RawDegrees rawLng = g_gpsState.parser.location.rawLng();
    const RawDegrees rawLat = g_gpsState.parser.location.rawLat();

    g_gpsState.longitudeNano = rawDegreesToNano(rawLng);
    g_gpsState.latitudeNano = rawDegreesToNano(rawLat);
    g_gpsState.hasValidLocation = true;
  }
}