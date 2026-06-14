#include "node.h"

#include <logger.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <aim_job.h>

static constexpr uint8_t kGpsReadChunkBytes = 64U;
static constexpr uint32_t kGpsNoDataWarnAfterMs = 5000U;
static constexpr int64_t kGpsDegreesToNanoScale = 1000000000LL;

struct GpsState {
  TinyGPSPlus parser;
  bool hasValidTime = false;
  bool hasValidLocation = false;
  uint32_t timeOfDayMs = 0U;
  int64_t longitudeNano = 0LL;
  int64_t latitudeNano = 0LL;
  aim::Job i2cErrorLogJob{5000U};
  aim::Job coordTxJob{1000U};
  bool loggedNoDataWarning = false;
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
  (void)nowMs;
  Wire.setSCL(pins::kGpsScl);
  Wire.setSDA(pins::kGpsSda);
  Wire.begin();
  LOG_INFO("GPS I2C ready addr=0x%02X", static_cast<unsigned>(pins::kGpsAddr));
}

void nodeServiceCanTx(uint32_t schedulerNowMs, AimNetwork& aim) {
  // GPS does not master the clock — it only consumes TimeSync (handled inside
  // AimNetwork::receive). The only periodic TX is the GPS position fix.
  if (!g_gpsState.coordTxJob.due(schedulerNowMs)) {
    return;
  }

  if (!g_gpsState.hasValidLocation) {
    if (!g_gpsState.loggedNoGpsLocationWarning) {
      LOG_WARN("GPS coordinate TX paused until location is valid");
      g_gpsState.loggedNoGpsLocationWarning = true;
    }
    return;
  }
  g_gpsState.loggedNoGpsLocationWarning = false;

  // Catalog scaling: GpsLat/GpsLon are degrees x10^7 (i32). Parser values are
  // nano-degrees (x10^9), so divide by 100. Max |180e7| < INT32_MAX.
  aim::Msg lon = {};
  lon.cls = aim::Class::Sensor;
  lon.subject = aim::subject::GpsLon;
  lon.setSensorValue(static_cast<int32_t>(g_gpsState.longitudeNano / 100LL));
  const bool lonSent = aim.send(lon);

  aim::Msg lat = {};
  lat.cls = aim::Class::Sensor;
  lat.subject = aim::subject::GpsLat;
  lat.setSensorValue(static_cast<int32_t>(g_gpsState.latitudeNano / 100LL));
  const bool latSent = aim.send(lat);

  if (!lonSent || !latSent) {
    LOG_ERROR("GPS coord TX failed (lon=%u lat=%u)",
              static_cast<unsigned>(lonSent ? 1U : 0U),
              static_cast<unsigned>(latSent ? 1U : 0U));
  }
}

void nodeUpdate(uint32_t schedulerNowMs) {
  // Retrieve and parse GPS NMEA sentences over I2C, and update network time base.

  // Pull GPS data over I2C
  // The GPS module exposes a single data register at 0xFF.
  Wire.beginTransmission(static_cast<uint8_t>(pins::kGpsAddr));
  Wire.write(0xFFU);
  const uint8_t txStatus = Wire.endTransmission(false);
  if (txStatus != 0U) {
    if (g_gpsState.i2cErrorLogJob.due(schedulerNowMs)) {
      LOG_WARN("GPS I2C request failed status=%u", static_cast<unsigned>(txStatus));
    }
  } else {
    (void)Wire.requestFrom(static_cast<uint8_t>(pins::kGpsAddr), kGpsReadChunkBytes);
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
void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  // TODO: handle cross-node events (e.g. power state) once subjects are defined.
  (void)m;
  (void)nowMs;
}
