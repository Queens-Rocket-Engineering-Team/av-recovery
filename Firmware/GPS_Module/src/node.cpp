#include "node.h"

#include <Adafruit_NeoPixel.h>
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
  aim::Job i2cErrorLogJob{5000U, 0U};
  aim::Job coordTxJob{1000U, 0U};
  bool loggedNoDataWarning = false;
  bool loggedNoGpsLocationWarning = false;
};

static GpsState s_gpsState = {};
static bool s_gpsI2cFailed = false;
static bool s_gpsNoData = false;
static bool s_lowPower = false;

static Adafruit_NeoPixel s_rgbLeds(1U, pins::kRgbData, NEO_GRB + NEO_KHZ800);

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

static void updateLed(aim::NodeState state) {
  uint8_t color = 3; // blue default
  if (state == aim::NodeState::Fault) { color = 2; }
  else if (state == aim::NodeState::Nominal && s_gpsState.hasValidLocation) { color = 1; }

  static uint8_t s_lastColor = 0xFF;
  if (color == s_lastColor) return;
  s_lastColor = color;

  uint8_t r = 0, g = 0, b = 0;
  switch (color) { case 1: g = 255; break; case 2: r = 255; break; default: b = 255; break; }
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(r, g, b));
  s_rgbLeds.show();
}

bool nodeGetGpsDebugSnapshot(GpsDebugSnapshot* out) {
  if (out == nullptr) {
    return false;
  }

  out->parserTimeValid = s_gpsState.parser.time.isValid();
  out->parserLocationValid = s_gpsState.parser.location.isValid();
  out->parserSatellitesValid = s_gpsState.parser.satellites.isValid();
  out->hasValidTime = s_gpsState.hasValidTime;
  out->hasValidLocation = s_gpsState.hasValidLocation;
  out->timeOfDayMs = s_gpsState.timeOfDayMs;
  out->longitudeNano = s_gpsState.longitudeNano;
  out->latitudeNano = s_gpsState.latitudeNano;
  out->satellites = s_gpsState.parser.satellites.value();
  out->charsProcessed = s_gpsState.parser.charsProcessed();
  out->sentencesWithFix = s_gpsState.parser.sentencesWithFix();
  out->failedChecksum = s_gpsState.parser.failedChecksum();
  out->passedChecksum = s_gpsState.parser.passedChecksum();
  return true;
}

void nodeInit() {
  s_rgbLeds.begin();
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(0, 0, 0));
  s_rgbLeds.show();

  Wire.setSCL(pins::kGpsScl);
  Wire.setSDA(pins::kGpsSda);
  Wire.begin();
  LOG_INFO("GPS I2C ready addr=0x%02X", static_cast<unsigned>(pins::kGpsAddr));
}

void nodeUpdate(uint32_t nowMs) {
  updateLed(nodeCurrentState());

  Wire.beginTransmission(static_cast<uint8_t>(pins::kGpsAddr));
  Wire.write(0xFFU);
  const uint8_t txStatus = Wire.endTransmission(false);
  if (txStatus != 0U) {
    s_gpsI2cFailed = true;
    if (s_gpsState.i2cErrorLogJob.due(nowMs)) {
      LOG_WARN("GPS I2C request failed status=%u", static_cast<unsigned>(txStatus));
    }
  } else {
    s_gpsI2cFailed = false;
    (void)Wire.requestFrom(static_cast<uint8_t>(pins::kGpsAddr), kGpsReadChunkBytes);
    for (uint8_t i = 0U; (i < kGpsReadChunkBytes) && (Wire.available() > 0); i++) {
      (void)s_gpsState.parser.encode(static_cast<char>(Wire.read()));
    }
  }

  if (nowMs > kGpsNoDataWarnAfterMs && s_gpsState.parser.charsProcessed() < 10UL) {
    s_gpsNoData = true;
    if (!s_gpsState.loggedNoDataWarning) {
      LOG_WARN("No GPS NMEA data detected yet");
      s_gpsState.loggedNoDataWarning = true;
    }
  } else {
    s_gpsNoData = false;
  }

  if (!s_gpsState.parser.time.isValid()) {
    s_gpsState.hasValidTime = false;
  } else {
    const uint32_t currentTimeOfDayCs = gpsTimeToCentiseconds(s_gpsState.parser.time);
    s_gpsState.timeOfDayMs = currentTimeOfDayCs * 10U;
    if (!s_gpsState.hasValidTime) {
      s_gpsState.hasValidTime = true;
      LOG_INFO(
          "GPS time lock acquired %02u:%02u:%02u.%02u",
          static_cast<unsigned>(s_gpsState.parser.time.hour()),
          static_cast<unsigned>(s_gpsState.parser.time.minute()),
          static_cast<unsigned>(s_gpsState.parser.time.second()),
          static_cast<unsigned>(s_gpsState.parser.time.centisecond()));
    }
  }

  if (!s_gpsState.parser.location.isValid()) {
    s_gpsState.hasValidLocation = false;
  } else {
    const RawDegrees rawLng = s_gpsState.parser.location.rawLng();
    const RawDegrees rawLat = s_gpsState.parser.location.rawLat();

    s_gpsState.longitudeNano = rawDegreesToNano(rawLng);
    s_gpsState.latitudeNano = rawDegreesToNano(rawLat);
    s_gpsState.hasValidLocation = true;
  }
}

void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim) {
  if (!s_gpsState.coordTxJob.due(nowMs)) {
    return;
  }

  if (!s_gpsState.hasValidLocation) {
    if (!s_gpsState.loggedNoGpsLocationWarning) {
      LOG_WARN("GPS coordinate TX paused until location is valid");
      s_gpsState.loggedNoGpsLocationWarning = true;
    }
    return;
  }
  s_gpsState.loggedNoGpsLocationWarning = false;

  aim::Msg lon = {};
  lon.cls = aim::Class::Sensor;
  lon.subject = aim::subject::GpsLon;
  lon.setSensorValue(static_cast<int32_t>(s_gpsState.longitudeNano / 100LL));
  const bool lonSent = aim.send(lon);

  aim::Msg lat = {};
  lat.cls = aim::Class::Sensor;
  lat.subject = aim::subject::GpsLat;
  lat.setSensorValue(static_cast<int32_t>(s_gpsState.latitudeNano / 100LL));
  const bool latSent = aim.send(lat);

  if (!lonSent || !latSent) {
    LOG_ERROR("GPS coord TX failed (lon=%u lat=%u)",
              static_cast<unsigned>(lonSent ? 1U : 0U),
              static_cast<unsigned>(latSent ? 1U : 0U));
  }
}

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  (void)nowMs;
  if (m.cls == aim::Class::Event && m.subject == aim::subject::LowPower) {
    s_lowPower = (m.b[0] == 1U);
    LOG_INFO("GPS low power state updated: %d", s_lowPower);
  }
}

aim::NodeState nodeCurrentState() {
  if (s_gpsI2cFailed) {
    return aim::NodeState::Fault;
  }
  return aim::NodeState::Nominal;
}

uint16_t nodeErrorBits() {
  uint16_t bits = 0U;
  if (s_gpsI2cFailed) {
    bits |= (1U << 0);
  }
  if (s_gpsNoData) {
    bits |= (1U << 1);
  }
  return bits;
}

#ifndef FLIGHT_BUILD
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

static const AimConsoleHook s_consoleHooks[] = {
  {'g', "gps snapshot", hookGpsSnapshot},
  {'p', "gps parser stats", hookGpsParserStats},
};

const AimConsoleHook* nodeConsoleHooks(uint8_t& count) {
  count = sizeof(s_consoleHooks) / sizeof(s_consoleHooks[0]);
  return s_consoleHooks;
}
#endif
