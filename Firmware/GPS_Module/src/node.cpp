// node.cpp - GPS Module Application Logic
// References:
//   - u-blox SAM-M10Q Data Sheet & Integration Manual (UBX-22002218)
//   - AIM Network v0.7.0 Protocol Spec & Migration Plan (Decision D12/D14)
// MANDATORY PRE-FLIGHT CHECK: Make sure the SAM-M10Q GPS receiver is configured via u-center (u-blox UBX-CFG) for:
//   1. 10 Hz navigation update rate (matches kGpsActivePeriodMs = 100 ms)
//   2. High-G / Airborne dynamic model (Airborne <4g / Dynamic Model 8)

#include "node.h"

#include <Adafruit_NeoPixel.h>
#include <logger.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <aim_job.h>
#include <aim_flight_recorder.h>

static constexpr uint8_t  kGpsReadChunkBytes    = 64U;
static constexpr uint32_t kGpsNoDataWarnAfterMs = 5000U;

// Current boot default: 1 Hz (kGpsIdlePeriodMs).
static constexpr uint32_t kGpsActivePeriodMs = 100U;   // 10 Hz active flight (SAM-M10Q hardware 10 Hz rate)
static constexpr uint32_t kGpsIdlePeriodMs   = 1000U;  // 1 Hz pad / landed / low power
static constexpr uint32_t kSatTxPeriodMs     = 5000U;  // 0.2 Hz (once every 5 seconds)

static TinyGPSPlus s_parser;
static bool s_hasValidTime = false;
static bool s_hasValidLocation = false;
static char s_readableTimeStr[20] = "N/A";
static int32_t s_lat1e7 = 0;
static int32_t s_lon1e7 = 0;
static int32_t s_altMeters = 0;

static bool s_gpsI2cFailed = false;
static bool s_gpsNoData = false;
static bool s_lowPower = false;
static bool s_loggedNoDataWarn = false;
static bool s_loggedNoLocWarn = false;

static aim::Job s_i2cErrorLogJob{5000U, 0U};
static aim::Job s_coordTxJob{kGpsIdlePeriodMs, 0U};
static aim::Job s_flashLogJob{kGpsIdlePeriodMs, 0U};
static aim::Job s_satTxJob{kSatTxPeriodMs, 0U};

static Adafruit_NeoPixel s_rgbLeds(1U, pins::kRgbData, NEO_GRB + NEO_KHZ800);

static int32_t rawDegreesTo1e7(const RawDegrees& raw) {
  int32_t deg1e7 = static_cast<int32_t>(raw.deg) * 10000000L + static_cast<int32_t>(raw.billionths / 100U);
  return raw.negative ? -deg1e7 : deg1e7;
}

static void updateLed(aim::NodeState state) {
  uint8_t color = 3; // blue default (searching)
  if (state == aim::NodeState::Fault) { color = 2; } // red
  else if (state == aim::NodeState::Nominal && s_hasValidLocation) { color = 1; } // green

  static uint8_t s_lastColor = 0xFF;
  if (color == s_lastColor) return;
  s_lastColor = color;

  uint8_t r = 0, g = 0, b = 0;
  switch (color) { case 1: g = 255; break; case 2: r = 255; break; default: b = 255; break; }
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(r, g, b));
  s_rgbLeds.show();
}

void nodeInit() {
  s_rgbLeds.begin();
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(0, 0, 0));
  s_rgbLeds.show();

  Wire.setSCL(pins::kGpsScl);
  Wire.setSDA(pins::kGpsSda);
  Wire.begin();
  Wire.setClock(400000);  // Fast-Mode I2C clock speed (400 kHz)
  Wire.setTimeout(10U);   // 10 ms bus timeout protection

  LOG_INFO("GPS I2C ready addr=0x%02X (SAM-M10Q 10 Hz High-G mode expected)", static_cast<unsigned>(pins::kGpsAddr));
}

void nodeUpdate(uint32_t nowMs) {
  updateLed(nodeCurrentState());

  Wire.beginTransmission(static_cast<uint8_t>(pins::kGpsAddr));
  Wire.write(0xFFU);
  const uint8_t txStatus = Wire.endTransmission(false);
  if (txStatus != 0U) {
    s_gpsI2cFailed = true;
    if (s_i2cErrorLogJob.due(nowMs)) {
      LOG_WARN("GPS I2C request failed status=%u", static_cast<unsigned>(txStatus));
    }
  } else {
    s_gpsI2cFailed = false;
    (void)Wire.requestFrom(static_cast<uint8_t>(pins::kGpsAddr), kGpsReadChunkBytes);
    for (uint8_t i = 0U; (i < kGpsReadChunkBytes) && (Wire.available() > 0); i++) {
      uint8_t b = static_cast<uint8_t>(Wire.read());
      // SAM-M10Q returns 0xFF when I2C stream FIFO is empty — do not feed 0xFF pad bytes to NMEA parser.
      if (b != 0xFFU) {
        (void)s_parser.encode(static_cast<char>(b));
      }
    }
  }

  if (nowMs > kGpsNoDataWarnAfterMs && s_parser.charsProcessed() < 10UL) {
    s_gpsNoData = true;
    if (!s_loggedNoDataWarn) {
      LOG_WARN("No GPS NMEA data detected yet");
      s_loggedNoDataWarn = true;
    }
  } else {
    s_gpsNoData = false;
  }

  if (!s_parser.time.isValid()) {
    s_hasValidTime = false;
  } else {
    if (!s_hasValidTime) {
      s_hasValidTime = true;
      LOG_INFO("GPS time lock acquired");
    }
    if (s_parser.time.isUpdated()) {
      snprintf(s_readableTimeStr, sizeof(s_readableTimeStr), "%02u:%02u:%02u.%02u",
               static_cast<unsigned>(s_parser.time.hour()),
               static_cast<unsigned>(s_parser.time.minute()),
               static_cast<unsigned>(s_parser.time.second()),
               static_cast<unsigned>(s_parser.time.centisecond()));
    }
  }

  if (!s_parser.location.isValid()) {
    s_hasValidLocation = false;
  } else {
    s_lon1e7 = rawDegreesTo1e7(s_parser.location.rawLng());
    s_lat1e7 = rawDegreesTo1e7(s_parser.location.rawLat());
    s_hasValidLocation = true;
  }

  if (s_parser.altitude.isValid()) {
    s_altMeters = static_cast<int32_t>(s_parser.altitude.meters()); // Whole meters MSL
  }
}

void nodeServiceLog(uint32_t nowMs, AimFlightRecorder& recorder) {
  if (!recorder.isLogging()) return;
  if (!s_flashLogJob.due(nowMs)) return;
  if (!s_hasValidLocation || !s_parser.location.isUpdated()) return;

  // FLASH WRITING DISABLED FOR BENCH TESTING.
  // ROADMAP (D14): AimFlightRecorder will accept an AimColumnDef array (name, type, scaleFactor)
  // to internally handle optimal bit-packing & self-describing serial dumps across all nodes.
  (void)nowMs;
  (void)recorder;

  /*
  uint32_t rowData[4] = {
    nowMs,
    AimFlightRecorder::unsignify(s_lon1e7),
    AimFlightRecorder::unsignify(s_lat1e7),
    AimFlightRecorder::unsignify(s_altMeters)
  };

  (void)recorder.writeRow(rowData);
  */
}

void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim) {
  // 1. High-frequency position broadcast (10 Hz in flight / 1 Hz on pad)
  if (s_coordTxJob.due(nowMs)) {
    if (!s_hasValidLocation) {
      if (!s_loggedNoLocWarn) {
        LOG_WARN("GPS coordinate TX paused until location is valid");
        s_loggedNoLocWarn = true;
      }
    } else {
      s_loggedNoLocWarn = false;
      aim::Msg gpsPos = {};
      gpsPos.cls = aim::Class::Sensor;
      gpsPos.subject = aim::subject::GpsPosition;
      gpsPos.setGpsPosition(s_lon1e7, s_lat1e7);
      if (!aim.send(gpsPos)) {
        LOG_ERROR("GPS coord TX failed");
      }
    }
  }

  // 2. Low-frequency satellite count broadcast (once every 5 seconds)
  if (s_satTxJob.due(nowMs) && s_parser.satellites.isValid()) {
    aim::Msg satsMsg = {};
    satsMsg.cls = aim::Class::Sensor;
    satsMsg.subject = aim::subject::GpsNumSats;
    satsMsg.setSensorValue(static_cast<int32_t>(s_parser.satellites.value()));
    (void)aim.send(satsMsg);
  }
}

void nodeSetTelemetryMode(bool active, AimNetwork& aim) {
  const uint32_t newPeriod = active ? kGpsActivePeriodMs : kGpsIdlePeriodMs;
  if (s_coordTxJob.periodMs != newPeriod) {
    s_coordTxJob.periodMs  = newPeriod;
    s_flashLogJob.periodMs = newPeriod;

    aim::Msg modeEvt = {};
    modeEvt.cls     = aim::Class::Event;
    modeEvt.subject = aim::subject::TelemetryMode;
    modeEvt.b[0]    = active ? 1U : 0U;
    (void)aim.send(modeEvt);

    LOG_INFO("GPS rate change event published (active=%d, period=%lu ms)",
             static_cast<int>(active), newPeriod);
  }
}

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  (void)nowMs;
  if (m.cls == aim::Class::Event) {
    if (m.subject == aim::subject::LaunchDetect || m.subject == aim::subject::TelemetryMode) {
      const bool isActive = (m.subject == aim::subject::LaunchDetect) || (m.b[0] == 1U);
      const uint32_t newPeriod = isActive ? kGpsActivePeriodMs : kGpsIdlePeriodMs;
      if (s_coordTxJob.periodMs != newPeriod) {
        s_coordTxJob.periodMs  = newPeriod;
        s_flashLogJob.periodMs = newPeriod;
        LOG_INFO("TelemetryMode event (subj=0x%02X active=%d): GPS rates -> %lu ms (10 Hz)",
                 static_cast<unsigned>(m.subject), static_cast<int>(isActive), newPeriod);
      }
    } else if (m.subject == aim::subject::LowPower) {
      s_lowPower = (m.b[0] == 1U);
      const uint32_t newPeriod = s_lowPower ? kGpsIdlePeriodMs : kGpsActivePeriodMs;
      if (s_coordTxJob.periodMs != newPeriod) {
        s_coordTxJob.periodMs  = newPeriod;
        s_flashLogJob.periodMs = newPeriod;
        LOG_INFO("GPS low power state updated: %d (period=%lu ms)", s_lowPower, newPeriod);
      }
    }
  }
}

aim::NodeState nodeCurrentState() {
  return s_gpsI2cFailed ? aim::NodeState::Fault : aim::NodeState::Nominal;
}

uint16_t nodeErrorBits() {
  uint16_t bits = 0U;
  if (s_gpsI2cFailed) bits |= (1U << 0);
  if (s_gpsNoData)    bits |= (1U << 1);
  return bits;
}

#ifndef FLIGHT_BUILD
static void hookGpsSnapshot(Stream& out) {
  out.print("gps timeValid(parser/state)=");
  out.print(s_parser.time.isValid() ? 1 : 0);
  out.print("/");
  out.println(s_hasValidTime ? 1 : 0);

  out.print("gps locValid(parser/state)=");
  out.print(s_parser.location.isValid() ? 1 : 0);
  out.print("/");
  out.println(s_hasValidLocation ? 1 : 0);

  out.print("gps sats(valid/count)=");
  out.print(s_parser.satellites.isValid() ? 1 : 0);
  out.print("/");
  out.println(static_cast<unsigned long>(s_parser.satellites.value()));

  out.print("UTC Time=");
  out.println(s_readableTimeStr);

  out.print("lon1e7=");
  out.println(static_cast<long>(s_lon1e7));
  out.print("lat1e7=");
  out.println(static_cast<long>(s_lat1e7));
  out.print("altMeters=");
  out.println(static_cast<long>(s_altMeters));

  out.print("txPeriodMs=");
  out.println(static_cast<unsigned long>(s_coordTxJob.periodMs));
  out.print("logPeriodMs=");
  out.println(static_cast<unsigned long>(s_flashLogJob.periodMs));
}

static void hookGpsParserStats(Stream& out) {
  out.print("chars=");
  out.println(static_cast<unsigned long>(s_parser.charsProcessed()));
  out.print("sentencesWithFix=");
  out.println(static_cast<unsigned long>(s_parser.sentencesWithFix()));
  out.print("checksum pass/fail=");
  out.print(static_cast<unsigned long>(s_parser.passedChecksum()));
  out.print("/");
  out.println(static_cast<unsigned long>(s_parser.failedChecksum()));
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
