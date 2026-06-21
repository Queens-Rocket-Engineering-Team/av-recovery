#include "node.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_NeoPixel.h>
#include <MS5611.h>
#include <Wire.h>
#include <aim_job.h>
#include <logger.h>
#include <math.h>

static constexpr float kLapseRate = -0.0065f;
static constexpr float kBaroExponent = 0.19026627f;
static constexpr uint16_t kBaselineSamples = 50U;

enum class FlightPhase : uint8_t {
  Pad,
  Powered,
  Coast,
  Descent,
  Landed
};

static FlightPhase s_phase = FlightPhase::Pad;
static bool s_lowPower = false;

static MS5611 s_baro(pins::kMs5611Addr);
static Adafruit_MPU6050 s_imu;
static bool s_baroOk = false;
static bool s_imuOk = false;

static float s_basePressurePa = 101325.0f;
static float s_baseTempK = 288.15f;
static float s_altitudeM = 0.0f;

static aim::Job s_altitudeJob{100U, 0U};

static Adafruit_NeoPixel s_rgbLeds(1U, pins::kRgbData, NEO_GRB + NEO_KHZ800);

static float computeAltitude(float pressurePa, float basePa, float baseTempK) {
  if (basePa <= 0.0f) return 0.0f;
  return (baseTempK / kLapseRate) * (powf(pressurePa / basePa, kBaroExponent) - 1.0f);
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

static void updateLed(aim::NodeState state) {
  static aim::NodeState s_lastState = static_cast<aim::NodeState>(0xFF);
  if (state == s_lastState) return;
  s_lastState = state;
  uint8_t r = 0, g = 0, b = 0;
  switch (state) {
    case aim::NodeState::Nominal: g = 255; break;
    case aim::NodeState::Fault:   r = 255; break;
    default:                      b = 255; break;
  }
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(r, g, b));
  s_rgbLeds.show();
}

void nodeInit() {
  s_rgbLeds.begin();
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(0, 0, 0));
  s_rgbLeds.show();

  Wire.setSCL(pins::kI2cScl);
  Wire.setSDA(pins::kI2cSda);
  Wire.begin();
  Wire.setClock(100000);

  s_baroOk = s_baro.begin();
  if (s_baroOk) {
    s_baro.setOversampling(OSR_ULTRA_LOW);
    float pressSum = 0.0f;
    float tempSum = 0.0f;
    for (uint16_t i = 0U; i < kBaselineSamples; i++) {
      s_baro.read();
      pressSum += s_baro.getPressure() * 100.0f;
      tempSum += s_baro.getTemperature() + 273.15f;
    }
    s_basePressurePa = pressSum / static_cast<float>(kBaselineSamples);
    s_baseTempK = tempSum / static_cast<float>(kBaselineSamples);
    LOG_INFO("Baro baseline P=%ld Pa T=%ld cK",
             static_cast<long>(s_basePressurePa),
             static_cast<long>(s_baseTempK * 100.0f));
  } else {
    LOG_ERROR("MS5611 init failed");
  }

  s_imuOk = s_imu.begin(pins::kMpu6050Addr, &Wire);
  if (s_imuOk) {
    s_imu.setAccelerometerRange(MPU6050_RANGE_16_G);
    s_imu.setGyroRange(MPU6050_RANGE_500_DEG);
    s_imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    LOG_INFO("MPU6050 ready");
  } else {
    LOG_ERROR("MPU6050 init failed");
  }

  LOG_INFO("Altimeter init baro=%u imu=%u",
           static_cast<unsigned>(s_baroOk ? 1U : 0U),
           static_cast<unsigned>(s_imuOk ? 1U : 0U));
}

void nodeUpdate(uint32_t nowMs) {
  updateLed(nodeCurrentState());
  (void)nowMs;

  if (s_baroOk) {
    s_baro.read();
    const float pressurePa = s_baro.getPressure() * 100.0f;
    s_altitudeM = computeAltitude(pressurePa, s_basePressurePa, s_baseTempK);
  }

  if (s_imuOk) {
    sensors_event_t a, g, temp;
    s_imu.getEvent(&a, &g, &temp);
    (void)a;
    (void)g;
    (void)temp;
  }

}

void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim) {
  if (!s_baroOk) return;
  if (!s_altitudeJob.due(nowMs)) return;

  aim::Msg msg = {};
  msg.cls = aim::Class::Sensor;
  msg.subject = aim::subject::Altitude;
  msg.setSensorValue(static_cast<int32_t>(s_altitudeM * 100.0f));
  if (!aim.send(msg)) {
    LOG_WARN("Altitude TX failed");
  }
}

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  (void)nowMs;
  if (m.cls == aim::Class::Event && m.subject == aim::subject::LowPower) {
    s_lowPower = (m.b[0] == 1U);
    LOG_INFO("Altimeter low power state updated: %d", s_lowPower);
  }
}

aim::NodeState nodeCurrentState() {
  if (!s_baroOk) {
    return aim::NodeState::Fault;
  }
  return aim::NodeState::Nominal;
}

uint16_t nodeErrorBits() {
  uint16_t bits = 0U;
  if (!s_baroOk) {
    bits |= (1U << 0);
  }
  if (!s_imuOk) {
    bits |= (1U << 1);
  }
  return bits;
}

#ifndef FLIGHT_BUILD
static void hookSensors(Stream& out) {
  out.print("baro=");
  out.print(s_baroOk ? "OK" : "FAIL");
  out.print(" imu=");
  out.println(s_imuOk ? "OK" : "FAIL");

  if (s_baroOk) {
    out.print("P=");
    out.print(static_cast<long>(s_baro.getPressure() * 100.0f));
    out.print(" Pa  T=");
    out.print(static_cast<long>(s_baro.getTemperature() * 100.0f));
    out.println(" cC");
    out.print("alt=");
    out.print(static_cast<long>(s_altitudeM * 100.0f));
    out.println(" cm");
    out.print("baseline P=");
    out.print(static_cast<long>(s_basePressurePa));
    out.print(" Pa  T=");
    out.print(static_cast<long>(s_baseTempK * 100.0f));
    out.println(" cK");
  }

  if (s_imuOk) {
    sensors_event_t a, g, temp;
    s_imu.getEvent(&a, &g, &temp);
    out.print("accel x=");
    out.print(static_cast<long>(a.acceleration.x * 1000.0f));
    out.print(" y=");
    out.print(static_cast<long>(a.acceleration.y * 1000.0f));
    out.print(" z=");
    out.print(static_cast<long>(a.acceleration.z * 1000.0f));
    out.println(" mm/s2");
    out.print("gyro x=");
    out.print(static_cast<long>(g.gyro.x * 1000.0f));
    out.print(" y=");
    out.print(static_cast<long>(g.gyro.y * 1000.0f));
    out.print(" z=");
    out.print(static_cast<long>(g.gyro.z * 1000.0f));
    out.println(" mrad/s");
  }

  out.print("phase=");
  out.print(static_cast<int>(s_phase));
  out.print(" errorBits=0x");
  out.println(nodeErrorBits(), HEX);
}

static const AimConsoleHook s_consoleHooks[] = {
  {'p', "sensor snapshot", hookSensors},
};

const AimConsoleHook* nodeConsoleHooks(uint8_t& count) {
  count = sizeof(s_consoleHooks) / sizeof(s_consoleHooks[0]);
  return s_consoleHooks;
}
#endif
