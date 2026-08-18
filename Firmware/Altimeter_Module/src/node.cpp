// node.cpp - Altimeter Module Application Logic
// References:
//   - ICAO Standard Atmosphere Model (ISA) for barometric altitude equation
//   - MS5611-01BA03 Barometer Datasheet (Oversampling & Pressure scaling)
//   - MPU6050 6-DOF IMU Specification (Filter bandwidth & Range scaling)
//   - AIM Network v0.7.0 Protocol Spec & Migration Plan (Decision D12/D14)

#include "node.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_NeoPixel.h>
#include <MS5611.h>
#include <Wire.h>
#include <aim_flight_recorder.h>
#include <aim_job.h>
#include <logger.h>
#include <math.h>

#include <SparkFun_KX13X.h>

namespace {
// Physical Conversion Constants (Reference: ICAO Standard Atmosphere Model)
constexpr float kLapseRate            = -0.0065f;    // ISA temperature lapse rate (K/m)
constexpr float kBaroExponent         = 0.19026627f; // ISA pressure exponent (g * M) / (R * L)
constexpr float kMbarToPa             = 100.0f;      // mbar/hPa to Pa scale multiplier (MS5611 Datasheet)
constexpr float kCelsiusToKelvin      = 273.15f;     // Celsius to Kelvin offset
constexpr float kMToMm                 = 1000.0f;     // m/s^2 to mm/s^2 multiplier
constexpr float kRadToMrad             = 1000.0f;     // rad/s to mrad/s multiplier

constexpr uint16_t kBaselineSamples   = 50U;         // Ground baseline initialization sample count
constexpr uint32_t kI2cClockHz        = 400000U;     // Fast-Mode I2C clock speed (400 kHz)

// Hardware Error Bitmasks
constexpr uint16_t kErrBaroFail  = (1U << 0);
constexpr uint16_t kErrImuFail   = (1U << 1);
constexpr uint16_t kErrKx134Fail = (1U << 2);
}  // namespace

static bool s_lowPower = false;

static MS5611 s_baro(pins::kMs5611Addr);
static Adafruit_MPU6050 s_imu;
static SparkFun_KX134 s_kx134;

static bool s_baroOk  = false;
static bool s_imuOk   = false;
static bool s_kx134Ok = false;

static float s_basePressurePa = 101325.0f;
static float s_baseTempK      = 288.15f;

static int32_t s_pressPa = 0;
static int32_t s_tempCc  = 0;
static int32_t s_altCm   = 0;

// MPU6050
static int32_t s_imuAccelX  = 0;  // mm/s^2
static int32_t s_imuAccelY  = 0;  // mm/s^2
static int32_t s_imuAccelZ  = 0;  // mm/s^2
static int32_t s_gyroX   = 0;  // mrad/s
static int32_t s_gyroY   = 0;  // mrad/s
static int32_t s_gyroZ   = 0;  // mrad/s
// KX134 
static int32_t s_highAccelX  = 0;  // mm/s^2
static int32_t s_highAccelY  = 0;  // mm/s^2
static int32_t s_highAccelZ  = 0;  // mm/s^2

static aim::Job s_txJob(10U);   // 100 Hz CAN tx (loop caps ~55 Hz: 18 ms baro read)
static aim::Job s_logJob(10U);  // 100 Hz flight log

static Adafruit_NeoPixel s_rgbLeds(1U, pins::kRgbData, NEO_GRB + NEO_KHZ800);

static float computeAltitude(float pressurePa, float basePa, float baseTempK) {
  if (basePa <= 0.0f) return 0.0f;
  return (baseTempK / kLapseRate) * (powf(pressurePa / basePa, kBaroExponent) - 1.0f);
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
  Wire.setClock(kI2cClockHz);

  // Set bus timeout to prevent Wire.endTransmission() hanging indefinitely on bus fault
  Wire.setTimeout(10U);

  s_baroOk = s_baro.begin();
  if (s_baroOk) {
    s_baro.setOversampling(OSR_ULTRA_HIGH);  // 4096-sample hardware oversampling (MS5611 Datasheet)

    // Pad baseline (P0/T0) locked over 50 samples at boot (~0.5s).
    // Note: Boot in pressurized/windy bays causes baseline offset error; consider pad re-zero frame.
    float pressSum = 0.0f;
    float tempSum  = 0.0f;
    for (uint16_t i = 0U; i < kBaselineSamples; i++) {
      s_baro.read();
      pressSum += s_baro.getPressure() * kMbarToPa;
      tempSum  += s_baro.getTemperature() + kCelsiusToKelvin;
    }
    s_basePressurePa = pressSum / static_cast<float>(kBaselineSamples);
    s_baseTempK      = tempSum / static_cast<float>(kBaselineSamples);
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

  s_kx134Ok = s_kx134.begin(Wire, pins::kKx134Addr);
  if (s_kx134Ok) {
    s_kx134.softwareReset();
    delay(50);
    s_kx134.setRange(SFE_KX134_RANGE64G);
    s_kx134.enableAccel();
    LOG_INFO("KX134 ready (SparkFun lib, 64g range)");
  } else {
    LOG_WARN("KX134 init failed");
  }

  LOG_INFO("Altimeter init baro=%u imu=%u kx134=%u",
           static_cast<unsigned>(s_baroOk ? 1U : 0U),
           static_cast<unsigned>(s_imuOk ? 1U : 0U),
           static_cast<unsigned>(s_kx134Ok ? 1U : 0U));
}

void nodeUpdate(uint32_t nowMs) {
  (void)nowMs;
  updateLed(nodeCurrentState());

  if (s_baroOk) {
    s_baro.read();
    const float pressurePa = s_baro.getPressure() * kMbarToPa;
    const float tempC      = s_baro.getTemperature();
    const float altM       = computeAltitude(pressurePa, s_basePressurePa, s_baseTempK);

    s_pressPa = static_cast<int32_t>(pressurePa);
    s_tempCc  = static_cast<int32_t>(tempC * 100.0f);
    s_altCm   = static_cast<int32_t>(altM * 100.0f);
  }

  if (s_imuOk) {
    sensors_event_t a, g, temp;
    s_imu.getEvent(&a, &g, &temp);
    s_imuAccelX = static_cast<int32_t>(a.acceleration.x * kMToMm);
    s_imuAccelY = static_cast<int32_t>(a.acceleration.y * kMToMm);
    s_imuAccelZ = static_cast<int32_t>(a.acceleration.z * kMToMm);
    s_gyroX  = static_cast<int32_t>(g.gyro.x * kRadToMrad);
    s_gyroY  = static_cast<int32_t>(g.gyro.y * kRadToMrad);
    s_gyroZ  = static_cast<int32_t>(g.gyro.z * kRadToMrad);
  }

  if (s_kx134Ok) {
    outputData kxData;
    if (s_kx134.getAccelData(&kxData)) {
      s_highAccelX = static_cast<int32_t>(kxData.xData * 9806.65f);
      s_highAccelY = static_cast<int32_t>(kxData.yData * 9806.65f);
      s_highAccelZ = static_cast<int32_t>(kxData.zData * 9806.65f);
    }
  }
}

void nodeServiceLog(uint32_t nowMs, AimFlightRecorder& recorder) {
  if (!s_logJob.due(nowMs)) return;
  if (!s_baroOk) return;

  uint32_t rowData[kLogCols] = {
    nowMs,
    static_cast<uint32_t>(s_pressPa),
    AimFlightRecorder::unsignify(s_altCm),
    AimFlightRecorder::unsignify(s_imuAccelX),
    AimFlightRecorder::unsignify(s_imuAccelY),
    AimFlightRecorder::unsignify(s_imuAccelZ),
    AimFlightRecorder::unsignify(s_gyroX),
    AimFlightRecorder::unsignify(s_gyroY),
    AimFlightRecorder::unsignify(s_gyroZ),
    AimFlightRecorder::unsignify(s_highAccelX),
    AimFlightRecorder::unsignify(s_highAccelY),
    AimFlightRecorder::unsignify(s_highAccelZ)
  };

  recorder.writeRow(rowData);
}

void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim) {
  if (!s_txJob.due(nowMs)) return;

  // Decoupled sensor checks: baro failure does NOT block high-g acceleration transmission over CAN
  if (s_baroOk) {
    aim::Msg altMsg = {};
    altMsg.cls     = aim::Class::Sensor;
    altMsg.subject = aim::subject::Altitude;
    altMsg.setSensorValue(s_altCm);
    (void)aim.send(altMsg);
  }

  if (s_kx134Ok || s_imuOk) {
    aim::Msg accMsg = {};
    accMsg.cls     = aim::Class::Sensor;
    accMsg.subject = aim::subject::Acceleration;
    const float ax = static_cast<float>(s_kx134Ok ? s_highAccelX : s_imuAccelX);
    const float ay = static_cast<float>(s_kx134Ok ? s_highAccelY : s_imuAccelY);
    const float az = static_cast<float>(s_kx134Ok ? s_highAccelZ : s_imuAccelZ);
    const int32_t magAccel = static_cast<int32_t>(sqrtf(ax * ax + ay * ay + az * az));
    accMsg.setSensorValue(magAccel);
    (void)aim.send(accMsg);
  }
}


void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  (void)nowMs;
  if (m.cls == aim::Class::Event) {
    if (m.subject == aim::subject::LowPower) {
      s_lowPower = (m.b[0] == 1U);
      LOG_INFO("Altimeter low power state updated: %d", s_lowPower);
    }
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
  if (!s_baroOk)  bits |= kErrBaroFail;
  if (!s_imuOk)   bits |= kErrImuFail;
  if (!s_kx134Ok) bits |= kErrKx134Fail;
  return bits;
}

#ifndef FLIGHT_BUILD
static void hookSensors(Stream& out) {
  out.print("baro=");
  out.print(s_baroOk ? "OK" : "FAIL");
  out.print(" imu=");
  out.print(s_imuOk ? "OK" : "FAIL");
  out.print(" kx134=");
  out.println(s_kx134Ok ? "OK" : "FAIL");

  if (s_baroOk) {
    out.print("P=");
    out.print(static_cast<long>(s_pressPa));
    out.print(" Pa  T=");
    out.print(static_cast<long>(s_tempCc));
    out.println(" cC");
    out.print("alt=");
    out.print(static_cast<long>(s_altCm));
    out.println(" cm");
  }

  if (s_imuOk) {
    out.print("imu accel xyz(mm/s2)=");
    out.print(static_cast<long>(s_imuAccelX));
    out.print(",");
    out.print(static_cast<long>(s_imuAccelY));
    out.print(",");
    out.println(static_cast<long>(s_imuAccelZ));

    out.print("gyro xyz(mrad/s)=");
    out.print(static_cast<long>(s_gyroX));
    out.print(",");
    out.print(static_cast<long>(s_gyroY));
    out.print(",");
    out.println(static_cast<long>(s_gyroZ));
  }

  if (s_kx134Ok) {
    out.print("highg accel xyz(mm/s2)=");
    out.print(static_cast<long>(s_highAccelX));
    out.print(",");
    out.print(static_cast<long>(s_highAccelY));
    out.print(",");
    out.println(static_cast<long>(s_highAccelZ));
  }

  out.print("txPeriodMs=");
  out.print(static_cast<unsigned long>(s_txJob.periodMs));
  out.print(" logPeriodMs=");
  out.print(static_cast<unsigned long>(s_logJob.periodMs));
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
