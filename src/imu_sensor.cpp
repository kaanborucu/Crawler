#include "imu_sensor.h"

#include <cmath>

#if defined(ARDUINO)
#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#include <chrono>
#endif

namespace {

constexpr float kStandardGravityMps2 = 9.80665f;
constexpr float kDegreesToRadians = 0.017453292519943295f;

uint32_t monotonicMs() {
#if defined(ARDUINO)
  return millis();
#else
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now.time_since_epoch())
          .count());
#endif
}

#if defined(ARDUINO)
bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t* data,
                   size_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  const size_t received = Wire.requestFrom(address, length, true);
  if (received != length) return false;
  for (size_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool readWhoAmI(uint8_t address) {
  uint8_t id = 0;
  return readRegisters(address, 0x75, &id, 1) && (id == 0x68 || id == 0x69);
}

int16_t decodeBigEndianInt16(const uint8_t* bytes) {
  return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) |
                              static_cast<uint16_t>(bytes[1]));
}

uint16_t readFifoCount(uint8_t address) {
  uint8_t bytes[2] = {};
  if (!readRegisters(address, 0x72, bytes, sizeof(bytes))) return 0;
  return static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8) |
                               static_cast<uint16_t>(bytes[1]));
}
#endif

}  // namespace

ImuSensor::ImuSensor()
    : address_(0), present_(false), latest_{}
#if defined(ARDUINO)
      , mutex_(portMUX_INITIALIZER_UNLOCKED), taskHandle_(nullptr)
#endif
{}

bool ImuSensor::begin() {
#if CRAWLER_USE_MOCK_HARDWARE
  present_ = true;
  latest_ = {};
  latest_.timestampMs = monotonicMs();
  latest_.valid = true;
  return true;
#elif defined(ARDUINO)
  Wire.begin(crawler::config::imu::sdaPin, crawler::config::imu::sclPin,
            crawler::config::imu::i2cFrequencyHz);

  const uint8_t candidates[] = {0x68, 0x69};
  for (const uint8_t candidate : candidates) {
    if (readWhoAmI(candidate)) {
      address_ = candidate;
      break;
    }
  }
  if (address_ == 0) return false;

  // Keep the MPU's internal sample rate at 1 kHz. Its DLPF removes the high
  // frequency noise; the FIFO task publishes the newest sample to the policy.
  if (!writeRegister(address_, 0x6B, 0x00) ||  // PWR_MGMT_1
      !writeRegister(address_, 0x19, 0x00) ||  // SMPLRT_DIV: 1 kHz
      !writeRegister(address_, 0x1A, 0x04) ||  // CONFIG: DLPF ~20 Hz
      !writeRegister(address_, 0x1B, 0x00) ||  // GYRO_CONFIG: +/-250 dps
      !writeRegister(address_, 0x1C, 0x00) ||  // ACCEL_CONFIG: +/-2 g
      !writeRegister(address_, 0x1D, 0x04) ||  // ACCEL_CONFIG2: DLPF ~21 Hz
      !writeRegister(address_, 0x6A, 0x04) ||  // USER_CTRL: reset FIFO
      !writeRegister(address_, 0x6A, 0x40) ||  // USER_CTRL: enable FIFO
      !writeRegister(address_, 0x23, 0x78) ||  // FIFO_EN: accel + gyro
      !writeRegister(address_, 0x38, 0x10)) {  // INT_ENABLE: FIFO overflow
    address_ = 0;
    return false;
  }

  present_ = true;
  if (xTaskCreatePinnedToCore(taskEntry, "crawler_imu", 4096, this, 8,
                              &taskHandle_, 0) != pdPASS) {
    present_ = false;
    address_ = 0;
    return false;
  }
  delay(20);
  return true;
#else
  return false;
#endif
}

crawler::ImuState ImuSensor::read() {
#if CRAWLER_USE_MOCK_HARDWARE
  crawler::ImuState state = latest_;
  state.timestampMs = monotonicMs();
  return state;
#elif defined(ARDUINO)
  crawler::ImuState state = {};
  portENTER_CRITICAL(&mutex_);
  state = latest_;
  portEXIT_CRITICAL(&mutex_);
  return state;
#else
  return {};
#endif
}

crawler::ImuState ImuSensor::readFifoSample() {
  crawler::ImuState state = {};
#if defined(ARDUINO)
  if (!present_ || address_ == 0) return state;

  uint8_t bytes[crawler::config::imu::fifoPacketBytes] = {};
  if (!readRegisters(address_, 0x74, bytes, sizeof(bytes))) return state;

  const int16_t accelRaw[3] = {decodeBigEndianInt16(&bytes[0]),
                                decodeBigEndianInt16(&bytes[2]),
                                decodeBigEndianInt16(&bytes[4])};
  const int16_t gyroRaw[3] = {decodeBigEndianInt16(&bytes[6]),
                              decodeBigEndianInt16(&bytes[8]),
                              decodeBigEndianInt16(&bytes[10])};
  constexpr float accelMps2PerCount = kStandardGravityMps2 / 16384.0f;
  constexpr float gyroRadPerSecondPerCount =
      (1.0f / 131.0f) * kDegreesToRadians;

  for (uint8_t outputAxis = 0; outputAxis < 3; ++outputAxis) {
    const int8_t accelAxis = crawler::config::imu::accelAxis[outputAxis];
    const int8_t gyroAxis = crawler::config::imu::gyroAxis[outputAxis];
    if (accelAxis < 0 || accelAxis > 2 || gyroAxis < 0 || gyroAxis > 2) {
      return state;
    }
    state.linearAccelerationMps2[outputAxis] =
        static_cast<float>(accelRaw[accelAxis]) * accelMps2PerCount *
        crawler::config::imu::accelSign[outputAxis];
    state.angularVelocityRadPerSecond[outputAxis] =
        static_cast<float>(gyroRaw[gyroAxis]) *
        gyroRadPerSecondPerCount * crawler::config::imu::gyroSign[outputAxis];
  }

  for (uint8_t axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(state.linearAccelerationMps2[axis]) ||
        !std::isfinite(state.angularVelocityRadPerSecond[axis])) {
      return {};
    }
  }
  state.timestampMs = monotonicMs();
  state.valid = true;
#endif
  return state;
}

void ImuSensor::publish(const crawler::ImuState& state) {
#if defined(ARDUINO)
  if (!state.valid) return;
  portENTER_CRITICAL(&mutex_);
  latest_ = state;
  portEXIT_CRITICAL(&mutex_);
#else
  (void)state;
#endif
}

#if defined(ARDUINO)
void ImuSensor::taskEntry(void* argument) {
  static_cast<ImuSensor*>(argument)->taskLoop();
}

void ImuSensor::taskLoop() {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1);
  for (;;) {
    uint16_t fifoCount = readFifoCount(address_);
    if (fifoCount > 1024u ||
        (fifoCount % crawler::config::imu::fifoPacketBytes) != 0u) {
      writeRegister(address_, 0x6A, 0x04);
      writeRegister(address_, 0x6A, 0x40);
      writeRegister(address_, 0x23, 0x78);
    } else {
      while (fifoCount >= crawler::config::imu::fifoPacketBytes) {
        publish(readFifoSample());
        fifoCount = static_cast<uint16_t>(
            fifoCount - crawler::config::imu::fifoPacketBytes);
      }
    }
    vTaskDelayUntil(&lastWake, period == 0 ? 1 : period);
  }
}
#endif
