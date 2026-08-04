#include "crawler_config.h"

#include <cmath>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if defined(ARDUINO)
#include <Preferences.h>
#endif

namespace crawler {
namespace config {
namespace servo {

ServoCalibration calibrations[kJointCount] = {
    // 160-degree servo travel centered at the policy zero position.
    {14, 1, 0.0f, 80.0f, 1.0f, -1.3962634f, 1.3962634f, 631, 2409,
     0.0f, 0.0f, false, false},
    {15, 2, 0.0f, 80.0f, 1.0f, -1.3962634f, 1.3962634f, 631, 2409,
     0.0f, 0.0f, false, false},
    {16, 4, 0.0f, 80.0f, 1.0f, -1.3962634f, 1.3962634f, 631, 2409,
     0.0f, 0.0f, false, false},
};

uint16_t rawAdcAtAngle[kJointCount][calibrationPointCount] = {};
bool rawTableValid[kJointCount] = {};

namespace {

constexpr uint32_t kCalibrationMagic = 0x43524157u;  // "CRAW"
constexpr uint16_t kCalibrationVersion = 4;

bool baseCalibrationValid(const ServoCalibration& c) {
  return c.valid && c.pwmPin >= 0 && c.feedbackAdcPin >= 0 &&
         std::isfinite(c.defaultPositionRad) &&
         std::isfinite(c.jointZeroServoDegrees) &&
         std::isfinite(c.directionSign) && c.directionSign != 0.0f &&
         std::isfinite(c.jointMinimumRad) &&
         std::isfinite(c.jointMaximumRad) &&
         c.jointMinimumRad <= c.jointMaximumRad && c.minimumPulseUs > 0 &&
         c.maximumPulseUs > c.minimumPulseUs;
}

bool legacyFeedbackValid(const ServoCalibration& c) {
  return std::isfinite(c.feedbackMillivoltsAtMinimum) &&
         std::isfinite(c.feedbackMillivoltsAtMaximum) &&
         c.feedbackMillivoltsAtMinimum < c.feedbackMillivoltsAtMaximum;
}

bool monotonicRawTable(const uint16_t table[calibrationPointCount]) {
  for (uint8_t i = 0; i < calibrationPointCount; ++i) {
    if (table[i] > 4095u) return false;
  }
  int direction = 0;
  for (uint8_t i = 1; i < calibrationPointCount; ++i) {
    const int difference = static_cast<int>(table[i]) -
                           static_cast<int>(table[i - 1]);
    if (difference == 0) continue;
    const int currentDirection = difference > 0 ? 1 : -1;
    if (direction == 0) {
      direction = currentDirection;
    } else if (direction != currentDirection) {
      return false;
    }
  }
  return direction != 0;
}

uint32_t checksum(const uint8_t* data, size_t length) {
  uint32_t value = 2166136261u;
  for (size_t i = 0; i < length; ++i) {
    value ^= data[i];
    value *= 16777619u;
  }
  return value;
}

#pragma pack(push, 1)
struct PersistentCalibration {
  uint32_t magic;
  uint16_t version;
  uint16_t pointCount;
  ServoCalibration calibrations[kJointCount];
  uint16_t rawAdcAtAngle[kJointCount][calibrationPointCount];
  uint8_t rawTableValid[kJointCount];
  uint32_t checksum;
};
#pragma pack(pop)

bool recordValid(const PersistentCalibration& record) {
  if (record.magic != kCalibrationMagic ||
      record.version != kCalibrationVersion ||
      record.pointCount != calibrationPointCount ||
      record.checksum != checksum(reinterpret_cast<const uint8_t*>(&record),
                                  offsetof(PersistentCalibration, checksum))) {
    return false;
  }
  for (uint8_t i = 0; i < kJointCount; ++i) {
    if (!baseCalibrationValid(record.calibrations[i]) ||
        record.rawTableValid[i] == 0u ||
        !monotonicRawTable(record.rawAdcAtAngle[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool hardwareConfigured() {
  for (uint8_t i = 0; i < kJointCount; ++i) {
    const ServoCalibration& c = calibrations[i];
    if (c.pwmPin < 0 || c.feedbackAdcPin < 0 ||
        !std::isfinite(c.defaultPositionRad) ||
        !std::isfinite(c.jointZeroServoDegrees) ||
        !std::isfinite(c.directionSign) || c.directionSign == 0.0f ||
        !std::isfinite(c.jointMinimumRad) ||
        !std::isfinite(c.jointMaximumRad) ||
        c.jointMinimumRad > c.jointMaximumRad || c.minimumPulseUs <= 0 ||
        c.maximumPulseUs <= c.minimumPulseUs) {
      return false;
    }
  }
  return true;
}

bool rawTableIsValid(uint8_t joint) {
  return joint < kJointCount && rawTableValid[joint] &&
         monotonicRawTable(rawAdcAtAngle[joint]);
}

bool calibrationValid() {
  if (!hardwareConfigured()) return false;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    if (!baseCalibrationValid(calibrations[i]) ||
        (!rawTableIsValid(i) && !legacyFeedbackValid(calibrations[i]))) {
      return false;
    }
  }
  return true;
}

void applyCalibration(
    const ServoCalibration* candidateCalibrations,
    const uint16_t candidateRawAdc[kJointCount][calibrationPointCount]) {
  if (candidateCalibrations == nullptr || candidateRawAdc == nullptr) return;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    calibrations[i] = candidateCalibrations[i];
    memcpy(rawAdcAtAngle[i], candidateRawAdc[i],
           sizeof(rawAdcAtAngle[i]));
    rawTableValid[i] = true;
  }
}

bool loadPersistentCalibration() {
#if defined(ARDUINO)
  Preferences preferences;
  if (!preferences.begin("crawler", true)) return false;
  const size_t length = preferences.getBytesLength("servo_cal");
  if (length != sizeof(PersistentCalibration)) {
    preferences.end();
    return false;
  }
  PersistentCalibration record = {};
  const size_t read =
      preferences.getBytes("servo_cal", &record, sizeof(record));
  preferences.end();
  if (read != sizeof(record) || !recordValid(record)) return false;

  for (uint8_t i = 0; i < kJointCount; ++i) {
    calibrations[i] = record.calibrations[i];
    memcpy(rawAdcAtAngle[i], record.rawAdcAtAngle[i],
           sizeof(rawAdcAtAngle[i]));
    rawTableValid[i] = true;
  }
  return true;
#else
  return false;
#endif
}

bool savePersistentCalibration(
    const ServoCalibration* candidateCalibrations,
    const uint16_t candidateRawAdc[kJointCount][calibrationPointCount]) {
  if (candidateCalibrations == nullptr || candidateRawAdc == nullptr) {
    return false;
  }

  PersistentCalibration record = {};
  record.magic = kCalibrationMagic;
  record.version = kCalibrationVersion;
  record.pointCount = calibrationPointCount;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    record.calibrations[i] = candidateCalibrations[i];
    memcpy(record.rawAdcAtAngle[i], candidateRawAdc[i],
           sizeof(record.rawAdcAtAngle[i]));
    record.rawTableValid[i] = 1u;
    if (!baseCalibrationValid(record.calibrations[i]) ||
        !monotonicRawTable(record.rawAdcAtAngle[i])) {
      return false;
    }
  }
  record.checksum = checksum(reinterpret_cast<const uint8_t*>(&record),
                             offsetof(PersistentCalibration, checksum));

#if defined(ARDUINO)
  Preferences preferences;
  if (!preferences.begin("crawler", false)) return false;
  const size_t written =
      preferences.putBytes("servo_cal", &record, sizeof(record));
  preferences.end();
  return written == sizeof(record);
#else
  (void)record;
  return false;
#endif
}

}  // namespace servo
}  // namespace config
}  // namespace crawler
