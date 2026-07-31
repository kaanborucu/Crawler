#pragma once

#include <cmath>
#include <stdint.h>

#include "crawler_types.h"

#ifndef CRAWLER_USE_MOCK_HARDWARE
#define CRAWLER_USE_MOCK_HARDWARE 1
#endif

#ifndef CRAWLER_ENABLE_BLE
#define CRAWLER_ENABLE_BLE 1
#endif

namespace crawler {
namespace config {

namespace policy {
constexpr uint8_t historyFrames = 5;
constexpr uint8_t observationSize = 55;
constexpr uint8_t actionCount = 3;
constexpr uint32_t periodUs = 20000;
constexpr uint32_t deadlineUs = 20000;
constexpr float positionClampRad = 1.5707963f;
constexpr float velocityClampRadPerSecond = 20.0f;
constexpr float velocityScale = 0.1f;
constexpr float commandClampMetersPerSecond = 1.5f;
constexpr float actionScaleRad = 1.4835298642f;
constexpr float filterPreviousWeight = 0.1f;
constexpr float filterNewWeight = 0.9f;
static_assert(historyFrames * 3 + historyFrames * 3 + historyFrames * 3 +
                      historyFrames * 2 == observationSize,
              "Policy observation layout must contain exactly 55 values");
}

namespace safety {
constexpr uint32_t commandTimeoutMs = 300;
constexpr uint32_t sensorTimeoutMs = 300;
constexpr float armPositionToleranceRad = 0.2f;
}

namespace telemetry {
constexpr uint32_t printEveryCycles = 5;
}

namespace ble {
constexpr char deviceName[] = "Crawler-S3";
constexpr char serviceUuid[] = "7f1f0001-9f2e-4c9c-9d53-4e2a4d4b0101";
constexpr char commandUuid[] = "7f1f0002-9f2e-4c9c-9d53-4e2a4d4b0101";
constexpr char statusUuid[] = "7f1f0003-9f2e-4c9c-9d53-4e2a4d4b0101";
constexpr uint8_t protocolVersion = 1;
}

namespace servo {
// These values are deliberately invalid until the physical robot is measured.
static const ServoCalibration calibrations[kJointCount] = {
    {-1, -1, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, false,
     false},
    {-1, -1, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, false,
     false},
    {-1, -1, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, false,
     false},
};

inline bool calibrationValid() {
  for (uint8_t i = 0; i < kJointCount; ++i) {
    const ServoCalibration& c = calibrations[i];
    if (!c.valid || c.pwmPin < 0 || c.feedbackAdcPin < 0 ||
        !std::isfinite(c.defaultPositionRad) ||
        !std::isfinite(c.jointZeroServoDegrees) ||
        !std::isfinite(c.directionSign) || c.directionSign == 0.0f ||
        !std::isfinite(c.jointMinimumRad) ||
        !std::isfinite(c.jointMaximumRad) ||
        c.jointMinimumRad > c.jointMaximumRad || c.minimumPulseUs <= 0 ||
        c.maximumPulseUs <= c.minimumPulseUs ||
        !std::isfinite(c.feedbackMillivoltsAtMinimum) ||
        !std::isfinite(c.feedbackMillivoltsAtMaximum) ||
        c.feedbackMillivoltsAtMinimum >= c.feedbackMillivoltsAtMaximum) {
      return false;
    }
  }
  return true;
}
}  // namespace servo

}  // namespace config
}  // namespace crawler
