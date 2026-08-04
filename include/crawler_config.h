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
constexpr uint8_t observationSize = 85;
constexpr uint8_t actionCount = 3;
constexpr uint32_t rateHz = 50;
constexpr uint32_t periodUs = 20000;
// Policy results slower than this are skipped and counted; they never become
// active servo commands.
constexpr uint32_t deadlineUs = 40000;
constexpr float positionClampRad = 1.5707963f;
constexpr float velocityClampRadPerSecond = 20.0f;
constexpr float velocityScale = 0.1f;
constexpr float imuAccelerationClampMps2 = 50.0f;
constexpr float imuAccelerationScale = 0.1f;
constexpr float imuAngularVelocityClampRadPerSecond = 20.0f;
constexpr float imuAngularVelocityScale = 0.25f;
constexpr float commandClampMetersPerSecond = 1.5f;
constexpr float actionScaleRad = 1.4835298642f;
constexpr float filterPreviousWeight = 0.1f;
constexpr float filterNewWeight = 0.9f;
static_assert(historyFrames * 3 + historyFrames * 3 + historyFrames * 3 +
                      historyFrames * 3 + historyFrames * 3 +
                      historyFrames * 2 == observationSize,
              "Policy observation layout must contain exactly 85 values");
}

namespace imu {
constexpr uint8_t spiSckPin = 12;
constexpr uint8_t spiMisoPin = 13;
constexpr uint8_t spiMosiPin = 11;
constexpr uint8_t chipSelectPin = 10;
constexpr uint8_t interruptPin = 7;
constexpr int8_t resetPin = 6;
constexpr uint32_t spiClockHz = 3000000;
constexpr uint32_t accelReportIntervalUs = 4000;
constexpr uint32_t gyroReportIntervalUs = 2500;
constexpr uint32_t accelRequestedHz = 250;
constexpr uint32_t gyroRequestedHz = 400;
constexpr uint32_t accelStaleLimitUs = 20000;
constexpr uint32_t gyroStaleLimitUs = 10000;
constexpr uint32_t startupTimeoutMs = 5000;
constexpr uint32_t interruptFallbackMs = 5;

// The current wiring is treated as the training IMU frame. Change these
// values after confirming the physical sensor orientation against training.
constexpr int8_t accelAxis[3] = {0, 1, 2};
constexpr float accelSign[3] = {1.0f, 1.0f, 1.0f};
constexpr int8_t gyroAxis[3] = {0, 1, 2};
constexpr float gyroSign[3] = {1.0f, 1.0f, 1.0f};
}

namespace joints {
constexpr uint32_t configuredRateHz = 500;
}

namespace safety {
constexpr uint32_t commandTimeoutMs = 300;
constexpr uint32_t sensorTimeoutMs = 300;
}

namespace telemetry {
// Control loop is 50 Hz; print telemetry at 5 Hz.
constexpr uint32_t printEveryCycles = 10;
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
extern ServoCalibration calibrations[kJointCount];
constexpr uint16_t calibrationAngleStepDegrees = 10;
constexpr uint16_t calibrationMaximumAngleDegrees = 160;
constexpr uint8_t calibrationPointCount =
    calibrationMaximumAngleDegrees / calibrationAngleStepDegrees + 1;
constexpr uint16_t calibrationAdcSampleCount = 31;
constexpr uint16_t calibrationAdcTrimCount = 5;
// The first point moves a high-torque servo from neutral to 0 degrees.
// Allow enough time for that move to finish before sampling feedback.
constexpr uint32_t calibrationSettleMs = 300;
constexpr uint32_t calibrationFirstPointSettleMs = 1000;
constexpr uint32_t calibrationInitialZeroWaitMs = 1000;

extern uint16_t rawAdcAtAngle[kJointCount][calibrationPointCount];
extern bool rawTableValid[kJointCount];

bool hardwareConfigured();
bool calibrationValid();
bool rawTableIsValid(uint8_t joint);
bool loadPersistentCalibration();
bool savePersistentCalibration(
    const ServoCalibration* candidateCalibrations,
    const uint16_t candidateRawAdc[kJointCount][calibrationPointCount]);
void applyCalibration(
    const ServoCalibration* candidateCalibrations,
    const uint16_t candidateRawAdc[kJointCount][calibrationPointCount]);
}  // namespace servo

}  // namespace config
}  // namespace crawler
