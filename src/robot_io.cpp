#include "robot_io.h"

#include <cmath>

#if !defined(ARDUINO)
#include <chrono>
#endif

#if defined(ARDUINO)
#include <Arduino.h>
#endif

namespace {

uint32_t monotonicMs() {
#if defined(ARDUINO)
  return millis();
#else
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now.time_since_epoch())
                                  .count());
#endif
}

float clampValue(float value, float minimum, float maximum) {
  return std::fmax(minimum, std::fmin(maximum, value));
}

}  // namespace

RobotIO::RobotIO()
    : mockStartMs_(0),
      mockWrites_(0),
      servosDisabled_(true),
      estimatorInitialized_(false),
      previousSampleMs_(0),
      previousPositionRad_{},
      latestMockTargetsRad_{},
      latestHardwareState_{},
      motionGate_(false),
      fastSafetyGate_(false)
#if defined(ARDUINO)
      , mutex_(portMUX_INITIALIZER_UNLOCKED), taskHandle_(nullptr)
#endif
{}

uint32_t RobotIO::nowMs() const { return monotonicMs(); }

bool RobotIO::begin() {
  mockStartMs_ = nowMs();
  mockWrites_ = 0;
  servosDisabled_ = true;
  estimatorInitialized_ = false;
  previousSampleMs_ = 0;
  latestHardwareState_ = {};
  motionGate_ = false;
  fastSafetyGate_ = false;
#if !CRAWLER_USE_MOCK_HARDWARE && defined(ARDUINO)
  if (calibrationValid()) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      if (ledcSetup(i, 50, 16) == 0) return false;
      ledcAttachPin(static_cast<uint8_t>(crawler::config::servo::calibrations[i].pwmPin), i);
    }
    if (xTaskCreatePinnedToCore(taskEntry, "crawler_joints", 4096, this, 9,
                                &taskHandle_, 0) != pdPASS) {
      return false;
    }
    delay(20);
  }
#endif
  disableServos();
  return true;
}

bool RobotIO::feedbackMillivoltsToJointRad(
    float millivolts, const crawler::ServoCalibration& calibration,
    float& positionRad) {
  if (!calibration.valid || !std::isfinite(millivolts) ||
      !std::isfinite(calibration.feedbackMillivoltsAtMinimum) ||
      !std::isfinite(calibration.feedbackMillivoltsAtMaximum) ||
      calibration.feedbackMillivoltsAtMinimum >=
          calibration.feedbackMillivoltsAtMaximum ||
      calibration.jointMinimumRad > calibration.jointMaximumRad ||
      millivolts < calibration.feedbackMillivoltsAtMinimum ||
      millivolts > calibration.feedbackMillivoltsAtMaximum) {
    return false;
  }
  float normalized =
      (millivolts - calibration.feedbackMillivoltsAtMinimum) /
      (calibration.feedbackMillivoltsAtMaximum -
       calibration.feedbackMillivoltsAtMinimum);
  if (calibration.feedbackInverted) normalized = 1.0f - normalized;
  positionRad = calibration.jointMinimumRad +
                normalized * (calibration.jointMaximumRad -
                             calibration.jointMinimumRad);
  return std::isfinite(positionRad);
}

int RobotIO::jointRadToPulseUs(
    float targetRad, const crawler::ServoCalibration& calibration) {
  if (!calibration.valid || !std::isfinite(targetRad) ||
      calibration.directionSign == 0.0f ||
      calibration.jointMinimumRad > calibration.jointMaximumRad ||
      calibration.minimumPulseUs <= 0 ||
      calibration.maximumPulseUs <= calibration.minimumPulseUs) {
    return -1;
  }
  const float commandedJointRad =
      calibration.defaultPositionRad + targetRad;
  const float limitedJointRad = clampValue(commandedJointRad,
                                            calibration.jointMinimumRad,
                                            calibration.jointMaximumRad);
  constexpr float radiansToDegrees = 57.29577951308232f;
  const float servoDegrees =
      calibration.jointZeroServoDegrees +
      calibration.directionSign *
          (limitedJointRad - calibration.defaultPositionRad) *
          radiansToDegrees;
  const float servoAtMinimum =
      calibration.jointZeroServoDegrees +
      calibration.directionSign *
          (calibration.jointMinimumRad - calibration.defaultPositionRad) *
          radiansToDegrees;
  const float servoAtMaximum =
      calibration.jointZeroServoDegrees +
      calibration.directionSign *
          (calibration.jointMaximumRad - calibration.defaultPositionRad) *
          radiansToDegrees;
  const float servoMinimum = std::fmin(servoAtMinimum, servoAtMaximum);
  const float servoMaximum = std::fmax(servoAtMinimum, servoAtMaximum);
  if (servoMinimum >= servoMaximum || !std::isfinite(servoDegrees)) return -1;
  const float normalized =
      (servoDegrees - servoMinimum) / (servoMaximum - servoMinimum);
  return static_cast<int>(static_cast<float>(calibration.minimumPulseUs) +
                          normalized * static_cast<float>(
                              calibration.maximumPulseUs -
                              calibration.minimumPulseUs) +
                          0.5f);
}

void RobotIO::updateEstimatedVelocity(const float position[3],
                                      uint32_t timestampMs,
                                      float velocity[3]) {
  if (!estimatorInitialized_) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      previousPositionRad_[i] = position[i];
      velocity[i] = 0.0f;
    }
    previousSampleMs_ = timestampMs;
    estimatorInitialized_ = true;
    return;
  }
  const uint32_t elapsedMs = timestampMs - previousSampleMs_;
  if (elapsedMs == 0u) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) velocity[i] = 0.0f;
    return;
  }
  const float elapsedSeconds = static_cast<float>(elapsedMs) * 0.001f;
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    velocity[i] = (position[i] - previousPositionRad_[i]) / elapsedSeconds;
    previousPositionRad_[i] = position[i];
  }
  previousSampleMs_ = timestampMs;
}

crawler::JointState RobotIO::sampleJointState() {
  crawler::JointState state = {};
  state.timestampMs = nowMs();
#if CRAWLER_USE_MOCK_HARDWARE
  const float elapsedSeconds =
      static_cast<float>(state.timestampMs - mockStartMs_) * 0.001f;
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    const float phase = static_cast<float>(i) * 0.7f;
    const float angle = 0.5f * elapsedSeconds + phase;
    state.positionRad[i] = 0.08f * std::sin(angle);
    state.velocityRadPerSecond[i] = 0.04f * std::cos(angle);
  }
  state.valid = true;
#else
#if defined(ARDUINO)
  if (!calibrationValid()) return state;
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    const uint32_t millivolts =
        analogReadMilliVolts(crawler::config::servo::calibrations[i].feedbackAdcPin);
    if (!feedbackMillivoltsToJointRad(
            static_cast<float>(millivolts),
            crawler::config::servo::calibrations[i], state.positionRad[i])) {
      return state;
    }
  }
  updateEstimatedVelocity(state.positionRad, state.timestampMs,
                          state.velocityRadPerSecond);
  state.valid = true;
#else
  state.valid = false;
#endif
#endif
  return state;
}

crawler::JointState RobotIO::readJointState() {
#if CRAWLER_USE_MOCK_HARDWARE
  return sampleJointState();
#elif defined(ARDUINO)
  crawler::JointState state = {};
  portENTER_CRITICAL(&mutex_);
  state = latestHardwareState_;
  portEXIT_CRITICAL(&mutex_);
  return state;
#else
  return {};
#endif
}

void RobotIO::writeTargets(const float targetRad[3]) {
  if (targetRad == nullptr) return;
  if (!motionGate_ || !fastSafetyGate_) {
    disableServos();
    return;
  }
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    if (!std::isfinite(targetRad[i])) {
      disableServos();
      return;
    }
  }
#if CRAWLER_USE_MOCK_HARDWARE
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    latestMockTargetsRad_[i] = targetRad[i];
  }
  ++mockWrites_;
  servosDisabled_ = false;
#else
#if defined(ARDUINO)
  if (!calibrationValid()) {
    disableServos();
    return;
  }
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    const int pulseUs = jointRadToPulseUs(
        targetRad[i], crawler::config::servo::calibrations[i]);
    if (pulseUs < 0) {
      disableServos();
      return;
    }
    const uint32_t duty = static_cast<uint32_t>(pulseUs) * 65535u / 20000u;
    ledcWrite(i, duty);
  }
  servosDisabled_ = false;
#endif
#endif
}

void RobotIO::holdCurrentPosition(const crawler::JointState& joints) {
  if (!joints.valid) {
    disableServos();
    return;
  }
  float holdTargetsRad[crawler::kJointCount] = {};
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    if (!std::isfinite(joints.positionRad[i])) {
      disableServos();
      return;
    }
    holdTargetsRad[i] =
        joints.positionRad[i] - crawler::config::servo::calibrations[i].defaultPositionRad;
  }
  writeTargets(holdTargetsRad);
}

void RobotIO::disableServos() {
  servosDisabled_ = true;
#if !CRAWLER_USE_MOCK_HARDWARE && defined(ARDUINO)
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) ledcWrite(i, 0);
#endif
}

bool RobotIO::calibrationValid() const {
  return crawler::config::servo::calibrationValid();
}

bool RobotIO::usingMockHardware() const {
#if CRAWLER_USE_MOCK_HARDWARE
  return true;
#else
  return false;
#endif
}

bool RobotIO::servosEnabled() const { return !servosDisabled_; }

uint32_t RobotIO::mockWriteCount() const { return mockWrites_; }

const float* RobotIO::latestMockTargets() const { return latestMockTargetsRad_; }

void RobotIO::setMotionGate(bool allowed) { motionGate_ = allowed; }

void RobotIO::setFastSafetyGate(bool allowed) { fastSafetyGate_ = allowed; }

#if defined(ARDUINO)
void RobotIO::taskEntry(void* argument) {
  static_cast<RobotIO*>(argument)->taskLoop();
}

void RobotIO::taskLoop() {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(2);
  for (;;) {
    const crawler::JointState sample = sampleJointState();
    if (sample.valid) {
      portENTER_CRITICAL(&mutex_);
      latestHardwareState_ = sample;
      portEXIT_CRITICAL(&mutex_);
    }
    vTaskDelayUntil(&lastWake, period == 0 ? 1 : period);
  }
}
#endif
