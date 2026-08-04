#include "robot_io.h"

#include <cmath>
#include <string.h>

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

#if !CRAWLER_USE_MOCK_HARDWARE && defined(ARDUINO)
constexpr uint8_t kServoPwmResolutionBits = 14;
constexpr uint32_t kServoPwmDutyMaximum =
    (1u << kServoPwmResolutionBits) - 1u;
constexpr uint32_t kServoPwmPeriodUs = 20000u;
#endif

#if !CRAWLER_USE_MOCK_HARDWARE && defined(ARDUINO)
bool rawAdcToServoDegrees(uint16_t rawAdc, uint8_t joint,
                          float& servoDegrees) {
  if (joint >= crawler::kJointCount ||
      !crawler::config::servo::rawTableIsValid(joint)) {
    return false;
  }
  const uint16_t* table = crawler::config::servo::rawAdcAtAngle[joint];
  if (rawAdc == table[0]) {
    servoDegrees = 0.0f;
    return true;
  }
  for (uint8_t i = 1; i < crawler::config::servo::calibrationPointCount; ++i) {
    const uint16_t lowRaw = table[i - 1];
    const uint16_t highRaw = table[i];
    const bool inInterval =
        (lowRaw <= rawAdc && rawAdc <= highRaw) ||
        (highRaw <= rawAdc && rawAdc <= lowRaw);
    if (!inInterval || lowRaw == highRaw) continue;
    const float ratio = (static_cast<float>(rawAdc) -
                         static_cast<float>(lowRaw)) /
                        (static_cast<float>(highRaw) -
                         static_cast<float>(lowRaw));
    servoDegrees = static_cast<float>((i - 1u) *
                                      crawler::config::servo::calibrationAngleStepDegrees) +
                    ratio * static_cast<float>(
                               crawler::config::servo::calibrationAngleStepDegrees);
    return std::isfinite(servoDegrees);
  }
  if ((table[0] > table[crawler::config::servo::calibrationPointCount - 1u] &&
       rawAdc >= table[0]) ||
      (table[0] < table[crawler::config::servo::calibrationPointCount - 1u] &&
       rawAdc <= table[0])) {
    servoDegrees = 0.0f;
  } else {
    servoDegrees = static_cast<float>(
        crawler::config::servo::calibrationMaximumAngleDegrees);
  }
  return true;
}

int servoDegreesToPulseUs(float servoDegrees,
                          const crawler::ServoCalibration& calibration) {
  if (!std::isfinite(servoDegrees) || calibration.minimumPulseUs <= 0 ||
      calibration.maximumPulseUs <= calibration.minimumPulseUs) {
    return -1;
  }
  const float limitedDegrees = clampValue(
      servoDegrees, 0.0f,
      static_cast<float>(crawler::config::servo::calibrationMaximumAngleDegrees));
  return static_cast<int>(static_cast<float>(calibration.minimumPulseUs) +
                          (limitedDegrees /
                           static_cast<float>(crawler::config::servo::calibrationMaximumAngleDegrees)) *
                              static_cast<float>(calibration.maximumPulseUs -
                                                 calibration.minimumPulseUs) +
                              0.5f);
}
#endif

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
      fastSafetyGate_(false),
      calibrationState_(ServoCalibrationState::Idle),
      calibrationPhase_(CalibrationPhase::InitialZeroWait),
      calibrationJoint_(0),
      calibrationPoint_(0),
      calibrationSettleUntilMs_(0),
      calibrationSamples_{}
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
  calibrationState_ = ServoCalibrationState::Idle;
  calibrationPhase_ = CalibrationPhase::InitialZeroWait;
  calibrationJoint_ = 0;
  calibrationPoint_ = 0;
  calibrationSettleUntilMs_ = 0;
  memset(calibrationSamples_, 0, sizeof(calibrationSamples_));
#if !CRAWLER_USE_MOCK_HARDWARE && defined(ARDUINO)
  crawler::config::servo::loadPersistentCalibration();
  if (crawler::config::servo::hardwareConfigured()) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      if (ledcSetup(i, 50, kServoPwmResolutionBits) == 0) return false;
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
    const crawler::ServoCalibration& calibration =
        crawler::config::servo::calibrations[i];
    if (crawler::config::servo::rawTableIsValid(i)) {
      const int raw = analogRead(calibration.feedbackAdcPin);
      float servoDegrees = 0.0f;
      if (raw < 0 || !rawAdcToServoDegrees(static_cast<uint16_t>(raw), i,
                                            servoDegrees)) {
        return state;
      }
      constexpr float degreesToRadians = 0.017453292519943295f;
      const float position = calibration.defaultPositionRad +
                             calibration.directionSign *
                                 (servoDegrees -
                                  calibration.jointZeroServoDegrees) *
                                 degreesToRadians;
      state.positionRad[i] = clampValue(position, calibration.jointMinimumRad,
                                        calibration.jointMaximumRad);
    } else {
      const uint32_t millivolts =
          analogReadMilliVolts(calibration.feedbackAdcPin);
      if (!feedbackMillivoltsToJointRad(static_cast<float>(millivolts),
                                        calibration,
                                        state.positionRad[i])) {
        return state;
      }
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
    const uint32_t duty = static_cast<uint32_t>(pulseUs) *
                          kServoPwmDutyMaximum / kServoPwmPeriodUs;
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
  // The hardware build can intentionally boot before mechanical calibration
  // is filled in. In that state LEDC was never initialized, so do not call
  // ledcWrite() just to enforce the already-disabled state.
  if (crawler::config::servo::hardwareConfigured()) {
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) ledcWrite(i, 0);
  }
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

bool RobotIO::startCalibration(const crawler::JointState& currentJoints) {
#if !CRAWLER_USE_MOCK_HARDWARE && defined(ARDUINO)
  if (calibrationActive() || !crawler::config::servo::hardwareConfigured()) {
    return false;
  }
#if defined(ARDUINO)
  Serial0.printf("CAL START: 3 joints, %u points, 0-%u degrees\n",
                 static_cast<unsigned>(crawler::config::servo::calibrationPointCount),
                 static_cast<unsigned>(crawler::config::servo::calibrationMaximumAngleDegrees));
#endif
  (void)currentJoints;
  if (!writeAllCalibrationPose(0.0f)) {
#if defined(ARDUINO)
    Serial0.println("CAL FAIL reason=pwm_write_initial_zero");
#endif
    calibrationState_ = ServoCalibrationState::Failed;
    disableServos();
    return false;
  }
  calibrationState_ = ServoCalibrationState::Running;
  calibrationPhase_ = CalibrationPhase::InitialZeroWait;
  calibrationJoint_ = 0;
  calibrationPoint_ = 0;
  calibrationSettleUntilMs_ =
      nowMs() + crawler::config::servo::calibrationInitialZeroWaitMs;
  memset(calibrationSamples_, 0, sizeof(calibrationSamples_));
#if defined(ARDUINO)
  Serial0.printf("CAL initial pose applied: all joints at 0 degrees; waiting %u ms\n",
                 static_cast<unsigned>(
                     crawler::config::servo::calibrationInitialZeroWaitMs));
#endif
  return true;
#else
  (void)currentJoints;
  return false;
#endif
}

void RobotIO::updateCalibration() {
#if !CRAWLER_USE_MOCK_HARDWARE && defined(ARDUINO)
  if (!calibrationActive()) return;
  const uint32_t now = nowMs();

  if (calibrationPhase_ == CalibrationPhase::InitialZeroWait) {
    if (static_cast<int32_t>(now - calibrationSettleUntilMs_) < 0) return;
    calibrationPhase_ = CalibrationPhase::Sampling;
    calibrationSettleUntilMs_ = 0;
    return;
  }

  if (calibrationPhase_ == CalibrationPhase::NeutralWait) {
    if (calibrationSettleUntilMs_ == 0u) {
      if (!writeCalibrationPose(
              calibrationJoint_,
              crawler::config::servo::calibrations[calibrationJoint_]
                  .jointZeroServoDegrees)) {
        Serial0.printf("CAL FAIL reason=pwm_write_neutral joint=%u\n",
                       static_cast<unsigned>(calibrationJoint_));
        calibrationState_ = ServoCalibrationState::Failed;
        disableServos();
        return;
      }
      Serial0.printf(
          "CAL joint=%u complete; neutral pose applied at %.0f degrees\n",
          static_cast<unsigned>(calibrationJoint_),
          crawler::config::servo::calibrations[calibrationJoint_]
              .jointZeroServoDegrees);
      calibrationSettleUntilMs_ = now + crawler::config::servo::calibrationSettleMs;
      return;
    }
    if (static_cast<int32_t>(now - calibrationSettleUntilMs_) < 0) return;

    calibrationSettleUntilMs_ = 0;
    if (calibrationJoint_ + 1u < crawler::kJointCount) {
      ++calibrationJoint_;
      calibrationPoint_ = 0;
      calibrationPhase_ = CalibrationPhase::Sampling;
      return;
    }
    finishCalibration();
    return;
  }

  if (calibrationSettleUntilMs_ == 0u) {
    if (!writeCalibrationPose(
            calibrationJoint_,
            static_cast<float>(calibrationPoint_ *
                               crawler::config::servo::calibrationAngleStepDegrees))) {
#if defined(ARDUINO)
      Serial0.printf("CAL FAIL reason=pwm_write joint=%u angle_deg=%u\n",
                     static_cast<unsigned>(calibrationJoint_),
                     static_cast<unsigned>(calibrationPoint_ *
                                           crawler::config::servo::calibrationAngleStepDegrees));
#endif
      calibrationState_ = ServoCalibrationState::Failed;
      disableServos();
      return;
    }
    const uint32_t settleMs =
        calibrationPoint_ == 0u
            ? crawler::config::servo::calibrationFirstPointSettleMs
            : crawler::config::servo::calibrationSettleMs;
    calibrationSettleUntilMs_ = now + settleMs;
    return;
  }
  if (static_cast<int32_t>(now - calibrationSettleUntilMs_) < 0) return;

  uint16_t rawAdc = 0;
  if (!readFilteredRawAdc(calibrationJoint_, rawAdc)) {
#if defined(ARDUINO)
    Serial0.printf("CAL FAIL reason=adc_read joint=%u angle_deg=%u pin=%d\n",
                   static_cast<unsigned>(calibrationJoint_),
                   static_cast<unsigned>(calibrationPoint_ *
                                         crawler::config::servo::calibrationAngleStepDegrees),
                   crawler::config::servo::calibrations[calibrationJoint_]
                       .feedbackAdcPin);
#endif
    calibrationState_ = ServoCalibrationState::Failed;
    disableServos();
    return;
  }
  calibrationSamples_[calibrationJoint_][calibrationPoint_] = rawAdc;
#if defined(ARDUINO)
  Serial0.printf("CAL sample joint=%u angle_deg=%u raw_adc=%u\n",
                 static_cast<unsigned>(calibrationJoint_),
                 static_cast<unsigned>(calibrationPoint_ *
                                       crawler::config::servo::calibrationAngleStepDegrees),
                 static_cast<unsigned>(rawAdc));
#endif
  calibrationSettleUntilMs_ = 0;

  if (calibrationPoint_ + 1u < crawler::config::servo::calibrationPointCount) {
    ++calibrationPoint_;
    return;
  }
  calibrationPhase_ = CalibrationPhase::NeutralWait;
#endif
}

void RobotIO::abortCalibration() {
  if (!calibrationActive()) return;
  calibrationState_ = ServoCalibrationState::Failed;
  calibrationSettleUntilMs_ = 0;
  disableServos();
}

bool RobotIO::calibrationActive() const {
  return calibrationState_ == ServoCalibrationState::Running;
}

ServoCalibrationState RobotIO::calibrationState() const {
  return calibrationState_;
}

bool RobotIO::writeCalibrationPose(uint8_t activeJoint, float servoDegrees) {
#if !CRAWLER_USE_MOCK_HARDWARE && defined(ARDUINO)
  if (activeJoint >= crawler::kJointCount ||
      !crawler::config::servo::hardwareConfigured()) {
    return false;
  }
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    const crawler::ServoCalibration& calibration =
        crawler::config::servo::calibrations[i];
    float targetDegrees =
        i < activeJoint
            ? crawler::config::servo::calibrations[i].jointZeroServoDegrees
            : 0.0f;
    if (i == activeJoint) {
      targetDegrees = servoDegrees;
    }
    const int pulseUs = servoDegreesToPulseUs(targetDegrees, calibration);
    if (pulseUs < 0) return false;
    const uint32_t duty = static_cast<uint32_t>(pulseUs) *
                          kServoPwmDutyMaximum / kServoPwmPeriodUs;
    ledcWrite(i, duty);
  }
  servosDisabled_ = false;
  return true;
#else
  (void)activeJoint;
  (void)servoDegrees;
  return false;
#endif
}

bool RobotIO::writeAllCalibrationPose(float servoDegrees) {
#if !CRAWLER_USE_MOCK_HARDWARE && defined(ARDUINO)
  if (!crawler::config::servo::hardwareConfigured() ||
      !std::isfinite(servoDegrees)) {
    return false;
  }
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    const crawler::ServoCalibration& calibration =
        crawler::config::servo::calibrations[i];
    const int pulseUs = servoDegreesToPulseUs(servoDegrees, calibration);
    if (pulseUs < 0) return false;
    const uint32_t duty = static_cast<uint32_t>(pulseUs) *
                          kServoPwmDutyMaximum / kServoPwmPeriodUs;
    ledcWrite(i, duty);
  }
  servosDisabled_ = false;
  return true;
#else
  (void)servoDegrees;
  return false;
#endif
}

bool RobotIO::writeNeutralCalibrationPose() {
#if !CRAWLER_USE_MOCK_HARDWARE && defined(ARDUINO)
  if (!crawler::config::servo::hardwareConfigured()) return false;
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    const crawler::ServoCalibration& calibration =
        crawler::config::servo::calibrations[i];
    const int pulseUs =
        servoDegreesToPulseUs(calibration.jointZeroServoDegrees, calibration);
    if (pulseUs < 0) return false;
    const uint32_t duty = static_cast<uint32_t>(pulseUs) *
                          kServoPwmDutyMaximum / kServoPwmPeriodUs;
    ledcWrite(i, duty);
  }
  servosDisabled_ = false;
  return true;
#else
  return false;
#endif
}

bool RobotIO::readFilteredRawAdc(uint8_t joint, uint16_t& rawAdc) const {
#if !CRAWLER_USE_MOCK_HARDWARE && defined(ARDUINO)
  if (joint >= crawler::kJointCount) return false;
  int samples[crawler::config::servo::calibrationAdcSampleCount] = {};
  const int pin = crawler::config::servo::calibrations[joint].feedbackAdcPin;
  if (pin < 0) return false;
  for (uint16_t i = 0; i < crawler::config::servo::calibrationAdcSampleCount;
       ++i) {
    samples[i] = analogRead(pin);
    delayMicroseconds(500);
  }
  for (uint16_t i = 1; i < crawler::config::servo::calibrationAdcSampleCount;
       ++i) {
    const int value = samples[i];
    int j = static_cast<int>(i) - 1;
    while (j >= 0 && samples[j] > value) {
      samples[j + 1] = samples[j];
      --j;
    }
    samples[j + 1] = value;
  }
  uint32_t total = 0;
  const uint16_t first = crawler::config::servo::calibrationAdcTrimCount;
  const uint16_t last = crawler::config::servo::calibrationAdcSampleCount -
                        crawler::config::servo::calibrationAdcTrimCount;
  if (first >= last) return false;
  for (uint16_t i = first; i < last; ++i) {
    if (samples[i] < 0 || samples[i] > 4095) return false;
    total += static_cast<uint32_t>(samples[i]);
  }
  rawAdc = static_cast<uint16_t>(
      (total + (last - first) / 2u) / (last - first));
  return true;
#else
  (void)joint;
  (void)rawAdc;
  return false;
#endif
}

bool RobotIO::validateCalibrationSamples() const {
  for (uint8_t joint = 0; joint < crawler::kJointCount; ++joint) {
    int direction = 0;
    for (uint8_t point = 1;
         point < crawler::config::servo::calibrationPointCount; ++point) {
      const int difference =
          static_cast<int>(calibrationSamples_[joint][point]) -
          static_cast<int>(calibrationSamples_[joint][point - 1u]);
      if (difference == 0) continue;
      const int currentDirection = difference > 0 ? 1 : -1;
      if (direction == 0) {
        direction = currentDirection;
      } else if (direction != currentDirection) {
        return false;
      }
    }
    if (direction == 0) return false;
  }
  return true;
}

bool RobotIO::finishCalibration() {
  if (!validateCalibrationSamples()) {
#if defined(ARDUINO)
    Serial0.println("CAL FAIL reason=adc_table_flat_or_non_monotonic");
    for (uint8_t joint = 0; joint < crawler::kJointCount; ++joint) {
      uint16_t minimum = UINT16_MAX;
      uint16_t maximum = 0;
      int direction = 0;
      int reversalPoint = -1;
      for (uint8_t point = 0;
           point < crawler::config::servo::calibrationPointCount; ++point) {
        const uint16_t value = calibrationSamples_[joint][point];
        if (value < minimum) minimum = value;
        if (value > maximum) maximum = value;
        if (point == 0) continue;
        const int difference = static_cast<int>(value) -
                               static_cast<int>(calibrationSamples_[joint][point - 1u]);
        if (difference == 0) continue;
        const int currentDirection = difference > 0 ? 1 : -1;
        if (direction == 0) {
          direction = currentDirection;
        } else if (direction != currentDirection && reversalPoint < 0) {
          reversalPoint = point;
        }
      }
      Serial0.printf(
          "CAL table joint=%u first=%u last=%u min=%u max=%u direction=%d reversal_point=%d\n",
          static_cast<unsigned>(joint),
          static_cast<unsigned>(calibrationSamples_[joint][0]),
          static_cast<unsigned>(calibrationSamples_[joint][crawler::config::servo::calibrationPointCount - 1u]),
          static_cast<unsigned>(minimum), static_cast<unsigned>(maximum), direction,
          reversalPoint);
    }
#endif
    calibrationState_ = ServoCalibrationState::Failed;
    disableServos();
    return false;
  }
  crawler::ServoCalibration candidate[crawler::kJointCount] = {};
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    candidate[i] = crawler::config::servo::calibrations[i];
    candidate[i].valid = true;
  }
#if defined(ARDUINO)
  Serial0.println("CAL ADC tables validated; saving to NVS");
#endif
  if (!crawler::config::servo::savePersistentCalibration(
          candidate, calibrationSamples_)) {
#if defined(ARDUINO)
    Serial0.println("CAL FAIL reason=nvs_save");
#endif
    calibrationState_ = ServoCalibrationState::Failed;
    disableServos();
    return false;
  }
  crawler::config::servo::applyCalibration(candidate, calibrationSamples_);
  if (!writeNeutralCalibrationPose()) {
#if defined(ARDUINO)
    Serial0.println("CAL FAIL reason=neutral_pose_write");
#endif
    calibrationState_ = ServoCalibrationState::Failed;
    disableServos();
    return false;
  }
#if defined(ARDUINO)
  Serial0.println("CAL NVS save OK");
  Serial0.printf("CAL neutral pose applied: all joints at %.0f degrees\n",
                 crawler::config::servo::calibrations[0].jointZeroServoDegrees);
#endif
  calibrationState_ = ServoCalibrationState::Completed;
  return true;
}

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
