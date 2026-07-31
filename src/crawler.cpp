#include "crawler.h"

#include <cmath>

#if defined(ARDUINO)
#include <Arduino.h>
#include <esp_timer.h>
#else
#include <chrono>
#endif

namespace {

uint64_t monotonicUs() {
#if defined(ARDUINO)
  return static_cast<uint64_t>(esp_timer_get_time());
#else
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                  now.time_since_epoch())
                                  .count());
#endif
}

uint32_t monotonicMs() {
  return static_cast<uint32_t>(monotonicUs() / 1000u);
}

const char* stateName(crawler::RobotState state) {
  switch (state) {
    case crawler::RobotState::Booting: return "BOOTING";
    case crawler::RobotState::Disarmed: return "DISARMED";
    case crawler::RobotState::Running: return "RUNNING";
    case crawler::RobotState::Fault: return "FAULT";
    case crawler::RobotState::EmergencyStopped: return "ESTOP";
  }
  return "UNKNOWN";
}

}  // namespace

Crawler::Crawler()
    : ble_(),
      robotIo_(),
      policy_(),
      safety_(),
      initialized_(false),
      nextCycleUs_(0),
      cycleCount_(0),
      missedDeadlines_(0),
      maximumInferenceUs_(0),
      latestJoints_{},
      latestCommand_{},
      latestResult_{} {}

uint64_t Crawler::nowUs() const { return monotonicUs(); }

bool Crawler::begin() {
  safety_.begin();
  initialized_ = false;
  if (!ble_.begin() || !robotIo_.begin()) return false;
  if (!robotIo_.usingMockHardware() && !robotIo_.calibrationValid()) {
    safety_.raiseFault(crawler::FaultCode::InvalidCalibration);
    return false;
  }
  if (!policy_.begin()) {
    safety_.raiseFault(crawler::FaultCode::PolicyLoadFailure);
    return false;
  }

  latestCommand_ = ble_.latestCommand();
  latestJoints_ = robotIo_.readJointState();
  if (!latestCommand_.valid) {
#if CRAWLER_USE_MOCK_HARDWARE
    latestCommand_.valid = true;
#else
    return false;
#endif
  }
  if (!policy_.initialize(latestJoints_, latestCommand_)) {
    safety_.raiseFault(crawler::FaultCode::SensorInvalid);
    return false;
  }
  nextCycleUs_ = nowUs();
  initialized_ = true;
#if defined(ARDUINO)
  Serial.printf("Crawler ready: mock=%u calibration=%u BLE=%u\n",
                robotIo_.usingMockHardware() ? 1u : 0u,
                robotIo_.calibrationValid() ? 1u : 0u,
                ble_.connected() ? 1u : 0u);
#endif
  return true;
}

void Crawler::update() {
  if (!initialized_) {
#if defined(ARDUINO)
    delay(10);
#endif
    return;
  }

  const uint64_t now = nowUs();
  if (now < nextCycleUs_) {
#if defined(ARDUINO)
    const uint64_t remainingUs = nextCycleUs_ - now;
    delayMicroseconds(static_cast<uint32_t>(remainingUs > 1000u ? 1000u
                                                                  : remainingUs));
#endif
    return;
  }
  controlCycle();
  nextCycleUs_ += crawler::config::policy::periodUs;
  if (nextCycleUs_ <= nowUs()) {
    nextCycleUs_ = nowUs() + crawler::config::policy::periodUs;
  }
}

void Crawler::controlCycle() {
  ++cycleCount_;
  latestCommand_ = ble_.latestCommand();
  latestJoints_ = robotIo_.readJointState();
  const bool calibrationReady =
      robotIo_.usingMockHardware() || robotIo_.calibrationValid();
  const crawler::SafetyDecision decision =
      safety_.evaluate(latestCommand_, latestJoints_, ble_.connected(),
                       calibrationReady);

  if (!decision.allowMotion) {
    robotIo_.disableServos();
    const crawler::RobotStatus status = {
        safety_.state(), safety_.fault(), ble_.connected(),
        latestCommand_.sequence, 0, missedDeadlines_};
    ble_.publishStatus(status);
    printTelemetry();
    return;
  }

  crawler::PolicyResult result = {};
  if (!policy_.step(latestJoints_, latestCommand_, result) || !result.valid) {
    const crawler::FaultCode failure = policy_.failureCode();
    safety_.raiseFault(failure == crawler::FaultCode::None
                           ? crawler::FaultCode::NonFinitePolicyOutput
                           : failure);
    robotIo_.disableServos();
    printTelemetry();
    return;
  }
  latestResult_ = result;
  if (result.inferenceTimeUs > maximumInferenceUs_) {
    maximumInferenceUs_ = result.inferenceTimeUs;
  }
  if (result.inferenceTimeUs > crawler::config::policy::deadlineUs) {
    ++missedDeadlines_;
    safety_.raiseFault(crawler::FaultCode::InferenceDeadlineMiss);
    robotIo_.disableServos();
  } else {
    robotIo_.writeTargets(result.filteredTargetRad);
  }

  const crawler::RobotStatus status = {
      safety_.state(), safety_.fault(), ble_.connected(), latestCommand_.sequence,
      result.inferenceTimeUs, missedDeadlines_};
  ble_.publishStatus(status);
  printTelemetry();
}

void Crawler::printTelemetry() {
#if defined(ARDUINO)
  if (cycleCount_ % crawler::config::telemetry::printEveryCycles != 0u) return;
  Serial.printf(
      "TEL t=%lu state=%s fault=%u ble=%u cmd=%.3f,%.3f enable=%u "
      "joint=%.4f,%.4f,%.4f vel=%.4f,%.4f,%.4f raw=%.4f,%.4f,%.4f "
      "filtered=%.4f,%.4f,%.4f infer_us=%lu max_infer_us=%lu missed=%lu "
      "heap=%lu\n",
      static_cast<unsigned long>(monotonicMs()), stateName(safety_.state()),
      static_cast<unsigned>(safety_.fault()), ble_.connected() ? 1u : 0u,
      static_cast<double>(latestCommand_.forwardMetersPerSecond),
      static_cast<double>(latestCommand_.lateralMetersPerSecond),
      latestCommand_.enableRequested ? 1u : 0u,
      static_cast<double>(latestJoints_.positionRad[0]),
      static_cast<double>(latestJoints_.positionRad[1]),
      static_cast<double>(latestJoints_.positionRad[2]),
      static_cast<double>(latestJoints_.velocityRadPerSecond[0]),
      static_cast<double>(latestJoints_.velocityRadPerSecond[1]),
      static_cast<double>(latestJoints_.velocityRadPerSecond[2]),
      static_cast<double>(latestResult_.rawActions[0]),
      static_cast<double>(latestResult_.rawActions[1]),
      static_cast<double>(latestResult_.rawActions[2]),
      static_cast<double>(latestResult_.filteredTargetRad[0]),
      static_cast<double>(latestResult_.filteredTargetRad[1]),
      static_cast<double>(latestResult_.filteredTargetRad[2]),
      static_cast<unsigned long>(latestResult_.inferenceTimeUs),
      static_cast<unsigned long>(maximumInferenceUs_),
      static_cast<unsigned long>(missedDeadlines_),
      static_cast<unsigned long>(ESP.getFreeHeap()));
#endif
}
