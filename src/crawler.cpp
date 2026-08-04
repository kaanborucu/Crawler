#include "crawler.h"

#include <cmath>

#if defined(ARDUINO)
#include <Arduino.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
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

bool fresh(uint32_t timestampMs, uint32_t timeoutMs, uint32_t nowMs) {
  return nowMs - timestampMs <= timeoutMs;
}

bool fastInputsHealthy(const crawler::VelocityCommand& command,
                       const crawler::JointState& joints,
                       const crawler::ImuState& imu, bool bleConnected,
                       bool calibrationValid) {
  if (!calibrationValid || !bleConnected || !command.valid || !joints.valid ||
      !imu.valid) {
    return false;
  }
  const uint32_t now = monotonicMs();
  if (!fresh(command.receivedAtMs, crawler::config::safety::commandTimeoutMs,
             now) ||
      !fresh(joints.timestampMs, crawler::config::safety::sensorTimeoutMs,
             now) ||
      !fresh(imu.timestampMs, crawler::config::safety::sensorTimeoutMs, now)) {
    return false;
  }
  if (!std::isfinite(command.forwardMetersPerSecond) ||
      !std::isfinite(command.lateralMetersPerSecond)) {
    return false;
  }
  if (command.mode > crawler::ControlMode::ScriptedSweep) return false;
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    if (!std::isfinite(joints.positionRad[i]) ||
        !std::isfinite(joints.velocityRadPerSecond[i]) ||
        !std::isfinite(command.rawPositionRad[i])) {
      return false;
    }
    if (calibrationValid) {
      const crawler::ServoCalibration& calibration =
          crawler::config::servo::calibrations[i];
      if (joints.positionRad[i] < calibration.jointMinimumRad ||
          joints.positionRad[i] > calibration.jointMaximumRad) {
        return false;
      }
    }
  }
  for (uint8_t i = 0; i < 3; ++i) {
    if (!std::isfinite(imu.linearAccelerationMps2[i]) ||
        !std::isfinite(imu.angularVelocityRadPerSecond[i])) {
      return false;
    }
  }
  return true;
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
      imu_(),
      robotIo_(),
      policy_(),
      safety_(),
      policyInferenceActive_(false),
      policyControlActive_(false),
      wifiTelemetry_(&policyInferenceActive_, &policyControlActive_),
      initialized_(false),
      policyNeedsInitialization_(false),
      nextCycleUs_(0),
      cycleCount_(0),
      missedDeadlines_(0),
      maximumInferenceUs_(0),
      calibrationRequestConsumed_(false),
      latestJoints_{},
      latestImu_{},
      latestImuDiagnostics_{},
      latestCommand_{},
      latestResult_{}
#if defined(ARDUINO)
      , safetyTaskHandle_(nullptr)
#endif
{}

uint64_t Crawler::nowUs() const { return monotonicUs(); }

bool Crawler::begin() {
  safety_.begin();
  initialized_ = false;
  if (!imu_.begin()) {
    safety_.raiseFault(crawler::FaultCode::SensorInvalid);
    return false;
  }
  // Keep the sensor's known-good standalone startup isolated from the
  // diagnostic networking tasks. Wi-Fi starts only after IMU initialization.
  wifiTelemetry_.begin();
  if (!ble_.begin() || !robotIo_.begin()) return false;
  const bool calibrationReady =
      robotIo_.usingMockHardware() || robotIo_.calibrationValid();
  if (!robotIo_.usingMockHardware() && !calibrationReady) {
    // An uncalibrated hardware build must still boot so the operator can
    // enter calibration from the latched E-stop.
    safety_.raiseFault(crawler::FaultCode::EmergencyStopRequested);
  }
  if (!policy_.begin()) {
    safety_.raiseFault(crawler::FaultCode::PolicyLoadFailure);
    return false;
  }

  latestCommand_ = ble_.latestCommand();
  latestJoints_ = robotIo_.readJointState();
  latestImu_ = imu_.read();
  latestImuDiagnostics_ = imu_.diagnostics();
  if (!latestCommand_.valid) {
#if CRAWLER_USE_MOCK_HARDWARE
    latestCommand_.valid = true;
#endif
  }
  policyNeedsInitialization_ = true;
  if (latestJoints_.valid && latestImu_.valid && latestCommand_.valid) {
    if (!policy_.initialize(latestJoints_, latestImu_, latestCommand_)) {
      safety_.raiseFault(crawler::FaultCode::SensorInvalid);
    } else {
      policyNeedsInitialization_ = false;
    }
  }
  robotIo_.setMotionGate(false);
  robotIo_.setFastSafetyGate(false);
#if defined(ARDUINO)
  if (xTaskCreatePinnedToCore(safetyTaskEntry, "crawler_safety", 4096, this,
                              configMAX_PRIORITIES - 2, &safetyTaskHandle_,
                              0) != pdPASS) {
    safety_.raiseFault(crawler::FaultCode::SensorInvalid);
    return false;
  }
#endif
  nextCycleUs_ = nowUs();
  initialized_ = true;
#if defined(ARDUINO)
  Serial0.printf("Crawler ready: mock=%u calibration=%u BLE=%u state=%u\n",
                 robotIo_.usingMockHardware() ? 1u : 0u,
                 robotIo_.calibrationValid() ? 1u : 0u,
                 ble_.connected() ? 1u : 0u,
                 static_cast<unsigned>(safety_.state()));
  Serial0.printf("Inference deadline: %lu us, late results are skipped\n",
                 static_cast<unsigned long>(crawler::config::policy::deadlineUs));
#endif
  return true;
}

void Crawler::update() {
  if (!initialized_) {
    refreshTelemetryInputs();
    publishTelemetrySnapshot();
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
  refreshTelemetryInputs();
  const bool calibrationReady =
      robotIo_.usingMockHardware() || robotIo_.calibrationValid();
  const crawler::SafetyDecision decision =
      safety_.evaluate(latestCommand_, latestJoints_, latestImu_,
                       ble_.connected(), calibrationReady);

  if (!latestCommand_.calibrationRequested) {
    calibrationRequestConsumed_ = false;
  }
  if (robotIo_.calibrationActive() &&
      (latestCommand_.emergencyStop || !ble_.connected())) {
    robotIo_.abortCalibration();
  }
  if (safety_.state() == crawler::RobotState::EmergencyStopped &&
      latestCommand_.calibrationRequested && !latestCommand_.emergencyStop &&
      !calibrationRequestConsumed_ &&
      !robotIo_.calibrationActive()) {
    calibrationRequestConsumed_ = true;
    if (!robotIo_.startCalibration(latestJoints_)) {
#if defined(ARDUINO)
      Serial0.println("Calibration request rejected: hardware is not configured");
#endif
    }
  }
  if (robotIo_.calibrationActive()) {
    wifiTelemetry_.setPolicyControlActive(false);
    robotIo_.setMotionGate(false);
    robotIo_.setFastSafetyGate(false);
    policyNeedsInitialization_ = true;
    latestResult_ = {};
    robotIo_.updateCalibration();
#if defined(ARDUINO)
    if (robotIo_.calibrationState() == ServoCalibrationState::Completed) {
      Serial0.println("Servo calibration saved to NVS and activated");
    }
#endif
    const crawler::RobotStatus status = {
        safety_.state(), safety_.fault(), ble_.connected(),
        latestCommand_.sequence, 0, missedDeadlines_};
    ble_.publishStatus(status);
    printTelemetry();
    publishTelemetrySnapshot();
    return;
  }

  const bool emergencyHoldActive =
      latestCommand_.emergencyStop && ble_.connected() &&
      monotonicMs() - latestCommand_.receivedAtMs <=
          crawler::config::safety::commandTimeoutMs;
  if (emergencyHoldActive) {
    wifiTelemetry_.setPolicyControlActive(false);
    robotIo_.setMotionGate(true);
    if (!policyNeedsInitialization_) {
      // Stop the ONNX pipeline as part of E-stop. Its history and previous
      // filtered action must not resume from the pre-stop motion state.
      policy_.reset();
      policyNeedsInitialization_ = true;
    }
    latestResult_ = {};
    // Keep the measured position commanded while the emergency-stop packet is
    // held. This freezes the current pose instead of dropping servo output.
    robotIo_.holdCurrentPosition(latestJoints_);
    const crawler::RobotStatus status = {
        safety_.state(), safety_.fault(), ble_.connected(),
        latestCommand_.sequence, 0, missedDeadlines_};
    ble_.publishStatus(status);
    printTelemetry();
    publishTelemetrySnapshot();
    return;
  }

  if (!decision.allowMotion) {
    wifiTelemetry_.setPolicyControlActive(false);
    robotIo_.setMotionGate(false);
    robotIo_.disableServos();
    const crawler::RobotStatus status = {
        safety_.state(), safety_.fault(), ble_.connected(),
        latestCommand_.sequence, 0, missedDeadlines_};
    ble_.publishStatus(status);
    printTelemetry();
    publishTelemetrySnapshot();
    return;
  }

  if (latestCommand_.mode == crawler::ControlMode::RawPosition) {
    wifiTelemetry_.setPolicyControlActive(false);
    robotIo_.setMotionGate(true);
    float targetRad[crawler::kJointCount] = {};
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      if (latestCommand_.centerPositionRequested) {
        constexpr float degreesToRadians = 0.017453292519943295f;
        const crawler::ServoCalibration& calibration =
            crawler::config::servo::calibrations[i];
        targetRad[i] =
            (90.0f - calibration.jointZeroServoDegrees) * degreesToRadians /
            calibration.directionSign;
      } else {
        targetRad[i] = std::fmax(
            -crawler::config::policy::positionClampRad,
            std::fmin(crawler::config::policy::positionClampRad,
                      latestCommand_.rawPositionRad[i]));
      }
      targetRad[i] = std::fmax(
          -crawler::config::policy::positionClampRad,
          std::fmin(crawler::config::policy::positionClampRad, targetRad[i]));
      latestResult_.rawActions[i] = targetRad[i];
      latestResult_.clampedActions[i] = targetRad[i];
      latestResult_.targetRad[i] = targetRad[i];
      latestResult_.filteredTargetRad[i] = targetRad[i];
    }
    latestResult_.inferenceTimeUs = 0;
    latestResult_.valid = true;
    robotIo_.writeTargets(targetRad);
    const crawler::RobotStatus status = {
        safety_.state(), safety_.fault(), ble_.connected(),
        latestCommand_.sequence, 0, missedDeadlines_};
    ble_.publishStatus(status);
    printTelemetry();
    publishTelemetrySnapshot();
    return;
  }

  if (latestCommand_.mode == crawler::ControlMode::ScriptedSweep) {
    wifiTelemetry_.setPolicyControlActive(false);
    robotIo_.setMotionGate(true);
    float targetRad[crawler::kJointCount] = {};
    const float phase = static_cast<float>(monotonicMs() % 6000u) / 6000.0f;
    const float sweep = phase < 0.5f ? -1.0f + phase * 4.0f
                                    : 3.0f - phase * 4.0f;
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      float minimum = -0.5f;
      float maximum = 0.5f;
      if (crawler::config::servo::calibrationValid()) {
        const crawler::ServoCalibration& calibration =
            crawler::config::servo::calibrations[i];
        minimum = calibration.jointMinimumRad - calibration.defaultPositionRad;
        maximum = calibration.jointMaximumRad - calibration.defaultPositionRad;
      }
      targetRad[i] = minimum + (sweep + 1.0f) * 0.5f * (maximum - minimum);
      latestResult_.rawActions[i] = targetRad[i];
      latestResult_.clampedActions[i] = targetRad[i];
      latestResult_.targetRad[i] = targetRad[i];
      latestResult_.filteredTargetRad[i] = targetRad[i];
    }
    latestResult_.inferenceTimeUs = 0;
    latestResult_.valid = true;
    robotIo_.writeTargets(targetRad);
    const crawler::RobotStatus status = {
        safety_.state(), safety_.fault(), ble_.connected(),
        latestCommand_.sequence, 0, missedDeadlines_};
    ble_.publishStatus(status);
    printTelemetry();
    publishTelemetrySnapshot();
    return;
  }

  if (policyNeedsInitialization_ &&
      !policy_.initialize(latestJoints_, latestImu_, latestCommand_)) {
    const crawler::FaultCode failure = policy_.failureCode();
    safety_.raiseFault(failure == crawler::FaultCode::None
                           ? crawler::FaultCode::SensorInvalid
                           : failure);
    robotIo_.setMotionGate(false);
    robotIo_.disableServos();
    wifiTelemetry_.setPolicyControlActive(false);
    printTelemetry();
    publishTelemetrySnapshot();
    return;
  }
  policyNeedsInitialization_ = false;

  crawler::PolicyResult result = {};
  wifiTelemetry_.setPolicyControlActive(true);
  policyInferenceActive_.store(true, std::memory_order_release);
  const bool policyStepSucceeded =
      policy_.step(latestJoints_, latestImu_, latestCommand_, result);
  policyInferenceActive_.store(false, std::memory_order_release);
  if (!policyStepSucceeded || !result.valid) {
    const crawler::FaultCode failure = policy_.failureCode();
    safety_.raiseFault(failure == crawler::FaultCode::None
                           ? crawler::FaultCode::NonFinitePolicyOutput
                           : failure);
    robotIo_.setMotionGate(false);
    robotIo_.disableServos();
    printTelemetry();
    publishTelemetrySnapshot();
    completePolicyNetworkWindow();
    wifiTelemetry_.setPolicyControlActive(false);
    return;
  }
  latestResult_ = result;
  if (result.inferenceTimeUs > maximumInferenceUs_) {
    maximumInferenceUs_ = result.inferenceTimeUs;
  }
  if (result.inferenceTimeUs > crawler::config::policy::deadlineUs) {
    ++missedDeadlines_;
    // Do not apply a late policy action. Hold the measured pose for this
    // cycle, and leave the policy history unchanged so the late action is
    // not fed into the next observation.
    latestResult_ = {};
    latestResult_.inferenceTimeUs = result.inferenceTimeUs;
    for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
      const float holdTarget =
          latestJoints_.positionRad[i] -
          crawler::config::servo::calibrations[i].defaultPositionRad;
      latestResult_.targetRad[i] = holdTarget;
      latestResult_.filteredTargetRad[i] = holdTarget;
    }
    latestResult_.valid = false;
    wifiTelemetry_.setPolicyControlActive(false);
    robotIo_.setMotionGate(true);
    robotIo_.holdCurrentPosition(latestJoints_);
  } else {
    policy_.commitAction(result);
    latestResult_ = result;
    robotIo_.setMotionGate(true);
    robotIo_.writeTargets(result.filteredTargetRad);
  }

  const crawler::RobotStatus status = {
      safety_.state(), safety_.fault(), ble_.connected(), latestCommand_.sequence,
      result.inferenceTimeUs, missedDeadlines_};
  ble_.publishStatus(status);
  printTelemetry();
  publishTelemetrySnapshot();
  completePolicyNetworkWindow();
}

void Crawler::completePolicyNetworkWindow() {
  const uint64_t now = nowUs();
  uint64_t availableUs = crawler::config::policy::periodUs;
  if (nextCycleUs_ > now) {
    const uint64_t remainingUs = nextCycleUs_ - now;
    if (remainingUs < availableUs) availableUs = remainingUs;
  }
  wifiTelemetry_.runPostInferenceWindow(
      availableUs > UINT32_MAX ? UINT32_MAX
                               : static_cast<uint32_t>(availableUs));
}

void Crawler::refreshTelemetryInputs() {
  latestCommand_ = ble_.latestCommand();
  latestJoints_ = robotIo_.readJointState();
  latestImu_ = imu_.read();
  latestImuDiagnostics_ = imu_.diagnostics();
}

void Crawler::publishTelemetrySnapshot() {
  TelemetrySnapshot snapshot = {};
  const uint32_t now = monotonicMs();
  snapshot.timestampMs = now;
  snapshot.uptimeMs = now;
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    snapshot.jointPositionRad[i] = latestJoints_.positionRad[i];
    snapshot.jointVelocityRadPerSecond[i] =
        latestJoints_.velocityRadPerSecond[i];
    snapshot.jointTargetRad[i] = latestResult_.filteredTargetRad[i];
    snapshot.policyActionNormalized[i] =
        latestCommand_.mode == crawler::ControlMode::Policy
            ? latestResult_.clampedActions[i]
            : 0.0f;
  }
  for (uint8_t i = 0; i < 3; ++i) {
    snapshot.accelerationMps2[i] = latestImu_.linearAccelerationMps2[i];
    snapshot.gyroRadPerSecond[i] =
        latestImu_.angularVelocityRadPerSecond[i];
  }
  snapshot.velocityCommandMps[0] = latestCommand_.forwardMetersPerSecond;
  snapshot.velocityCommandMps[1] = latestCommand_.lateralMetersPerSecond;
  snapshot.inferenceTimeUs = latestResult_.inferenceTimeUs;
  snapshot.maximumInferenceTimeUs = maximumInferenceUs_;
  snapshot.policyDeadlineMisses = missedDeadlines_;
  snapshot.policyRateHz = static_cast<float>(crawler::config::policy::rateHz);
  // Keep the legacy single-rate field for existing dashboard consumers. The
  // separate fields below are the authoritative IMU configuration/metrics.
  snapshot.imuConfiguredHz = crawler::config::imu::gyroRequestedHz;
  snapshot.imuAccelRequestedHz = latestImuDiagnostics_.accelRequestedHz;
  snapshot.imuAccelMeasuredHz = latestImuDiagnostics_.accelMeasuredHz;
  snapshot.imuAccelAgeUs = latestImuDiagnostics_.accelAgeUs;
  snapshot.imuAccelValid = latestImuDiagnostics_.accelValid;
  snapshot.imuAccelAccuracy = latestImuDiagnostics_.accelAccuracy;
  snapshot.imuAccelSequenceGaps = latestImuDiagnostics_.accelSequenceGaps;
  snapshot.imuAccelTimestampBacksteps =
      latestImuDiagnostics_.accelTimestampBacksteps;
  snapshot.imuGyroRequestedHz = latestImuDiagnostics_.gyroRequestedHz;
  snapshot.imuGyroMeasuredHz = latestImuDiagnostics_.gyroMeasuredHz;
  snapshot.imuGyroAgeUs = latestImuDiagnostics_.gyroAgeUs;
  snapshot.imuGyroValid = latestImuDiagnostics_.gyroValid;
  snapshot.imuGyroAccuracy = latestImuDiagnostics_.gyroAccuracy;
  snapshot.imuGyroSequenceGaps = latestImuDiagnostics_.gyroSequenceGaps;
  snapshot.imuGyroTimestampBacksteps =
      latestImuDiagnostics_.gyroTimestampBacksteps;
  snapshot.imuResetRecoveryActive =
      latestImuDiagnostics_.resetRecoveryActive;
  snapshot.imuResetCount = latestImuDiagnostics_.resetCount;
  snapshot.imuResetGeneration = latestImuDiagnostics_.resetGeneration;
  snapshot.imuLastDrainUs = latestImuDiagnostics_.lastDrainUs;
  snapshot.imuMaxDrainUs = latestImuDiagnostics_.maxDrainUs;
  snapshot.imuLastEventsPerDrain = latestImuDiagnostics_.lastEventsPerDrain;
  snapshot.imuMaxEventsPerDrain = latestImuDiagnostics_.maxEventsPerDrain;
  snapshot.imuDrainBudgetHits = latestImuDiagnostics_.drainBudgetHits;
  snapshot.imuEventLimitHits = latestImuDiagnostics_.eventLimitHits;
  snapshot.imuTransactionFailures =
      latestImuDiagnostics_.transactionFailures;
  snapshot.imuRecoveryAttempts = latestImuDiagnostics_.recoveryAttempts;
  snapshot.imuRecoveryFailures = latestImuDiagnostics_.recoveryFailures;
  snapshot.imuLastFailureStage = latestImuDiagnostics_.lastFailureStage;
  snapshot.jointConfiguredHz = crawler::config::joints::configuredRateHz;
  snapshot.faultCode = safety_.fault();
  snapshot.robotState = safety_.state();
  snapshot.servosEnabled = robotIo_.servosEnabled();
  snapshot.bleCommandAgeMs = latestCommand_.valid
                                 ? now - latestCommand_.receivedAtMs
                                 : 0u;
  wifiTelemetry_.publish(snapshot);
}

#if defined(ARDUINO)
void Crawler::safetyTaskEntry(void* argument) {
  static_cast<Crawler*>(argument)->safetyTaskLoop();
}

void Crawler::safetyTaskLoop() {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1);
  for (;;) {
    const crawler::VelocityCommand command = ble_.latestCommand();
    const crawler::JointState joints = robotIo_.readJointState();
    const crawler::ImuState imu = imu_.read();
    const bool healthy = fastInputsHealthy(
        command, joints, imu, ble_.connected(),
        robotIo_.usingMockHardware() || robotIo_.calibrationValid());
    // The main loop keeps the emergency pose hold active. This fast gate
    // still closes immediately for stale, invalid, or disconnected inputs.
    robotIo_.setFastSafetyGate(healthy);
    vTaskDelayUntil(&lastWake, period == 0 ? 1 : period);
  }
}
#endif

void Crawler::printTelemetry() {
#if defined(ARDUINO)
  if (cycleCount_ % crawler::config::telemetry::printEveryCycles != 0u) return;
  const WifiTelemetryMetrics wifiMetrics = wifiTelemetry_.metrics();
#if CRAWLER_WIFI_USE_IDF_HTTPD
  Serial0.printf(
      "TEL t=%lu state=%s fault=%u ble=%u cmd=%.3f,%.3f enable=%u "
      "joint=%.4f,%.4f,%.4f vel=%.4f,%.4f,%.4f raw=%.4f,%.4f,%.4f "
      "filtered=%.4f,%.4f,%.4f infer_us=%lu max_infer_us=%lu missed=%lu "
      "imu=%u accel_valid=%u accel_age_us=%lu gyro_valid=%u gyro_age_us=%lu "
      "drain_us=%lu drain_max_us=%lu events=%lu budget_hits=%lu "
      "event_limit_hits=%lu tx_fail=%lu recovery=%lu/%lu stage=%u "
      "heap_internal_free=%lu heap_internal_min=%lu psram_free=%lu "
      "psram_min=%lu json_us=%lu json_max_us=%lu ws_us=%lu "
      "ws_max_us=%lu drops=%lu ws_failures=%lu ws_connected=%lu "
      "ws_connects=%lu ws_disconnects=%lu ws_generation=%lu "
      "httpd_queue_failures=%lu httpd_pending=%lu pending_age_us=%lu "
      "pending_age_max_us=%lu close_failures=%lu "
      "network_health_degraded=%lu pending_age_exceeded=%lu\n",
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
      latestImu_.valid ? 1u : 0u,
      latestImuDiagnostics_.accelValid ? 1u : 0u,
      static_cast<unsigned long>(latestImuDiagnostics_.accelAgeUs),
      latestImuDiagnostics_.gyroValid ? 1u : 0u,
      static_cast<unsigned long>(latestImuDiagnostics_.gyroAgeUs),
      static_cast<unsigned long>(latestImuDiagnostics_.lastDrainUs),
      static_cast<unsigned long>(latestImuDiagnostics_.maxDrainUs),
      static_cast<unsigned long>(latestImuDiagnostics_.lastEventsPerDrain),
      static_cast<unsigned long>(latestImuDiagnostics_.drainBudgetHits),
      static_cast<unsigned long>(latestImuDiagnostics_.eventLimitHits),
      static_cast<unsigned long>(latestImuDiagnostics_.transactionFailures),
      static_cast<unsigned long>(latestImuDiagnostics_.recoveryAttempts),
      static_cast<unsigned long>(latestImuDiagnostics_.recoveryFailures),
      static_cast<unsigned>(latestImuDiagnostics_.lastFailureStage),
      static_cast<unsigned long>(heap_caps_get_free_size(
          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
      static_cast<unsigned long>(heap_caps_get_minimum_free_size(
          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
      static_cast<unsigned long>(heap_caps_get_free_size(
          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)),
      static_cast<unsigned long>(heap_caps_get_minimum_free_size(
          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)),
      static_cast<unsigned long>(wifiMetrics.jsonEncodeUs),
      static_cast<unsigned long>(wifiMetrics.jsonEncodeMaxUs),
      static_cast<unsigned long>(wifiMetrics.websocketSendUs),
      static_cast<unsigned long>(wifiMetrics.websocketSendMaxUs),
      static_cast<unsigned long>(wifiMetrics.telemetryDrops),
      static_cast<unsigned long>(wifiMetrics.websocketSendFailures),
      static_cast<unsigned long>(wifiMetrics.websocketConnected),
      static_cast<unsigned long>(wifiMetrics.websocketConnectCount),
      static_cast<unsigned long>(wifiMetrics.websocketDisconnectCount),
      static_cast<unsigned long>(wifiMetrics.websocketGeneration),
      static_cast<unsigned long>(wifiMetrics.httpdQueueFailures),
      static_cast<unsigned long>(wifiMetrics.httpdPendingSends),
      static_cast<unsigned long>(wifiMetrics.httpdPendingAgeUs),
      static_cast<unsigned long>(wifiMetrics.httpdPendingAgeMaxUs),
      static_cast<unsigned long>(wifiMetrics.sessionCloseFailures),
      static_cast<unsigned long>(wifiMetrics.networkHealthDegraded),
      static_cast<unsigned long>(wifiMetrics.pendingAgeExceededCount));
#else
  Serial0.printf(
      "TEL t=%lu state=%s fault=%u ble=%u cmd=%.3f,%.3f enable=%u "
      "joint=%.4f,%.4f,%.4f vel=%.4f,%.4f,%.4f raw=%.4f,%.4f,%.4f "
      "filtered=%.4f,%.4f,%.4f infer_us=%lu max_infer_us=%lu missed=%lu "
      "imu=%u accel_valid=%u accel_age_us=%lu gyro_valid=%u gyro_age_us=%lu "
      "drain_us=%lu drain_max_us=%lu events=%lu budget_hits=%lu "
      "event_limit_hits=%lu tx_fail=%lu recovery=%lu/%lu stage=%u "
      "heap=%lu net_us=%lu net_max_us=%lu json_us=%lu json_max_us=%lu "
      "ws_us=%lu ws_max_us=%lu win_overruns=%lu drops=%lu\n",
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
      latestImu_.valid ? 1u : 0u,
      latestImuDiagnostics_.accelValid ? 1u : 0u,
      static_cast<unsigned long>(latestImuDiagnostics_.accelAgeUs),
      latestImuDiagnostics_.gyroValid ? 1u : 0u,
      static_cast<unsigned long>(latestImuDiagnostics_.gyroAgeUs),
      static_cast<unsigned long>(latestImuDiagnostics_.lastDrainUs),
      static_cast<unsigned long>(latestImuDiagnostics_.maxDrainUs),
      static_cast<unsigned long>(latestImuDiagnostics_.lastEventsPerDrain),
      static_cast<unsigned long>(latestImuDiagnostics_.drainBudgetHits),
      static_cast<unsigned long>(latestImuDiagnostics_.eventLimitHits),
      static_cast<unsigned long>(latestImuDiagnostics_.transactionFailures),
      static_cast<unsigned long>(latestImuDiagnostics_.recoveryAttempts),
      static_cast<unsigned long>(latestImuDiagnostics_.recoveryFailures),
      static_cast<unsigned>(latestImuDiagnostics_.lastFailureStage),
      static_cast<unsigned long>(ESP.getFreeHeap()),
      static_cast<unsigned long>(wifiMetrics.networkServiceUs),
      static_cast<unsigned long>(wifiMetrics.networkServiceMaxUs),
      static_cast<unsigned long>(wifiMetrics.jsonEncodeUs),
      static_cast<unsigned long>(wifiMetrics.jsonEncodeMaxUs),
      static_cast<unsigned long>(wifiMetrics.websocketSendUs),
      static_cast<unsigned long>(wifiMetrics.websocketSendMaxUs),
      static_cast<unsigned long>(wifiMetrics.networkWindowOverruns),
      static_cast<unsigned long>(wifiMetrics.telemetryDrops));
#endif
#endif
}
