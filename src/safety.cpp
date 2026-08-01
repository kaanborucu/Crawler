#include "safety.h"

#include <cmath>

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <chrono>
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

bool finiteJointData(const crawler::JointState& joints) {
  if (!joints.valid) return false;
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    if (!std::isfinite(joints.positionRad[i]) ||
        !std::isfinite(joints.velocityRadPerSecond[i])) {
      return false;
    }
  }
  return true;
}

bool finiteCommandData(const crawler::VelocityCommand& command) {
  if (!command.valid) return true;
  if (!std::isfinite(command.forwardMetersPerSecond) ||
      !std::isfinite(command.lateralMetersPerSecond) ||
      command.mode > crawler::ControlMode::ScriptedSweep) {
    return false;
  }
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    if (!std::isfinite(command.rawPositionRad[i])) return false;
  }
  return true;
}

bool finiteImuData(const crawler::ImuState& imu) {
  if (!imu.valid) return false;
  for (uint8_t i = 0; i < 3; ++i) {
    if (!std::isfinite(imu.linearAccelerationMps2[i]) ||
        !std::isfinite(imu.angularVelocityRadPerSecond[i])) {
      return false;
    }
  }
  return true;
}

bool jointsWithinConfiguredLimits(const crawler::JointState& joints) {
  // Mock mode intentionally has no physical calibration, so there are no
  // meaningful limits to enforce there. Hardware mode is checked at startup.
  if (!crawler::config::servo::calibrationValid()) return true;
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    const crawler::ServoCalibration& calibration =
        crawler::config::servo::calibrations[i];
    if (joints.positionRad[i] < calibration.jointMinimumRad ||
        joints.positionRad[i] > calibration.jointMaximumRad) {
      return false;
    }
  }
  return true;
}

}  // namespace

Safety::Safety()
    : state_(crawler::RobotState::Booting),
      fault_(crawler::FaultCode::None),
      requireNextEnable_(false),
      clearedFaultSequence_(0),
      requireNewSequence_(false) {}

void Safety::begin() {
  state_ = crawler::RobotState::Disarmed;
  fault_ = crawler::FaultCode::None;
  requireNextEnable_ = false;
  requireNewSequence_ = false;
}

uint32_t Safety::nowMs() const { return monotonicMs(); }

void Safety::setFault(crawler::FaultCode fault) {
  fault_ = fault;
  state_ = fault == crawler::FaultCode::EmergencyStopRequested
               ? crawler::RobotState::EmergencyStopped
               : crawler::RobotState::Fault;
  requireNextEnable_ = true;
}

void Safety::raiseFault(crawler::FaultCode fault) { setFault(fault); }

bool Safety::jointsNearDefault(const crawler::JointState& joints) const {
  if (!joints.valid) return false;
  for (uint8_t i = 0; i < crawler::kJointCount; ++i) {
    if (!std::isfinite(joints.positionRad[i]) ||
        std::fabs(joints.positionRad[i] -
                  crawler::config::servo::calibrations[i].defaultPositionRad) >
            crawler::config::safety::armPositionToleranceRad) {
      return false;
    }
  }
  return true;
}

crawler::SafetyDecision Safety::evaluate(
    const crawler::VelocityCommand& command,
    const crawler::JointState& joints, const crawler::ImuState& imu,
    bool bleConnected, bool calibrationValid) {
  if (command.emergencyStop) setFault(crawler::FaultCode::EmergencyStopRequested);

  const bool causeGone =
      !command.emergencyStop && calibrationValid && finiteJointData(joints) &&
      finiteImuData(imu) &&
      bleConnected && command.valid && finiteCommandData(command) &&
      nowMs() - joints.timestampMs <= crawler::config::safety::sensorTimeoutMs &&
      nowMs() - imu.timestampMs <= crawler::config::safety::sensorTimeoutMs &&
      nowMs() - command.receivedAtMs <= crawler::config::safety::commandTimeoutMs;

  if (faultActive()) {
    if (command.clearFaultRequested && causeGone &&
        command.sequence != clearedFaultSequence_) {
      state_ = crawler::RobotState::Disarmed;
      fault_ = crawler::FaultCode::None;
      requireNextEnable_ = true;
      requireNewSequence_ = true;
      clearedFaultSequence_ = command.sequence;
    }
    return {false, state_, fault_};
  }

  if (!calibrationValid) {
    setFault(crawler::FaultCode::InvalidCalibration);
  } else if (!joints.valid) {
    setFault(crawler::FaultCode::SensorInvalid);
  } else if (!finiteJointData(joints)) {
    setFault(crawler::FaultCode::NonFiniteObservation);
  } else if (!imu.valid) {
    setFault(crawler::FaultCode::SensorInvalid);
  } else if (!finiteImuData(imu)) {
    setFault(crawler::FaultCode::NonFiniteObservation);
  } else if (!jointsWithinConfiguredLimits(joints)) {
    setFault(crawler::FaultCode::JointLimitViolation);
  } else if (nowMs() - joints.timestampMs >
             crawler::config::safety::sensorTimeoutMs) {
    setFault(crawler::FaultCode::SensorTimeout);
  } else if (nowMs() - imu.timestampMs >
             crawler::config::safety::sensorTimeoutMs) {
    setFault(crawler::FaultCode::SensorTimeout);
  } else if (!bleConnected) {
    setFault(crawler::FaultCode::BleDisconnected);
  } else if (!command.valid ||
             nowMs() - command.receivedAtMs >
                 crawler::config::safety::commandTimeoutMs) {
    setFault(crawler::FaultCode::CommandTimeout);
  } else if (!finiteCommandData(command)) {
    setFault(crawler::FaultCode::NonFiniteObservation);
  } else if (!command.enableRequested) {
    state_ = crawler::RobotState::Disarmed;
    fault_ = crawler::FaultCode::None;
  } else if (requireNewSequence_ && command.sequence == clearedFaultSequence_) {
    state_ = crawler::RobotState::Disarmed;
  } else if (requireNextEnable_ && !command.enableRequested) {
    state_ = crawler::RobotState::Disarmed;
  } else if (!jointsNearDefault(joints)) {
    state_ = crawler::RobotState::Disarmed;
  } else {
    state_ = crawler::RobotState::Running;
    fault_ = crawler::FaultCode::None;
    requireNextEnable_ = false;
    requireNewSequence_ = false;
  }

  return {state_ == crawler::RobotState::Running, state_, fault_};
}

crawler::RobotState Safety::state() const { return state_; }

crawler::FaultCode Safety::fault() const { return fault_; }

bool Safety::faultActive() const {
  return state_ == crawler::RobotState::Fault ||
         state_ == crawler::RobotState::EmergencyStopped;
}
