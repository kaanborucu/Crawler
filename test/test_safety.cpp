#include <chrono>
#include <cmath>
#include <cstdio>

#include "safety.h"

namespace safety_tests {

uint32_t nowMs() {
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now.time_since_epoch())
                                  .count());
}

void expect(bool condition, const char* message, int& failures) {
  if (!condition) {
    std::fprintf(stderr, "safety: %s\n", message);
    ++failures;
  }
}

crawler::JointState validJoints(uint32_t timestamp) {
  crawler::JointState joints = {};
  joints.valid = true;
  joints.timestampMs = timestamp;
  return joints;
}

crawler::VelocityCommand validCommand(uint16_t sequence, uint32_t timestamp,
                                       bool enable) {
  crawler::VelocityCommand command = {};
  command.valid = true;
  command.sequence = sequence;
  command.receivedAtMs = timestamp;
  command.enableRequested = enable;
  return command;
}

}  // namespace safety_tests

int runSafetyTests() {
  using namespace safety_tests;
  int failures = 0;
  const uint32_t timestamp = nowMs();
  const crawler::JointState joints = validJoints(timestamp);

  Safety safety;
  safety.begin();
  crawler::VelocityCommand command = validCommand(1, timestamp, true);
  crawler::SafetyDecision decision = safety.evaluate(command, joints, true, true);
  expect(decision.allowMotion && safety.state() == crawler::RobotState::Running,
         "valid enable did not enter running state", failures);

  command.enableRequested = false;
  decision = safety.evaluate(command, joints, true, true);
  expect(!decision.allowMotion && safety.state() == crawler::RobotState::Disarmed,
         "disable command did not disarm", failures);

  Safety invalidCalibration;
  invalidCalibration.begin();
  command = validCommand(2, timestamp, true);
  decision = invalidCalibration.evaluate(command, joints, true, false);
  expect(!decision.allowMotion &&
             invalidCalibration.fault() == crawler::FaultCode::InvalidCalibration,
         "invalid calibration did not fault safely", failures);
  command.clearFaultRequested = true;
  command.sequence = 3;
  decision = invalidCalibration.evaluate(command, joints, true, false);
  expect(!decision.allowMotion && invalidCalibration.faultActive(),
         "fault cleared while its cause remained", failures);
  command.clearFaultRequested = true;
  command.sequence = 4;
  decision = invalidCalibration.evaluate(command, joints, true, true);
  expect(!decision.allowMotion &&
             invalidCalibration.state() == crawler::RobotState::Disarmed,
         "fault clear did not require a disarmed recovery step", failures);
  command.clearFaultRequested = false;
  command.sequence = 5;
  decision = invalidCalibration.evaluate(command, joints, true, true);
  expect(decision.allowMotion, "new enable sequence did not re-arm", failures);

  Safety sensorSafety;
  sensorSafety.begin();
  crawler::JointState invalidSensor = joints;
  invalidSensor.valid = false;
  decision = sensorSafety.evaluate(validCommand(6, timestamp, true),
                                   invalidSensor, true, true);
  expect(sensorSafety.fault() == crawler::FaultCode::SensorInvalid &&
             !decision.allowMotion,
         "invalid sensor did not fault safely", failures);

  Safety nonfiniteSafety;
  nonfiniteSafety.begin();
  crawler::JointState nonfiniteJoints = joints;
  nonfiniteJoints.positionRad[0] = NAN;
  decision = nonfiniteSafety.evaluate(validCommand(7, timestamp, true),
                                      nonfiniteJoints, true, true);
  expect(nonfiniteSafety.fault() == crawler::FaultCode::NonFiniteObservation &&
             !decision.allowMotion,
         "non-finite observation did not fault safely", failures);

  Safety timeoutSafety;
  timeoutSafety.begin();
  command = validCommand(8, timestamp - 1000u, true);
  decision = timeoutSafety.evaluate(command, joints, true, true);
  expect(timeoutSafety.fault() == crawler::FaultCode::CommandTimeout &&
             !decision.allowMotion,
         "stale command did not fault safely", failures);

  Safety bleSafety;
  bleSafety.begin();
  decision = bleSafety.evaluate(validCommand(9, timestamp, true), joints, false,
                                true);
  expect(bleSafety.fault() == crawler::FaultCode::BleDisconnected &&
             !decision.allowMotion,
         "BLE disconnect did not fault safely", failures);

  Safety activeFault;
  activeFault.begin();
  activeFault.raiseFault(crawler::FaultCode::NonFinitePolicyOutput);
  command = validCommand(10, timestamp, true);
  decision = activeFault.evaluate(command, joints, true, true);
  expect(activeFault.fault() == crawler::FaultCode::NonFinitePolicyOutput &&
             !decision.allowMotion,
         "active policy fault was bypassed", failures);
  command.clearFaultRequested = true;
  command.sequence = 11;
  decision = activeFault.evaluate(command, joints, true, true);
  expect(!decision.allowMotion &&
             activeFault.state() == crawler::RobotState::Disarmed,
         "clearing a fault automatically restarted motion", failures);

  Safety stopSafety;
  stopSafety.begin();
  command = validCommand(9, timestamp, true);
  command.emergencyStop = true;
  decision = stopSafety.evaluate(command, joints, true, true);
  expect(stopSafety.state() == crawler::RobotState::EmergencyStopped &&
             !decision.allowMotion,
         "emergency stop did not latch", failures);
  return failures;
}
