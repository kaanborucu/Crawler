#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

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

crawler::ImuState validImu(uint32_t timestamp) {
  crawler::ImuState imu = {};
  imu.valid = true;
  imu.timestampMs = timestamp;
  return imu;
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
  const crawler::ImuState imu = validImu(timestamp);

  Safety safety;
  safety.begin();
  crawler::VelocityCommand command = validCommand(1, timestamp, true);
  crawler::SafetyDecision decision =
      safety.evaluate(command, joints, imu, true, true);
  expect(decision.allowMotion && safety.state() == crawler::RobotState::Running,
         "valid enable did not enter running without network input", failures);

  command.enableRequested = false;
  decision = safety.evaluate(command, joints, imu, true, true);
  expect(!decision.allowMotion && safety.state() == crawler::RobotState::Disarmed,
         "disable command did not disarm", failures);

  Safety movingSafety;
  movingSafety.begin();
  command = validCommand(20, timestamp, true);
  decision = movingSafety.evaluate(command, joints, imu, true, true);
  expect(decision.allowMotion, "moving safety test did not start running",
         failures);
  crawler::JointState movedJoints = joints;
  movedJoints.positionRad[0] = 0.5f;
  movedJoints.timestampMs = nowMs();
  command.sequence = 21;
  command.receivedAtMs = nowMs();
  decision = movingSafety.evaluate(command, movedJoints, imu, true, true);
  expect(decision.allowMotion &&
             movingSafety.state() == crawler::RobotState::Running,
         "running robot was disarmed after moving away from neutral", failures);

  Safety centerSafety;
  centerSafety.begin();
  crawler::VelocityCommand centerCommand = validCommand(22, nowMs(), true);
  centerCommand.mode = crawler::ControlMode::RawPosition;
  centerCommand.centerPositionRequested = true;
  decision = centerSafety.evaluate(centerCommand, movedJoints, imu, true, true);
  expect(decision.allowMotion &&
             centerSafety.state() == crawler::RobotState::Running,
         "center-position recovery was blocked away from neutral", failures);

  Safety invalidCalibration;
  invalidCalibration.begin();
  command = validCommand(2, timestamp, true);
  decision = invalidCalibration.evaluate(command, joints, imu, true, false);
  expect(!decision.allowMotion &&
             invalidCalibration.fault() == crawler::FaultCode::InvalidCalibration,
         "invalid calibration did not fault safely", failures);
  command.clearFaultRequested = true;
  command.sequence = 3;
  decision = invalidCalibration.evaluate(command, joints, imu, true, false);
  expect(!decision.allowMotion && invalidCalibration.faultActive(),
         "fault cleared while its cause remained", failures);
  command.clearFaultRequested = true;
  command.sequence = 4;
  decision = invalidCalibration.evaluate(command, joints, imu, true, true);
  expect(!decision.allowMotion &&
             invalidCalibration.state() == crawler::RobotState::Disarmed,
         "fault clear did not require a disarmed recovery step", failures);
  command.clearFaultRequested = false;
  command.sequence = 5;
  decision = invalidCalibration.evaluate(command, joints, imu, true, true);
  expect(decision.allowMotion, "new enable sequence did not re-arm", failures);

  Safety sensorSafety;
  sensorSafety.begin();
  crawler::JointState invalidSensor = joints;
  invalidSensor.valid = false;
  decision = sensorSafety.evaluate(validCommand(6, timestamp, true),
                                   invalidSensor, imu, true, true);
  expect(sensorSafety.fault() == crawler::FaultCode::SensorInvalid &&
             !decision.allowMotion,
         "invalid sensor did not fault safely", failures);

  Safety temporaryImu;
  temporaryImu.begin();
  crawler::ImuState invalidImu = imu;
  invalidImu.valid = false;
  decision = temporaryImu.evaluate(validCommand(12, timestamp, true), joints,
                                   invalidImu, true, true);
  expect(!decision.allowMotion &&
             temporaryImu.fault() == crawler::FaultCode::None &&
             temporaryImu.state() == crawler::RobotState::Disarmed,
         "temporary invalid IMU did not disarm without latching", failures);
  decision = temporaryImu.evaluate(validCommand(13, timestamp, true), joints,
                                   imu, true, true);
  expect(decision.allowMotion && temporaryImu.fault() == crawler::FaultCode::None,
         "recovered IMU did not clear the temporary invalid timer", failures);

  Safety nonfiniteSafety;
  nonfiniteSafety.begin();
  crawler::JointState nonfiniteJoints = joints;
  nonfiniteJoints.positionRad[0] = NAN;
  decision = nonfiniteSafety.evaluate(validCommand(7, timestamp, true),
                                      nonfiniteJoints, imu, true, true);
  expect(nonfiniteSafety.fault() == crawler::FaultCode::NonFiniteObservation &&
             !decision.allowMotion,
         "non-finite observation did not fault safely", failures);

  Safety timeoutSafety;
  timeoutSafety.begin();
  command = validCommand(8, timestamp - 1000u, true);
  decision = timeoutSafety.evaluate(command, joints, imu, true, true);
  expect(timeoutSafety.fault() == crawler::FaultCode::CommandTimeout &&
             !decision.allowMotion,
         "stale command did not fault safely", failures);

  Safety bleSafety;
  bleSafety.begin();
  decision = bleSafety.evaluate(validCommand(9, timestamp, true), joints, imu,
                                false, true);
  expect(bleSafety.fault() == crawler::FaultCode::BleDisconnected &&
             !decision.allowMotion,
         "BLE disconnect did not fault safely", failures);

  Safety activeFault;
  activeFault.begin();
  activeFault.raiseFault(crawler::FaultCode::NonFinitePolicyOutput);
  command = validCommand(10, timestamp, true);
  decision = activeFault.evaluate(command, joints, imu, true, true);
  expect(activeFault.fault() == crawler::FaultCode::NonFinitePolicyOutput &&
             !decision.allowMotion,
         "active policy fault was bypassed", failures);
  command.clearFaultRequested = true;
  command.sequence = 11;
  decision = activeFault.evaluate(command, joints, imu, true, true);
  expect(!decision.allowMotion &&
             activeFault.state() == crawler::RobotState::Disarmed,
         "clearing a fault automatically restarted motion", failures);

  Safety stopSafety;
  stopSafety.begin();
  command = validCommand(9, timestamp, true);
  command.emergencyStop = true;
  decision = stopSafety.evaluate(command, joints, imu, true, true);
  expect(stopSafety.state() == crawler::RobotState::EmergencyStopped &&
             !decision.allowMotion,
         "emergency stop did not latch", failures);

  Safety persistentImu;
  persistentImu.begin();
  const uint32_t persistentTimestamp = nowMs();
  const crawler::JointState persistentJoints =
      validJoints(persistentTimestamp);
  const crawler::ImuState persistentInvalidImu = [&]() {
    crawler::ImuState value = validImu(persistentTimestamp);
    value.valid = false;
    return value;
  }();
  decision = persistentImu.evaluate(
      validCommand(14, persistentTimestamp, true), persistentJoints,
      persistentInvalidImu, true, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(310));
  decision = persistentImu.evaluate(
      validCommand(15, nowMs(), true), validJoints(nowMs()),
      persistentInvalidImu, true, true);
  expect(!decision.allowMotion &&
             persistentImu.fault() == crawler::FaultCode::SensorInvalid,
         "persistent invalid IMU did not fault after the existing timeout",
         failures);
  return failures;
}
