#pragma once

#include <stdint.h>

namespace crawler {

constexpr uint8_t kJointCount = 3;

enum class RobotState : uint8_t {
  Booting = 0,
  Disarmed = 1,
  Running = 2,
  Fault = 3,
  EmergencyStopped = 4,
};

enum class ControlMode : uint8_t {
  Policy = 0,
  RawPosition = 1,
  ScriptedSweep = 2,
};

enum class FaultCode : uint8_t {
  None = 0,
  InvalidCalibration = 1,
  SensorInvalid = 2,
  SensorTimeout = 3,
  CommandTimeout = 4,
  BleDisconnected = 5,
  NonFiniteObservation = 6,
  PolicyLoadFailure = 7,
  NonFinitePolicyOutput = 8,
  InferenceDeadlineMiss = 9,
  JointLimitViolation = 10,
  EmergencyStopRequested = 11,
};

struct JointState {
  float positionRad[kJointCount];
  float velocityRadPerSecond[kJointCount];
  bool valid;
  uint32_t timestampMs;
};

struct ImuState {
  float linearAccelerationMps2[3];
  float angularVelocityRadPerSecond[3];
  bool valid;
  uint32_t timestampMs;
};

struct VelocityCommand {
  float forwardMetersPerSecond;
  float lateralMetersPerSecond;
  float rawPositionRad[kJointCount];
  ControlMode mode;
  bool enableRequested;
  bool emergencyStop;
  bool clearFaultRequested;
  bool calibrationRequested;
  bool centerPositionRequested;
  bool valid;
  uint16_t sequence;
  uint32_t receivedAtMs;
};

struct PolicyResult {
  float rawActions[kJointCount];
  float clampedActions[kJointCount];
  float targetRad[kJointCount];
  float filteredTargetRad[kJointCount];
  uint32_t inferenceTimeUs;
  bool valid;
};

struct RobotStatus {
  RobotState state;
  FaultCode fault;
  bool bleConnected;
  uint16_t lastCommandSequence;
  uint32_t inferenceTimeUs;
  uint32_t missedDeadlines;
};

struct SafetyDecision {
  bool allowMotion;
  RobotState state;
  FaultCode fault;
};

struct ServoCalibration {
  int pwmPin;
  int feedbackAdcPin;
  float defaultPositionRad;
  float jointZeroServoDegrees;
  float directionSign;
  float jointMinimumRad;
  float jointMaximumRad;
  int minimumPulseUs;
  int maximumPulseUs;
  float feedbackMillivoltsAtMinimum;
  float feedbackMillivoltsAtMaximum;
  bool feedbackInverted;
  bool valid;
};

}  // namespace crawler
