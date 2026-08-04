#pragma once

#include "crawler_config.h"
#include "crawler_types.h"

#if defined(ARDUINO)
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#endif

enum class ServoCalibrationState : uint8_t {
  Idle = 0,
  Running = 1,
  Completed = 2,
  Failed = 3,
};

class RobotIO {
 public:
  RobotIO();

  bool begin();
  crawler::JointState readJointState();
  void writeTargets(const float targetRad[3]);
  void holdCurrentPosition(const crawler::JointState& joints);
  void disableServos();
  void setMotionGate(bool allowed);
  void setFastSafetyGate(bool allowed);

  bool startCalibration(const crawler::JointState& currentJoints);
  void updateCalibration();
  void abortCalibration();
  bool calibrationActive() const;
  ServoCalibrationState calibrationState() const;

  bool calibrationValid() const;
  bool usingMockHardware() const;
  bool servosEnabled() const;
  uint32_t mockWriteCount() const;
  const float* latestMockTargets() const;

  static bool feedbackMillivoltsToJointRad(
      float millivolts, const crawler::ServoCalibration& calibration,
      float& positionRad);
  static int jointRadToPulseUs(
      float targetRad, const crawler::ServoCalibration& calibration);

 private:
  enum class CalibrationPhase : uint8_t {
    InitialZeroWait = 0,
    Sampling = 1,
    NeutralWait = 2,
  };

  uint32_t nowMs() const;
  crawler::JointState sampleJointState();
  void updateEstimatedVelocity(const float position[3], uint32_t timestampMs,
                               float velocity[3]);
  bool writeAllCalibrationPose(float servoDegrees);
  bool writeCalibrationPose(uint8_t activeJoint, float servoDegrees);
  bool writeNeutralCalibrationPose();
  bool readFilteredRawAdc(uint8_t joint, uint16_t& rawAdc) const;
  bool finishCalibration();
  bool validateCalibrationSamples() const;
#if defined(ARDUINO)
  static void taskEntry(void* argument);
  void taskLoop();
#endif

  uint32_t mockStartMs_;
  uint32_t mockWrites_;
  bool servosDisabled_;
  bool estimatorInitialized_;
  uint32_t previousSampleMs_;
  float previousPositionRad_[3];
  float latestMockTargetsRad_[3];
  crawler::JointState latestHardwareState_;
  volatile bool motionGate_;
  volatile bool fastSafetyGate_;
  ServoCalibrationState calibrationState_;
  CalibrationPhase calibrationPhase_;
  uint8_t calibrationJoint_;
  uint8_t calibrationPoint_;
  uint32_t calibrationSettleUntilMs_;
  uint16_t calibrationSamples_[3][crawler::config::servo::calibrationPointCount];
#if defined(ARDUINO)
  portMUX_TYPE mutex_;
  TaskHandle_t taskHandle_;
#endif
};
