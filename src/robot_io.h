#pragma once

#include "crawler_config.h"
#include "crawler_types.h"

class RobotIO {
 public:
  RobotIO();

  bool begin();
  crawler::JointState readJointState();
  void writeTargets(const float targetRad[3]);
  void disableServos();

  bool calibrationValid() const;
  bool usingMockHardware() const;
  uint32_t mockWriteCount() const;
  const float* latestMockTargets() const;

  static bool feedbackMillivoltsToJointRad(
      float millivolts, const crawler::ServoCalibration& calibration,
      float& positionRad);
  static int jointRadToPulseUs(
      float targetRad, const crawler::ServoCalibration& calibration);

 private:
  uint32_t nowMs() const;
  void updateEstimatedVelocity(const float position[3], uint32_t timestampMs,
                               float velocity[3]);

  uint32_t mockStartMs_;
  uint32_t mockWrites_;
  bool servosDisabled_;
  bool estimatorInitialized_;
  uint32_t previousSampleMs_;
  float previousPositionRad_[3];
  float latestMockTargetsRad_[3];
};
