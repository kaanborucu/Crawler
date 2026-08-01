#pragma once

#include "crawler_config.h"
#include "crawler_types.h"

#if defined(ARDUINO)
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#endif

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
  uint32_t nowMs() const;
  crawler::JointState sampleJointState();
  void updateEstimatedVelocity(const float position[3], uint32_t timestampMs,
                               float velocity[3]);
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
#if defined(ARDUINO)
  portMUX_TYPE mutex_;
  TaskHandle_t taskHandle_;
#endif
};
