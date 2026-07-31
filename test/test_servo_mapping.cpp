#include <cmath>
#include <cstdio>

#include "robot_io.h"

namespace servo_tests {

void expect(bool condition, const char* message, int& failures) {
  if (!condition) {
    std::fprintf(stderr, "servo: %s\n", message);
    ++failures;
  }
}

crawler::ServoCalibration calibration(float direction) {
  crawler::ServoCalibration value = {
      1, 2, 0.0f, 90.0f, direction, -1.0f, 1.0f, 1000, 2000,
      1000.0f, 3000.0f, false, true};
  return value;
}

}  // namespace servo_tests

int runServoMappingTests() {
  using namespace servo_tests;
  int failures = 0;
  const crawler::ServoCalibration normal = calibration(1.0f);
  expect(RobotIO::jointRadToPulseUs(0.0f, normal) == 1500,
         "zero joint target did not map to the servo midpoint", failures);
  expect(RobotIO::jointRadToPulseUs(1.0f, normal) == 2000,
         "positive joint limit did not map to the maximum pulse", failures);
  expect(RobotIO::jointRadToPulseUs(2.0f, normal) == 2000,
         "joint target was not clamped to its limit", failures);

  const crawler::ServoCalibration inverted = calibration(-1.0f);
  expect(RobotIO::jointRadToPulseUs(1.0f, inverted) == 1000,
         "direction sign was not applied", failures);

  float position = 0.0f;
  expect(RobotIO::feedbackMillivoltsToJointRad(2000.0f, normal, position) &&
             std::fabs(position) < 1.0e-5f,
         "feedback midpoint did not map to zero radians", failures);
  const crawler::ServoCalibration feedbackInverted = {1, 2, 0.0f, 90.0f, 1.0f,
                                                      -1.0f, 1.0f, 1000, 2000,
                                                      1000.0f, 3000.0f, true,
                                                      true};
  expect(RobotIO::feedbackMillivoltsToJointRad(1000.0f, feedbackInverted,
                                               position) &&
             std::fabs(position - 1.0f) < 1.0e-5f,
         "feedback inversion was not applied", failures);
  expect(!RobotIO::feedbackMillivoltsToJointRad(500.0f, normal, position),
         "out-of-range feedback was accepted", failures);

  crawler::ServoCalibration invalid = normal;
  invalid.valid = false;
  expect(RobotIO::jointRadToPulseUs(0.0f, invalid) < 0,
         "invalid calibration produced a PWM value", failures);
  return failures;
}
