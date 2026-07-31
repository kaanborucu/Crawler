#pragma once

#include "ble_control.h"
#include "policy_pipeline.h"
#include "robot_io.h"
#include "safety.h"

class Crawler {
 public:
  Crawler();

  bool begin();
  void update();

 private:
  void controlCycle();
  void printTelemetry();
  uint64_t nowUs() const;

  BleControl ble_;
  RobotIO robotIo_;
  PolicyPipeline policy_;
  Safety safety_;
  bool initialized_;
  uint64_t nextCycleUs_;
  uint32_t cycleCount_;
  uint32_t missedDeadlines_;
  uint32_t maximumInferenceUs_;
  crawler::JointState latestJoints_;
  crawler::VelocityCommand latestCommand_;
  crawler::PolicyResult latestResult_;
};
