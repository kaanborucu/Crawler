#pragma once

#include <atomic>

#include "ble_control.h"
#include "imu_sensor.h"
#include "policy_pipeline.h"
#include "robot_io.h"
#include "safety.h"
#include "wifi_telemetry.h"

#if defined(ARDUINO)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

class Crawler {
 public:
  Crawler();

  bool begin();
  void update();

 private:
  void controlCycle();
  void printTelemetry();
  void refreshTelemetryInputs();
  void publishTelemetrySnapshot();
  void completePolicyNetworkWindow();
  uint64_t nowUs() const;
#if defined(ARDUINO)
  static void safetyTaskEntry(void* argument);
  void safetyTaskLoop();
#endif

  BleControl ble_;
  ImuSensor imu_;
  RobotIO robotIo_;
  PolicyPipeline policy_;
  Safety safety_;
  std::atomic<bool> policyInferenceActive_;
  std::atomic<bool> policyControlActive_;
  WifiTelemetry wifiTelemetry_;
  bool initialized_;
  bool policyNeedsInitialization_;
  uint64_t nextCycleUs_;
  uint32_t cycleCount_;
  uint32_t missedDeadlines_;
  uint32_t maximumInferenceUs_;
  bool calibrationRequestConsumed_;
  crawler::JointState latestJoints_;
  crawler::ImuState latestImu_;
  ImuDiagnostics latestImuDiagnostics_;
  crawler::VelocityCommand latestCommand_;
  crawler::PolicyResult latestResult_;
#if defined(ARDUINO)
  TaskHandle_t safetyTaskHandle_;
#endif
};
