#pragma once

#include <stdint.h>

#include "crawler_config.h"
#include "crawler_types.h"

#if defined(ARDUINO)
#include <Adafruit_BNO08x.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#endif

struct ImuDiagnostics {
  float accelRequestedHz;
  float accelMeasuredHz;
  uint32_t accelAgeUs;
  bool accelValid;
  uint8_t accelAccuracy;
  uint32_t accelSequenceGaps;
  uint32_t accelTimestampBacksteps;

  float gyroRequestedHz;
  float gyroMeasuredHz;
  uint32_t gyroAgeUs;
  bool gyroValid;
  uint8_t gyroAccuracy;
  uint32_t gyroSequenceGaps;
  uint32_t gyroTimestampBacksteps;

  bool resetRecoveryActive;
  uint32_t resetCount;
  uint32_t resetGeneration;

  uint32_t lastDrainUs;
  uint32_t maxDrainUs;
  uint32_t lastEventsPerDrain;
  uint32_t maxEventsPerDrain;
  uint32_t drainBudgetHits;
  uint32_t eventLimitHits;
  uint32_t transactionFailures;
  uint32_t recoveryAttempts;
  uint32_t recoveryFailures;
  uint8_t lastFailureStage;
};

class ImuSensor {
 public:
  ImuSensor();

  bool begin();
  crawler::ImuState read();
  ImuDiagnostics diagnostics();

 private:
  struct SharedState {
    float accelerationMps2[3];
    float angularVelocityRadPerSecond[3];
    uint64_t accelHostTimestampUs;
    uint64_t gyroHostTimestampUs;
    uint64_t accelSensorTimestampUs;
    uint64_t gyroSensorTimestampUs;
    uint8_t accelAccuracy;
    uint8_t gyroAccuracy;
    bool accelValid;
    bool gyroValid;
    bool resetRecoveryActive;
  };

  struct StreamStats {
    uint32_t received;
    uint32_t sequenceGaps;
    uint32_t sequenceAdvances;
    uint64_t firstHostTimestampUs;
    uint64_t lastHostTimestampUs;
    uint64_t firstSensorTimestampUs;
    uint64_t lastSensorTimestampUs;
    uint32_t timestampBacksteps;
    uint8_t lastSequence;
    bool hasPrevious;
  };

  struct TaskDiagnostics {
    uint32_t lastDrainUs;
    uint32_t maxDrainUs;
    uint32_t lastEventsPerDrain;
    uint32_t maxEventsPerDrain;
    uint32_t drainBudgetHits;
    uint32_t eventLimitHits;
    uint32_t transactionFailures;
    uint32_t recoveryAttempts;
    uint32_t recoveryFailures;
    uint8_t lastFailureStage;
  };

#if defined(ARDUINO)
  static void taskEntry(void* argument);
  static void interruptHandler();
  void taskLoop();
  void processSensorEvent(const sh2_SensorValue_t& value);
  void recordReport(StreamStats& stats, const sh2_SensorValue_t& value,
                   uint64_t hostTimestampUs);
  float measuredRate(const StreamStats& stats) const;
  bool configureReports();
  void handleResetIfNeeded();
  void runRecoveryIfNeeded();
  void markResetRecovery();
  uint16_t drainEvents();
#endif

  bool present_;
  SharedState shared_;
  StreamStats accelStats_;
  StreamStats gyroStats_;
  TaskDiagnostics taskDiagnostics_;
  uint32_t resetCount_;
  uint32_t resetGeneration_;
#if defined(ARDUINO)
  portMUX_TYPE mutex_;
  TaskHandle_t taskHandle_;
  Adafruit_BNO08x bno08x_;
  sh2_SensorValue_t sensorValue_;
  bool recoveryInProgress_;
  uint8_t recoveryAttemptsSinceFresh_;
  bool recoveryPending_;
  bool runtimeMode_;
  static ImuSensor* activeInstance_;
#endif
};
