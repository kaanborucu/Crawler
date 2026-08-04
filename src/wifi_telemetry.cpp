#include "wifi_telemetry.h"

#if CRAWLER_ENABLE_WIFI_TELEMETRY && !CRAWLER_WIFI_USE_IDF_HTTPD

#include <Arduino.h>
#include <WiFi.h>
#include <esp_system.h>

#include <cstdio>

#include "wifi_assets.h"
#include "wifi_config.h"

WifiTelemetry* WifiTelemetry::activeInstance_ = nullptr;

namespace {

const char* robotStateName(crawler::RobotState state) {
  switch (state) {
    case crawler::RobotState::Booting: return "BOOTING";
    case crawler::RobotState::Disarmed: return "DISARMED";
    case crawler::RobotState::Running: return "RUNNING";
    case crawler::RobotState::Fault: return "FAULT";
    case crawler::RobotState::EmergencyStopped: return "ESTOP";
  }
  return "UNKNOWN";
}

const char* faultName(crawler::FaultCode fault) {
  switch (fault) {
    case crawler::FaultCode::None: return "NONE";
    case crawler::FaultCode::InvalidCalibration: return "INVALID_CALIBRATION";
    case crawler::FaultCode::SensorInvalid: return "SENSOR_INVALID";
    case crawler::FaultCode::SensorTimeout: return "SENSOR_TIMEOUT";
    case crawler::FaultCode::CommandTimeout: return "COMMAND_TIMEOUT";
    case crawler::FaultCode::BleDisconnected: return "BLE_DISCONNECTED";
    case crawler::FaultCode::NonFiniteObservation:
      return "NONFINITE_OBSERVATION";
    case crawler::FaultCode::PolicyLoadFailure: return "POLICY_LOAD_FAILURE";
    case crawler::FaultCode::NonFinitePolicyOutput:
      return "NONFINITE_POLICY_OUTPUT";
    case crawler::FaultCode::InferenceDeadlineMiss:
      return "INFERENCE_DEADLINE_MISS";
    case crawler::FaultCode::JointLimitViolation:
      return "JOINT_LIMIT_VIOLATION";
    case crawler::FaultCode::EmergencyStopRequested:
      return "EMERGENCY_STOP_REQUESTED";
  }
  return "UNKNOWN";
}

}  // namespace

WifiTelemetry::WifiTelemetry(std::atomic<bool>* policyInferenceActive,
                             std::atomic<bool>* policyControlActive)
    : httpServer_(80),
      webSocketServer_(81),
      snapshotMutex_(portMUX_INITIALIZER_UNLOCKED),
      metricsMutex_(portMUX_INITIALIZER_UNLOCKED),
      taskHandle_(nullptr),
      completionTask_(nullptr),
      policyWindowRequested_(false),
      requestedWindowUs_(0),
      windowOverrunRecorded_(false),
      latest_{},
      metrics_{},
      lastTelemetryMs_(0),
      telemetrySuppressed_(false),
      clientNumber_(0),
      clientConnected_(false),
      policyInferenceActive_(policyInferenceActive),
      policyControlActive_(policyControlActive) {}

bool WifiTelemetry::begin() {
  activeInstance_ = this;

  WiFi.mode(WIFI_AP);
  const IPAddress apIp(192, 168, 4, 1);
  const IPAddress gateway(192, 168, 4, 1);
  const IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIp, gateway, subnet);
  const bool apStarted = WiFi.softAP(
      crawler::config::wifi::apSsid, crawler::config::wifi::apPassword,
      crawler::config::wifi::apChannel, 0,
      crawler::config::wifi::apMaxClients);

  httpServer_.on("/", HTTP_GET, [this]() { serveIndex(); });
  httpServer_.on("/app.js", HTTP_GET, [this]() { serveApp(); });
  httpServer_.on("/style.css", HTTP_GET, [this]() { serveStyle(); });
  httpServer_.onNotFound([this]() {
    httpServer_.send(404, "text/plain", "Not found");
  });
  httpServer_.begin();

  webSocketServer_.onEvent(webSocketEvent);
  webSocketServer_.begin();

  if (xTaskCreatePinnedToCore(taskEntry, "crawler_wifi",
                              crawler::config::wifi::taskStackSize, this,
                              crawler::config::wifi::taskPriority, &taskHandle_,
                              crawler::config::wifi::taskCore) != pdPASS) {
    Serial0.println("Wi-Fi telemetry task could not start");
    return false;
  }

  Serial0.printf("Wi-Fi SSID: %s\n", crawler::config::wifi::apSsid);
  Serial0.printf("SoftAP IP: %s\n", WiFi.softAPIP().toString().c_str());
  Serial0.println("Dashboard: http://192.168.4.1");
  Serial0.println("WebSocket: ws://192.168.4.1:81/");
  if (!apStarted) Serial0.println("Wi-Fi SoftAP failed to start");
  return apStarted;
}

void WifiTelemetry::publish(const TelemetrySnapshot& snapshot) {
  portENTER_CRITICAL(&snapshotMutex_);
  latest_ = snapshot;
  portEXIT_CRITICAL(&snapshotMutex_);
}

void WifiTelemetry::setPolicyControlActive(bool active) {
  if (policyControlActive_ != nullptr) {
    const bool wasActive =
        policyControlActive_->exchange(active, std::memory_order_acq_rel);
    if (!active && wasActive && taskHandle_ != nullptr) {
      xTaskNotifyGive(taskHandle_);
    }
    return;
  }
}

void WifiTelemetry::runPostInferenceWindow(uint32_t availableWindowUs) {
  if (taskHandle_ == nullptr) return;

  // The control task waits for completion so the next policy cycle cannot
  // overlap this network pass. A late pass is recorded, then completed before
  // the control task is allowed to continue.
  (void)ulTaskNotifyTake(pdTRUE, 0);
  windowOverrunRecorded_.store(false, std::memory_order_release);
  requestedWindowUs_.store(availableWindowUs, std::memory_order_release);
  completionTask_.store(xTaskGetCurrentTaskHandle(),
                        std::memory_order_release);
  policyWindowRequested_.store(true, std::memory_order_release);
  xTaskNotifyGive(taskHandle_);

  const uint32_t waitMs = (availableWindowUs + 999u) / 1000u;
  if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(waitMs == 0 ? 1 : waitMs)) ==
      0) {
    recordNetworkWindowOverrun();
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}

WifiTelemetryMetrics WifiTelemetry::metrics() {
  WifiTelemetryMetrics result = {};
  portENTER_CRITICAL(&metricsMutex_);
  result = metrics_;
  portEXIT_CRITICAL(&metricsMutex_);
  return result;
}

void WifiTelemetry::taskEntry(void* argument) {
  static_cast<WifiTelemetry*>(argument)->taskLoop();
}

void WifiTelemetry::webSocketEvent(uint8_t clientNumber, WStype_t type,
                                   uint8_t* payload, size_t length) {
  (void)payload;
  (void)length;
  WifiTelemetry* instance = activeInstance_;
  if (instance == nullptr) return;

  if (type == WStype_CONNECTED) {
    if (instance->clientConnected_) {
      instance->webSocketServer_.disconnect(clientNumber);
      return;
    }
    instance->clientNumber_ = clientNumber;
    instance->clientConnected_ = true;
  } else if (type == WStype_DISCONNECTED &&
             clientNumber == instance->clientNumber_) {
    instance->clientConnected_ = false;
  }
  // Text and binary application messages are deliberately ignored.
}

void WifiTelemetry::taskLoop() {
  lastTelemetryMs_ = millis();
  for (;;) {
    const bool policyControlActive =
        policyControlActive_ != nullptr &&
        policyControlActive_->load(std::memory_order_acquire);
    if (policyControlActive) {
      (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      if (policyWindowRequested_.exchange(false, std::memory_order_acq_rel)) {
        serviceNetworkPass(requestedWindowUs_.load(std::memory_order_acquire),
                           true);
        completePostInferenceWindow();
      }
      continue;
    }

    if (ulTaskNotifyTake(pdTRUE, 0) != 0 &&
        policyWindowRequested_.exchange(false, std::memory_order_acq_rel)) {
      serviceNetworkPass(requestedWindowUs_.load(std::memory_order_acquire),
                         true);
      completePostInferenceWindow();
      continue;
    }

    serviceNetworkPass(UINT32_MAX, false);
    vTaskDelay(pdMS_TO_TICKS(crawler::config::wifi::networkServiceIntervalMs));
  }
}

void WifiTelemetry::serviceNetworkPass(uint32_t availableWindowUs,
                                       bool postInference) {
  const uint32_t passStartUs = micros();
  httpServer_.handleClient();
  webSocketServer_.loop();
  const uint32_t networkServiceUs = micros() - passStartUs;
  recordNetworkService(networkServiceUs);

  const bool serviceOverrun =
      postInference && networkServiceUs > availableWindowUs;
  if (serviceOverrun) {
    recordNetworkWindowOverrun();
    telemetrySuppressed_ = true;
  } else if (postInference && telemetrySuppressed_) {
    // A later service pass completed inside its available window, so it is a
    // safe opportunity to resume telemetry.
    telemetrySuppressed_ = false;
  } else if (!postInference) {
    telemetrySuppressed_ = false;
  }

  const uint32_t now = millis();
  const bool telemetryDue =
      now - lastTelemetryMs_ >= crawler::config::wifi::telemetryPeriodMs;
  if (telemetryDue) lastTelemetryMs_ = now;
  if (telemetryDue && clientConnected_ &&
      webSocketServer_.clientIsConnected(clientNumber_)) {
    if (serviceOverrun || telemetrySuppressed_) {
      recordTelemetryDrop();
    } else {
      sendTelemetry();
    }
  }

  if (postInference && micros() - passStartUs > availableWindowUs) {
    recordNetworkWindowOverrun();
    telemetrySuppressed_ = true;
  }
}

void WifiTelemetry::completePostInferenceWindow() {
  TaskHandle_t waiter =
      completionTask_.exchange(nullptr, std::memory_order_acq_rel);
  if (waiter != nullptr) xTaskNotifyGive(waiter);
}

void WifiTelemetry::recordTelemetryDrop() {
  portENTER_CRITICAL(&metricsMutex_);
  ++metrics_.telemetryDrops;
  portEXIT_CRITICAL(&metricsMutex_);
}

void WifiTelemetry::recordWebSocketFailure() {
  portENTER_CRITICAL(&metricsMutex_);
  ++metrics_.telemetryDrops;
  ++metrics_.websocketSendFailures;
  portEXIT_CRITICAL(&metricsMutex_);
}

void WifiTelemetry::recordNetworkService(uint32_t elapsedUs) {
  portENTER_CRITICAL(&metricsMutex_);
  metrics_.networkServiceUs = elapsedUs;
  if (elapsedUs > metrics_.networkServiceMaxUs) {
    metrics_.networkServiceMaxUs = elapsedUs;
  }
  portEXIT_CRITICAL(&metricsMutex_);
}

void WifiTelemetry::recordJsonEncode(uint32_t elapsedUs) {
  portENTER_CRITICAL(&metricsMutex_);
  metrics_.jsonEncodeUs = elapsedUs;
  if (elapsedUs > metrics_.jsonEncodeMaxUs) {
    metrics_.jsonEncodeMaxUs = elapsedUs;
  }
  portEXIT_CRITICAL(&metricsMutex_);
}

void WifiTelemetry::recordWebSocketSend(uint32_t elapsedUs) {
  portENTER_CRITICAL(&metricsMutex_);
  metrics_.websocketSendUs = elapsedUs;
  if (elapsedUs > metrics_.websocketSendMaxUs) {
    metrics_.websocketSendMaxUs = elapsedUs;
  }
  portEXIT_CRITICAL(&metricsMutex_);
}

void WifiTelemetry::recordNetworkWindowOverrun() {
  bool expected = false;
  if (!windowOverrunRecorded_.compare_exchange_strong(
          expected, true, std::memory_order_acq_rel)) {
    return;
  }
  portENTER_CRITICAL(&metricsMutex_);
  ++metrics_.networkWindowOverruns;
  portEXIT_CRITICAL(&metricsMutex_);
}

void WifiTelemetry::sendTelemetry() {
  // Keep HTTP/WebSocket servicing alive, but do not snapshot, format, or send
  // telemetry while the real-time policy pipeline is active. This is a
  // diagnostic guard; dropped frames are reported by the next successful
  // telemetry packet.
  if (policyInferenceActive_ != nullptr &&
      policyInferenceActive_->load(std::memory_order_acquire)) {
    recordTelemetryDrop();
    return;
  }

  TelemetrySnapshot snapshot = {};
  portENTER_CRITICAL(&snapshotMutex_);
  snapshot = latest_;
  portEXIT_CRITICAL(&snapshotMutex_);

  snapshot.uptimeMs = millis();
  const uint32_t freeHeap = ESP.getFreeHeap();
  const uint32_t minimumFreeHeap = esp_get_minimum_free_heap_size();
  const WifiTelemetryMetrics currentMetrics = metrics();
  char json[crawler::config::wifi::jsonBufferSize] = {};
  const uint32_t jsonStartUs = micros();
  const int written = snprintf(
      json, sizeof(json),
      "{\"timestamp_ms\":%lu,\"uptime_ms\":%lu,"
      "\"q_rad\":[%.6g,%.6g,%.6g],\"qd_rad_s\":[%.6g,%.6g,%.6g],"
      "\"target_rad\":[%.6g,%.6g,%.6g],"
      "\"acc_mps2\":[%.6g,%.6g,%.6g],"
      "\"gyro_rad_s\":[%.6g,%.6g,%.6g],"
      "\"action_norm\":[%.6g,%.6g,%.6g],"
      "\"cmd_mps\":[%.6g,%.6g],"
      "\"inference_us\":%lu,\"inference_max_us\":%lu,"
      "\"deadline_misses\":%lu,\"policy_hz\":%.6g,"
      "\"imu_configured_hz\":%lu,"
      "\"imu_accel_requested_hz\":%.6g,\"imu_accel_measured_hz\":%.6g,"
      "\"imu_accel_age_us\":%lu,\"imu_accel_valid\":%s,"
      "\"imu_accel_accuracy\":%u,\"imu_accel_sequence_gaps\":%lu,"
      "\"imu_accel_timestamp_backsteps\":%lu,"
      "\"imu_gyro_requested_hz\":%.6g,\"imu_gyro_measured_hz\":%.6g,"
      "\"imu_gyro_age_us\":%lu,\"imu_gyro_valid\":%s,"
      "\"imu_gyro_accuracy\":%u,\"imu_gyro_sequence_gaps\":%lu,"
      "\"imu_gyro_timestamp_backsteps\":%lu,"
      "\"imu_reset_recovery_active\":%s,\"imu_reset_count\":%lu,"
      "\"imu_reset_generation\":%lu,"
      "\"imu_last_drain_us\":%lu,\"imu_max_drain_us\":%lu,"
      "\"imu_last_events_per_drain\":%lu,\"imu_max_events_per_drain\":%lu,"
      "\"imu_drain_budget_hits\":%lu,\"imu_event_limit_hits\":%lu,"
      "\"imu_transaction_failures\":%lu,\"imu_recovery_attempts\":%lu,"
      "\"imu_recovery_failures\":%lu,\"imu_last_failure_stage\":%u,"
      "\"joint_configured_hz\":%lu,"
      "\"fault_code\":%u,\"fault_name\":\"%s\","
      "\"state_name\":\"%s\",\"enabled\":%s,"
      "\"command_age_ms\":%lu,\"heap\":%lu,\"heap_min\":%lu,"
      "\"telemetry_drops\":%lu,\"ws_send_failures\":%lu,"
      "\"network_service_us\":%lu,\"network_service_max_us\":%lu,"
      "\"json_encode_us\":%lu,\"json_encode_max_us\":%lu,"
      "\"websocket_send_us\":%lu,\"websocket_send_max_us\":%lu,"
      "\"network_window_overruns\":%lu,\"rssi\":null}",
      static_cast<unsigned long>(snapshot.timestampMs),
      static_cast<unsigned long>(snapshot.uptimeMs),
      static_cast<double>(snapshot.jointPositionRad[0]),
      static_cast<double>(snapshot.jointPositionRad[1]),
      static_cast<double>(snapshot.jointPositionRad[2]),
      static_cast<double>(snapshot.jointVelocityRadPerSecond[0]),
      static_cast<double>(snapshot.jointVelocityRadPerSecond[1]),
      static_cast<double>(snapshot.jointVelocityRadPerSecond[2]),
      static_cast<double>(snapshot.jointTargetRad[0]),
      static_cast<double>(snapshot.jointTargetRad[1]),
      static_cast<double>(snapshot.jointTargetRad[2]),
      static_cast<double>(snapshot.accelerationMps2[0]),
      static_cast<double>(snapshot.accelerationMps2[1]),
      static_cast<double>(snapshot.accelerationMps2[2]),
      static_cast<double>(snapshot.gyroRadPerSecond[0]),
      static_cast<double>(snapshot.gyroRadPerSecond[1]),
      static_cast<double>(snapshot.gyroRadPerSecond[2]),
      static_cast<double>(snapshot.policyActionNormalized[0]),
      static_cast<double>(snapshot.policyActionNormalized[1]),
      static_cast<double>(snapshot.policyActionNormalized[2]),
      static_cast<double>(snapshot.velocityCommandMps[0]),
      static_cast<double>(snapshot.velocityCommandMps[1]),
      static_cast<unsigned long>(snapshot.inferenceTimeUs),
      static_cast<unsigned long>(snapshot.maximumInferenceTimeUs),
      static_cast<unsigned long>(snapshot.policyDeadlineMisses),
      static_cast<double>(snapshot.policyRateHz),
      static_cast<unsigned long>(snapshot.imuConfiguredHz),
      static_cast<double>(snapshot.imuAccelRequestedHz),
      static_cast<double>(snapshot.imuAccelMeasuredHz),
      static_cast<unsigned long>(snapshot.imuAccelAgeUs),
      snapshot.imuAccelValid ? "true" : "false",
      static_cast<unsigned>(snapshot.imuAccelAccuracy),
      static_cast<unsigned long>(snapshot.imuAccelSequenceGaps),
      static_cast<unsigned long>(snapshot.imuAccelTimestampBacksteps),
      static_cast<double>(snapshot.imuGyroRequestedHz),
      static_cast<double>(snapshot.imuGyroMeasuredHz),
      static_cast<unsigned long>(snapshot.imuGyroAgeUs),
      snapshot.imuGyroValid ? "true" : "false",
      static_cast<unsigned>(snapshot.imuGyroAccuracy),
      static_cast<unsigned long>(snapshot.imuGyroSequenceGaps),
      static_cast<unsigned long>(snapshot.imuGyroTimestampBacksteps),
      snapshot.imuResetRecoveryActive ? "true" : "false",
      static_cast<unsigned long>(snapshot.imuResetCount),
      static_cast<unsigned long>(snapshot.imuResetGeneration),
      static_cast<unsigned long>(snapshot.imuLastDrainUs),
      static_cast<unsigned long>(snapshot.imuMaxDrainUs),
      static_cast<unsigned long>(snapshot.imuLastEventsPerDrain),
      static_cast<unsigned long>(snapshot.imuMaxEventsPerDrain),
      static_cast<unsigned long>(snapshot.imuDrainBudgetHits),
      static_cast<unsigned long>(snapshot.imuEventLimitHits),
      static_cast<unsigned long>(snapshot.imuTransactionFailures),
      static_cast<unsigned long>(snapshot.imuRecoveryAttempts),
      static_cast<unsigned long>(snapshot.imuRecoveryFailures),
      static_cast<unsigned>(snapshot.imuLastFailureStage),
      static_cast<unsigned long>(snapshot.jointConfiguredHz),
      static_cast<unsigned>(snapshot.faultCode), faultName(snapshot.faultCode),
      robotStateName(snapshot.robotState), snapshot.servosEnabled ? "true" : "false",
      static_cast<unsigned long>(snapshot.bleCommandAgeMs),
      static_cast<unsigned long>(freeHeap),
      static_cast<unsigned long>(minimumFreeHeap),
      static_cast<unsigned long>(currentMetrics.telemetryDrops),
      static_cast<unsigned long>(currentMetrics.websocketSendFailures),
      static_cast<unsigned long>(currentMetrics.networkServiceUs),
      static_cast<unsigned long>(currentMetrics.networkServiceMaxUs),
      static_cast<unsigned long>(currentMetrics.jsonEncodeUs),
      static_cast<unsigned long>(currentMetrics.jsonEncodeMaxUs),
      static_cast<unsigned long>(currentMetrics.websocketSendUs),
      static_cast<unsigned long>(currentMetrics.websocketSendMaxUs),
      static_cast<unsigned long>(currentMetrics.networkWindowOverruns));
  recordJsonEncode(micros() - jsonStartUs);

  if (written < 0 || static_cast<size_t>(written) >= sizeof(json)) {
    recordTelemetryDrop();
    return;
  }
  if (!clientConnected_ ||
      !webSocketServer_.clientIsConnected(clientNumber_)) {
    return;
  }
  const uint32_t sendStartUs = micros();
  const bool sent = webSocketServer_.sendTXT(
      clientNumber_, json, static_cast<size_t>(written));
  recordWebSocketSend(micros() - sendStartUs);
  if (!sent) {
    recordWebSocketFailure();
  }
}

void WifiTelemetry::serveIndex() {
  httpServer_.send(200, "text/html; charset=utf-8", kCrawlerIndexHtml);
}

void WifiTelemetry::serveApp() {
  httpServer_.send(200, "application/javascript", kCrawlerAppJs);
}

void WifiTelemetry::serveStyle() {
  httpServer_.send(200, "text/css", kCrawlerStyleCss);
}

#endif
