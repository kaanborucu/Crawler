#include "wifi_telemetry.h"

#if CRAWLER_ENABLE_WIFI_TELEMETRY && CRAWLER_WIFI_USE_IDF_HTTPD

#include <Arduino.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <errno.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

#include "wifi_assets.h"
#include "wifi_config.h"

WifiTelemetry* WifiTelemetry::activeInstance_ = nullptr;

namespace {

constexpr uint32_t kSlowSendThresholdUs = 100000;
constexpr size_t kWebSocketReceiveLimit = 128;

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
    : serverHandle_(nullptr),
      snapshotMutex_(portMUX_INITIALIZER_UNLOCKED),
      metricsMutex_(portMUX_INITIALIZER_UNLOCKED),
      lifecycleMutex_(portMUX_INITIALIZER_UNLOCKED),
      taskHandle_(nullptr),
      latest_{},
      metrics_{},
      lifecycle_(),
      pendingWork_{},
      pendingStartedUs_(0),
      pendingAgeMaxUs_(0),
      pendingAgeExceededCount_(0),
      pendingAgeExceededForCurrentSend_(false),
      networkHealthDegraded_(false),
      policyInferenceActive_(policyInferenceActive),
      policyControlActive_(policyControlActive) {
  pendingWork_.owner = this;
  pendingWork_.fd = -1;
}

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

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.core_id = 0;
  config.task_priority = 3;
  config.stack_size = 8192;
  config.max_uri_handlers = 4;
  config.max_open_sockets = 5;
  config.lru_purge_enable = true;
  config.send_wait_timeout = 1;
  config.recv_wait_timeout = 1;
  config.close_fn = httpdCloseCallback;

  esp_err_t serverResult = httpd_start(&serverHandle_, &config);
  if (serverResult == ESP_OK) {
    const httpd_uri_t index = {"/", HTTP_GET, indexHandler, nullptr, false,
                               false, nullptr};
    const httpd_uri_t app = {"/app.js", HTTP_GET, appHandler, nullptr, false,
                             false, nullptr};
    const httpd_uri_t style = {"/style.css", HTTP_GET, styleHandler, nullptr,
                               false, false, nullptr};
    const httpd_uri_t websocket = {"/ws", HTTP_GET, webSocketHandler, nullptr,
                                   true, false, nullptr};
    const esp_err_t indexResult = httpd_register_uri_handler(serverHandle_, &index);
    const esp_err_t appResult = httpd_register_uri_handler(serverHandle_, &app);
    const esp_err_t styleResult = httpd_register_uri_handler(serverHandle_, &style);
    const esp_err_t wsResult =
        httpd_register_uri_handler(serverHandle_, &websocket);
    if (indexResult != ESP_OK || appResult != ESP_OK ||
        styleResult != ESP_OK || wsResult != ESP_OK) {
      Serial0.printf("HTTPD route registration failed: %d,%d,%d,%d\n",
                     indexResult, appResult, styleResult, wsResult);
      serverResult = ESP_FAIL;
    }
  }

  if (serverHandle_ != nullptr &&
      xTaskCreatePinnedToCore(producerTaskEntry, "crawler_wifi_tx",
                              crawler::config::wifi::taskStackSize, this,
                              crawler::config::wifi::taskPriority, &taskHandle_,
                              crawler::config::wifi::taskCore) != pdPASS) {
    Serial0.println("Wi-Fi telemetry producer could not start");
    serverResult = ESP_FAIL;
  }

  Serial0.printf("Wi-Fi SSID: %s\n", crawler::config::wifi::apSsid);
  Serial0.printf("SoftAP IP: %s\n", WiFi.softAPIP().toString().c_str());
  Serial0.println("Dashboard: http://192.168.4.1/");
  Serial0.println("WebSocket: ws://192.168.4.1/ws");
  if (!apStarted) Serial0.println("Wi-Fi SoftAP failed to start");
  if (serverResult != ESP_OK) {
    Serial0.printf("ESP-IDF HTTPD startup failed: %d\n", serverResult);
  }
  return apStarted && serverResult == ESP_OK;
}

void WifiTelemetry::publish(const TelemetrySnapshot& snapshot) {
  portENTER_CRITICAL(&snapshotMutex_);
  latest_ = snapshot;
  portEXIT_CRITICAL(&snapshotMutex_);
}

void WifiTelemetry::setPolicyControlActive(bool active) {
  if (policyControlActive_ != nullptr) {
    policyControlActive_->store(active, std::memory_order_release);
  }
}

void WifiTelemetry::runPostInferenceWindow(uint32_t availableWindowUs) {
  // Event-driven HTTPD builds never make the policy task call or wait for
  // networking. A CPU0 send can continue into a later CPU1 inference cycle.
  (void)availableWindowUs;
}

WifiTelemetryMetrics WifiTelemetry::metrics() {
  WifiTelemetryMetrics result = {};
  portENTER_CRITICAL(&metricsMutex_);
  result = metrics_;
  portEXIT_CRITICAL(&metricsMutex_);
  return result;
}

void WifiTelemetry::observePendingHealthLocked() {
  const crawler::wifi::PendingWebSocketSend pending = lifecycle_.pending();
  const uint32_t pendingAgeUs = pending.occupied ? micros() - pendingStartedUs_
                                                  : 0u;
  if (pendingAgeUs > pendingAgeMaxUs_) pendingAgeMaxUs_ = pendingAgeUs;
  if (pending.occupied &&
      pendingAgeUs > crawler::config::wifi::pendingSendAgeLimitUs) {
    networkHealthDegraded_ = true;
    if (!pendingAgeExceededForCurrentSend_) {
      pendingAgeExceededForCurrentSend_ = true;
      ++pendingAgeExceededCount_;
    }
  } else if (!pending.occupied) {
    pendingAgeExceededForCurrentSend_ = false;
  }
}

void WifiTelemetry::markNetworkHealthDegradedLocked() {
  networkHealthDegraded_ = true;
}

void WifiTelemetry::refreshLifecycleMetrics() {
  WifiTelemetryMetrics lifecycleSnapshot = {};
  portENTER_CRITICAL(&lifecycleMutex_);
  observePendingHealthLocked();
  const crawler::wifi::ActiveWebSocket active = lifecycle_.active();
  const crawler::wifi::WebSocketLifecycleMetrics current = lifecycle_.metrics();
  lifecycleSnapshot.telemetryDrops = current.telemetryDrops;
  lifecycleSnapshot.websocketSendFailures = current.websocketSendFailures;
  lifecycleSnapshot.websocketConnected =
      active.state == crawler::wifi::WebSocketState::Connected ? 1u : 0u;
  lifecycleSnapshot.websocketConnectCount = current.websocketConnectCount;
  lifecycleSnapshot.websocketDisconnectCount =
      current.websocketDisconnectCount;
  lifecycleSnapshot.websocketGeneration = active.generation;
  lifecycleSnapshot.httpdQueueFailures = current.httpdQueueFailures;
  lifecycleSnapshot.httpdPendingSends = current.httpdPendingSends;
  lifecycleSnapshot.sessionCloseFailures = current.sessionCloseFailures;
  lifecycleSnapshot.httpdPendingAgeUs = lifecycle_.pending().occupied
                                                ? micros() - pendingStartedUs_
                                                : 0u;
  lifecycleSnapshot.httpdPendingAgeMaxUs = pendingAgeMaxUs_;
  lifecycleSnapshot.networkHealthDegraded = networkHealthDegraded_ ? 1u : 0u;
  lifecycleSnapshot.pendingAgeExceededCount = pendingAgeExceededCount_;
  portEXIT_CRITICAL(&lifecycleMutex_);

  portENTER_CRITICAL(&metricsMutex_);
  metrics_.telemetryDrops = lifecycleSnapshot.telemetryDrops;
  metrics_.websocketSendFailures = lifecycleSnapshot.websocketSendFailures;
  metrics_.websocketConnected = lifecycleSnapshot.websocketConnected;
  metrics_.websocketConnectCount = lifecycleSnapshot.websocketConnectCount;
  metrics_.websocketDisconnectCount =
      lifecycleSnapshot.websocketDisconnectCount;
  metrics_.websocketGeneration = lifecycleSnapshot.websocketGeneration;
  metrics_.httpdQueueFailures = lifecycleSnapshot.httpdQueueFailures;
  metrics_.httpdPendingSends = lifecycleSnapshot.httpdPendingSends;
  metrics_.sessionCloseFailures = lifecycleSnapshot.sessionCloseFailures;
  metrics_.httpdPendingAgeUs = lifecycleSnapshot.httpdPendingAgeUs;
  metrics_.httpdPendingAgeMaxUs = lifecycleSnapshot.httpdPendingAgeMaxUs;
  metrics_.networkHealthDegraded = lifecycleSnapshot.networkHealthDegraded;
  metrics_.pendingAgeExceededCount = lifecycleSnapshot.pendingAgeExceededCount;
  portEXIT_CRITICAL(&metricsMutex_);
}

void WifiTelemetry::producerTaskEntry(void* argument) {
  static_cast<WifiTelemetry*>(argument)->producerTaskLoop();
}

void WifiTelemetry::producerTaskLoop() {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period =
      pdMS_TO_TICKS(crawler::config::wifi::telemetryPeriodMs);
  for (;;) {
    vTaskDelayUntil(&lastWake, period == 0 ? 1 : period);
    produceTelemetry();
  }
}

void WifiTelemetry::produceTelemetry() {
  if (serverHandle_ == nullptr) return;

  refreshLifecycleMetrics();

  if (policyInferenceActive_ != nullptr &&
      policyInferenceActive_->load(std::memory_order_acquire)) {
    portENTER_CRITICAL(&lifecycleMutex_);
    if (lifecycle_.active().state ==
        crawler::wifi::WebSocketState::Connected) {
      lifecycle_.recordTelemetryDrop();
    }
    portEXIT_CRITICAL(&lifecycleMutex_);
    refreshLifecycleMetrics();
    return;
  }

  crawler::wifi::PreparePendingResult prepared;
  portENTER_CRITICAL(&lifecycleMutex_);
  prepared = lifecycle_.preparePendingSend(0);
  if (prepared == crawler::wifi::PreparePendingResult::Prepared) {
    const crawler::wifi::PendingWebSocketSend pending = lifecycle_.pending();
    pendingWork_.owner = this;
    pendingWork_.server = serverHandle_;
    pendingWork_.fd = pending.fd;
    pendingWork_.generation = pending.generation;
    pendingWork_.bufferIndex = pending.bufferIndex;
    pendingWork_.payloadLength = 0;
    pendingStartedUs_ = micros();
  } else if (prepared == crawler::wifi::PreparePendingResult::PendingBusy) {
    lifecycle_.recordTelemetryDrop();
  }
  portEXIT_CRITICAL(&lifecycleMutex_);
  if (prepared != crawler::wifi::PreparePendingResult::Prepared) return;

  if (!encodePendingPayload()) {
    portENTER_CRITICAL(&lifecycleMutex_);
    lifecycle_.cancelProducerOwnedPending(true);
    pendingStartedUs_ = 0;
    portEXIT_CRITICAL(&lifecycleMutex_);
    return;
  }

  // Recheck at the submission boundary. Encoding began outside inference, but
  // policy execution may have started while the CPU0 producer was formatting.
  if (policyInferenceActive_ != nullptr &&
      policyInferenceActive_->load(std::memory_order_acquire)) {
    portENTER_CRITICAL(&lifecycleMutex_);
    lifecycle_.cancelProducerOwnedPending(true);
    pendingStartedUs_ = 0;
    portEXIT_CRITICAL(&lifecycleMutex_);
    return;
  }

  const esp_err_t result =
      httpd_queue_work(serverHandle_, sendWorkCallback, &pendingWork_);
  if (result == ESP_OK) {
    // The callback may already have run. Do not read or write pendingWork_ or
    // its lifecycle gauge after successful submission.
    return;
  }

  portENTER_CRITICAL(&lifecycleMutex_);
  lifecycle_.queueSubmissionFailed();
  markNetworkHealthDegradedLocked();
  pendingStartedUs_ = 0;
  portEXIT_CRITICAL(&lifecycleMutex_);
  refreshLifecycleMetrics();
}

bool WifiTelemetry::encodePendingPayload() {
  TelemetrySnapshot snapshot = {};
  portENTER_CRITICAL(&snapshotMutex_);
  snapshot = latest_;
  portEXIT_CRITICAL(&snapshotMutex_);
  snapshot.uptimeMs = millis();

  const uint32_t freeHeap =
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const uint32_t minimumFreeHeap = heap_caps_get_minimum_free_size(
      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const uint32_t psramFree =
      heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  const uint32_t psramMinimumFree = heap_caps_get_minimum_free_size(
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  const WifiTelemetryMetrics current = metrics();
  const uint32_t encodeStartUs = micros();
  const int written = snprintf(
      pendingWork_.payload, sizeof(pendingWork_.payload),
      "{\"timestamp_ms\":%lu,\"uptime_ms\":%lu,"
      "\"q_rad\":[%.6g,%.6g,%.6g],\"qd_rad_s\":[%.6g,%.6g,%.6g],"
      "\"target_rad\":[%.6g,%.6g,%.6g],"
      "\"acc_mps2\":[%.6g,%.6g,%.6g],"
      "\"gyro_rad_s\":[%.6g,%.6g,%.6g],"
      "\"action_norm\":[%.6g,%.6g,%.6g],"
      "\"cmd_mps\":[%.6g,%.6g],"
      "\"inference_us\":%lu,\"inference_max_us\":%lu,"
      "\"deadline_misses\":%lu,\"policy_hz\":%.6g,"
      "\"imu_configured_hz\":%lu,\"joint_configured_hz\":%lu,"
      "\"fault_code\":%u,\"fault_name\":\"%s\","
      "\"state_name\":\"%s\",\"enabled\":%s,"
      "\"command_age_ms\":%lu,\"heap_internal_free\":%lu,"
      "\"heap_internal_min\":%lu,\"psram_free\":%lu,\"psram_min\":%lu,"
      "\"telemetry_drops\":%lu,\"ws_send_failures\":%lu,"
      "\"websocket_connected\":%lu,\"websocket_connect_count\":%lu,"
      "\"websocket_disconnect_count\":%lu,\"websocket_generation\":%lu,"
      "\"httpd_queue_failures\":%lu,\"httpd_pending_sends\":%lu,"
      "\"session_close_failures\":%lu,\"httpd_pending_age_us\":%lu,"
      "\"httpd_pending_age_max_us\":%lu,"
      "\"network_health_degraded\":%s,"
      "\"pending_age_exceeded_count\":%lu,"
      "\"json_encode_us\":%lu,\"json_encode_max_us\":%lu,"
      "\"websocket_send_us\":%lu,\"websocket_send_max_us\":%lu,"
      "\"rssi\":null}",
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
      static_cast<unsigned long>(snapshot.jointConfiguredHz),
      static_cast<unsigned>(snapshot.faultCode), faultName(snapshot.faultCode),
      robotStateName(snapshot.robotState), snapshot.servosEnabled ? "true" : "false",
      static_cast<unsigned long>(snapshot.bleCommandAgeMs),
      static_cast<unsigned long>(freeHeap),
      static_cast<unsigned long>(minimumFreeHeap),
      static_cast<unsigned long>(psramFree),
      static_cast<unsigned long>(psramMinimumFree),
      static_cast<unsigned long>(current.telemetryDrops),
      static_cast<unsigned long>(current.websocketSendFailures),
      static_cast<unsigned long>(current.websocketConnected),
      static_cast<unsigned long>(current.websocketConnectCount),
      static_cast<unsigned long>(current.websocketDisconnectCount),
      static_cast<unsigned long>(current.websocketGeneration),
      static_cast<unsigned long>(current.httpdQueueFailures),
      static_cast<unsigned long>(current.httpdPendingSends),
      static_cast<unsigned long>(current.sessionCloseFailures),
      static_cast<unsigned long>(current.httpdPendingAgeUs),
      static_cast<unsigned long>(current.httpdPendingAgeMaxUs),
      current.networkHealthDegraded ? "true" : "false",
      static_cast<unsigned long>(current.pendingAgeExceededCount),
      static_cast<unsigned long>(current.jsonEncodeUs),
      static_cast<unsigned long>(current.jsonEncodeMaxUs),
      static_cast<unsigned long>(current.websocketSendUs),
      static_cast<unsigned long>(current.websocketSendMaxUs));
  const uint32_t elapsedUs = micros() - encodeStartUs;
  portENTER_CRITICAL(&metricsMutex_);
  metrics_.jsonEncodeUs = elapsedUs;
  if (elapsedUs > metrics_.jsonEncodeMaxUs) metrics_.jsonEncodeMaxUs = elapsedUs;
  portEXIT_CRITICAL(&metricsMutex_);
  if (written < 0 || static_cast<size_t>(written) >= sizeof(pendingWork_.payload)) {
    return false;
  }
  pendingWork_.payloadLength = static_cast<size_t>(written);
  return true;
}

void WifiTelemetry::sendWorkCallback(void* argument) {
  PendingWork* work = static_cast<PendingWork*>(argument);
  if (work != nullptr && work->owner != nullptr) work->owner->runSendWork(work);
}

void WifiTelemetry::runSendWork(PendingWork* work) {
  bool maySend = false;
  portENTER_CRITICAL(&lifecycleMutex_);
  maySend = lifecycle_.beginQueuedWork(work->fd, work->generation);
  portEXIT_CRITICAL(&lifecycleMutex_);

  if (maySend) {
    httpd_ws_frame_t frame = {};
    frame.final = true;
    frame.fragmented = false;
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = reinterpret_cast<uint8_t*>(work->payload);
    frame.len = work->payloadLength;
    const uint32_t sendStartUs = micros();
    const esp_err_t result =
        httpd_ws_send_frame_async(work->server, work->fd, &frame);
    const uint32_t elapsedUs = micros() - sendStartUs;

    portENTER_CRITICAL(&metricsMutex_);
    metrics_.websocketSendUs = elapsedUs;
    if (elapsedUs > metrics_.websocketSendMaxUs) {
      metrics_.websocketSendMaxUs = elapsedUs;
    }
    portEXIT_CRITICAL(&metricsMutex_);

    bool closeSession = false;
    bool successfulSession = false;
    portENTER_CRITICAL(&lifecycleMutex_);
    if (result != ESP_OK) {
      closeSession = lifecycle_.sendFailed(work->fd, work->generation);
    } else if (elapsedUs > kSlowSendThresholdUs) {
      closeSession = lifecycle_.sendTooSlow(work->fd, work->generation);
    } else {
      successfulSession =
          lifecycle_.sendSucceeded(work->fd, work->generation);
    }
    if (!successfulSession) {
      markNetworkHealthDegradedLocked();
    }
    portEXIT_CRITICAL(&lifecycleMutex_);

    if (successfulSession) {
      (void)httpd_sess_update_lru_counter(work->server, work->fd);
    } else if (closeSession) {
      requestSessionClose(work->server, work->fd, work->generation);
    }
  }

  portENTER_CRITICAL(&lifecycleMutex_);
  if (!maySend) lifecycle_.recordTelemetryDrop();
  if (pendingStartedUs_ != 0) {
    const uint32_t ageUs = micros() - pendingStartedUs_;
    if (ageUs > pendingAgeMaxUs_) pendingAgeMaxUs_ = ageUs;
  }
  lifecycle_.completeQueuedWork(work->fd, work->generation,
                                work->bufferIndex);
  pendingStartedUs_ = 0;
  observePendingHealthLocked();
  portEXIT_CRITICAL(&lifecycleMutex_);
  refreshLifecycleMetrics();
}

void WifiTelemetry::requestSessionClose(httpd_handle_t server, int fd,
                                        uint32_t generation) {
  const esp_err_t result = httpd_sess_trigger_close(server, fd);
  if (result == ESP_OK) return;

  bool useShutdown = false;
  portENTER_CRITICAL(&lifecycleMutex_);
  useShutdown = lifecycle_.closeRequestFailed(fd, generation);
  markNetworkHealthDegradedLocked();
  portEXIT_CRITICAL(&lifecycleMutex_);
  refreshLifecycleMetrics();
  if (useShutdown) (void)shutdown(fd, SHUT_RDWR);
}

esp_err_t WifiTelemetry::serveAsset(httpd_req_t* request,
                                    const char* contentType,
                                    const char* content) {
  httpd_resp_set_type(request, contentType);
  const esp_err_t result =
      httpd_resp_send(request, content, HTTPD_RESP_USE_STRLEN);
  if (result != ESP_OK && activeInstance_ != nullptr) {
    portENTER_CRITICAL(&activeInstance_->lifecycleMutex_);
    activeInstance_->markNetworkHealthDegradedLocked();
    portEXIT_CRITICAL(&activeInstance_->lifecycleMutex_);
    activeInstance_->refreshLifecycleMetrics();
  }
  return result;
}

esp_err_t WifiTelemetry::indexHandler(httpd_req_t* request) {
  return serveAsset(request, "text/html; charset=utf-8", kCrawlerIndexHtml);
}

esp_err_t WifiTelemetry::appHandler(httpd_req_t* request) {
  return serveAsset(request, "application/javascript", kCrawlerAppJs);
}

esp_err_t WifiTelemetry::styleHandler(httpd_req_t* request) {
  return serveAsset(request, "text/css", kCrawlerStyleCss);
}

esp_err_t WifiTelemetry::webSocketHandler(httpd_req_t* request) {
  WifiTelemetry* instance = activeInstance_;
  return instance == nullptr ? ESP_FAIL : instance->handleWebSocket(request);
}

esp_err_t WifiTelemetry::handleWebSocket(httpd_req_t* request) {
  const int fd = httpd_req_to_sockfd(request);
  if (request->method == HTTP_GET) {
    bool accepted = false;
    portENTER_CRITICAL(&lifecycleMutex_);
    accepted = lifecycle_.acceptClient(fd);
    if (accepted) {
      networkHealthDegraded_ = false;
    }
    portEXIT_CRITICAL(&lifecycleMutex_);
    refreshLifecycleMetrics();
    return accepted ? ESP_OK : ESP_FAIL;
  }

  httpd_ws_frame_t frame = {};
  esp_err_t result = httpd_ws_recv_frame(request, &frame, 0);
  if (result != ESP_OK) return result;

  uint32_t generation = 0;
  bool isAccepted = false;
  portENTER_CRITICAL(&lifecycleMutex_);
  const crawler::wifi::ActiveWebSocket active = lifecycle_.active();
  isAccepted = active.state == crawler::wifi::WebSocketState::Connected &&
               active.fd == fd;
  generation = active.generation;
  portEXIT_CRITICAL(&lifecycleMutex_);
  if (!isAccepted) return ESP_FAIL;

  if (frame.len > kWebSocketReceiveLimit) {
    bool closeSession = false;
    portENTER_CRITICAL(&lifecycleMutex_);
    closeSession = lifecycle_.protocolFailed(fd, generation);
    markNetworkHealthDegradedLocked();
    portEXIT_CRITICAL(&lifecycleMutex_);
    refreshLifecycleMetrics();
    if (closeSession) requestSessionClose(serverHandle_, fd, generation);
    return ESP_FAIL;
  }

  uint8_t discard[kWebSocketReceiveLimit] = {};
  if (frame.len != 0) {
    frame.payload = discard;
    result = httpd_ws_recv_frame(request, &frame, frame.len);
  }
  return result;
}

void WifiTelemetry::httpdCloseCallback(httpd_handle_t server, int sockfd) {
  WifiTelemetry* instance = activeInstance_;
  if (instance != nullptr) instance->handleClientClose(sockfd);

  // A custom close_fn replaces HTTPD's default close(), so every supplied
  // client descriptor must be closed here, including assets and rejected WS.
  if (close(sockfd) != 0 && errno != EBADF) {
    (void)server;
  }
}

void WifiTelemetry::handleClientClose(int sockfd) {
  portENTER_CRITICAL(&lifecycleMutex_);
  (void)lifecycle_.clientClosed(sockfd);
  portEXIT_CRITICAL(&lifecycleMutex_);
  refreshLifecycleMetrics();
}

#endif
