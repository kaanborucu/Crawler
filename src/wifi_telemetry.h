#pragma once

#include <atomic>
#include <stdint.h>

#include "crawler_config.h"
#include "crawler_types.h"

#ifndef CRAWLER_ENABLE_WIFI_TELEMETRY
#define CRAWLER_ENABLE_WIFI_TELEMETRY 0
#endif

#ifndef CRAWLER_WIFI_USE_IDF_HTTPD
#define CRAWLER_WIFI_USE_IDF_HTTPD 0
#endif

struct TelemetrySnapshot {
  uint32_t timestampMs;
  uint32_t uptimeMs;
  float jointPositionRad[crawler::kJointCount];
  float jointVelocityRadPerSecond[crawler::kJointCount];
  float jointTargetRad[crawler::kJointCount];
  float accelerationMps2[3];
  float gyroRadPerSecond[3];
  float policyActionNormalized[crawler::kJointCount];
  float velocityCommandMps[2];
  uint32_t inferenceTimeUs;
  uint32_t maximumInferenceTimeUs;
  uint32_t policyDeadlineMisses;
  float policyRateHz;
  uint32_t imuConfiguredHz;
  uint32_t jointConfiguredHz;
  crawler::FaultCode faultCode;
  crawler::RobotState robotState;
  bool servosEnabled;
  uint32_t bleCommandAgeMs;
};

struct WifiTelemetryMetrics {
  uint32_t networkServiceUs;
  uint32_t networkServiceMaxUs;
  uint32_t jsonEncodeUs;
  uint32_t jsonEncodeMaxUs;
  uint32_t websocketSendUs;
  uint32_t websocketSendMaxUs;
  uint32_t networkWindowOverruns;
  uint32_t telemetryDrops;
  uint32_t websocketSendFailures;
  uint32_t websocketConnected;
  uint32_t websocketConnectCount;
  uint32_t websocketDisconnectCount;
  uint32_t websocketGeneration;
  uint32_t httpdQueueFailures;
  uint32_t httpdPendingSends;
  uint32_t sessionCloseFailures;
  uint32_t httpdPendingAgeUs;
  uint32_t httpdPendingAgeMaxUs;
  uint32_t networkHealthDegraded;
  uint32_t pendingAgeExceededCount;
};

#if CRAWLER_ENABLE_WIFI_TELEMETRY

#if CRAWLER_WIFI_USE_IDF_HTTPD
#include <esp_http_server.h>
#include "wifi_config.h"
#include "websocket_lifecycle.h"
#else
#include <WebServer.h>
#include <WebSocketsServer.h>
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

class WifiTelemetry {
 public:
  explicit WifiTelemetry(
      std::atomic<bool>* policyInferenceActive = nullptr,
      std::atomic<bool>* policyControlActive = nullptr);

  // Failure is reported to Serial but is never a robot startup failure.
  bool begin();
  void publish(const TelemetrySnapshot& snapshot);
  void setPolicyControlActive(bool active);
  void runPostInferenceWindow(uint32_t availableWindowUs);
  WifiTelemetryMetrics metrics();

 private:
#if CRAWLER_WIFI_USE_IDF_HTTPD
  struct PendingWork {
    WifiTelemetry* owner;
    httpd_handle_t server;
    int fd;
    uint32_t generation;
    uint8_t bufferIndex;
    size_t payloadLength;
    char payload[crawler::config::wifi::jsonBufferSize];
  };

  static void producerTaskEntry(void* argument);
  static void sendWorkCallback(void* argument);
  static void httpdCloseCallback(httpd_handle_t server, int sockfd);
  static esp_err_t indexHandler(httpd_req_t* request);
  static esp_err_t appHandler(httpd_req_t* request);
  static esp_err_t styleHandler(httpd_req_t* request);
  static esp_err_t webSocketHandler(httpd_req_t* request);
  void producerTaskLoop();
  void produceTelemetry();
  void runSendWork(PendingWork* work);
  esp_err_t handleWebSocket(httpd_req_t* request);
  void handleClientClose(int sockfd);
  void requestSessionClose(httpd_handle_t server, int fd,
                           uint32_t generation);
  bool encodePendingPayload();
  static esp_err_t serveAsset(httpd_req_t* request, const char* contentType,
                              const char* content);
  void observePendingHealthLocked();
  void markNetworkHealthDegradedLocked();
  void refreshLifecycleMetrics();

  static WifiTelemetry* activeInstance_;
  httpd_handle_t serverHandle_;
  portMUX_TYPE snapshotMutex_;
  portMUX_TYPE metricsMutex_;
  portMUX_TYPE lifecycleMutex_;
  TaskHandle_t taskHandle_;
  TelemetrySnapshot latest_;
  WifiTelemetryMetrics metrics_;
  crawler::wifi::WebSocketLifecycle lifecycle_;
  PendingWork pendingWork_;
  uint32_t pendingStartedUs_;
  uint32_t pendingAgeMaxUs_;
  uint32_t pendingAgeExceededCount_;
  bool pendingAgeExceededForCurrentSend_;
  bool networkHealthDegraded_;
  std::atomic<bool>* policyInferenceActive_;
  std::atomic<bool>* policyControlActive_;
#else
  static void taskEntry(void* argument);
  static void webSocketEvent(uint8_t clientNumber, WStype_t type,
                             uint8_t* payload, size_t length);
  void taskLoop();
  void serviceNetworkPass(uint32_t availableWindowUs, bool postInference);
  void completePostInferenceWindow();
  void sendTelemetry();
  void recordTelemetryDrop();
  void recordWebSocketFailure();
  void recordNetworkService(uint32_t elapsedUs);
  void recordJsonEncode(uint32_t elapsedUs);
  void recordWebSocketSend(uint32_t elapsedUs);
  void recordNetworkWindowOverrun();
  void serveIndex();
  void serveApp();
  void serveStyle();

  static WifiTelemetry* activeInstance_;
  WebServer httpServer_;
  WebSocketsServer webSocketServer_;
  portMUX_TYPE snapshotMutex_;
  portMUX_TYPE metricsMutex_;
  TaskHandle_t taskHandle_;
  std::atomic<TaskHandle_t> completionTask_;
  std::atomic<bool> policyWindowRequested_;
  std::atomic<uint32_t> requestedWindowUs_;
  std::atomic<bool> windowOverrunRecorded_;
  TelemetrySnapshot latest_;
  WifiTelemetryMetrics metrics_;
  uint32_t lastTelemetryMs_;
  bool telemetrySuppressed_;
  uint8_t clientNumber_;
  bool clientConnected_;
  std::atomic<bool>* policyInferenceActive_;
  std::atomic<bool>* policyControlActive_;
#endif
};

#else

class WifiTelemetry {
 public:
  explicit WifiTelemetry(std::atomic<bool>* policyInferenceActive = nullptr,
                         std::atomic<bool>* policyControlActive = nullptr) {
    (void)policyInferenceActive;
    (void)policyControlActive;
  }
  bool begin() { return true; }
  void publish(const TelemetrySnapshot&) {}
  void setPolicyControlActive(bool) {}
  void runPostInferenceWindow(uint32_t) {}
  WifiTelemetryMetrics metrics() { return {}; }
};

#endif
