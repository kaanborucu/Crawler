#pragma once

#include <stdint.h>

namespace crawler {
namespace wifi {

enum class WebSocketState : uint8_t { Empty, Connected, Closing };

struct ActiveWebSocket {
  WebSocketState state;
  int fd;
  uint32_t generation;
};

struct PendingWebSocketSend {
  int fd;
  uint32_t generation;
  uint8_t bufferIndex;
  bool occupied;
};

struct WebSocketLifecycleMetrics {
  uint32_t telemetryDrops;
  uint32_t websocketSendFailures;
  uint32_t websocketConnectCount;
  uint32_t websocketDisconnectCount;
  uint32_t httpdQueueFailures;
  uint32_t httpdPendingSends;
  uint32_t sessionCloseFailures;
};

enum class PreparePendingResult : uint8_t {
  Prepared,
  NoConnectedClient,
  PendingBusy
};

// Pure lifecycle and ownership logic. Callers serialize access and perform all
// ESP-IDF socket operations in the HTTPD adapter, outside this type.
class WebSocketLifecycle {
 public:
  WebSocketLifecycle();

  bool acceptClient(int fd);
  bool rejectSecondClient(int fd) const;
  PreparePendingResult preparePendingSend(uint8_t bufferIndex);
  void queueSubmissionSucceeded();
  void queueSubmissionFailed();
  bool beginQueuedWork(int fd, uint32_t generation) const;
  bool sendSucceeded(int fd, uint32_t generation) const;
  bool sendFailed(int fd, uint32_t generation);
  bool sendTooSlow(int fd, uint32_t generation);
  bool protocolFailed(int fd, uint32_t generation);
  bool closeRequestFailed(int fd, uint32_t generation);
  bool clientClosed(int fd);
  void completeQueuedWork(int fd, uint32_t generation, uint8_t bufferIndex);
  void cancelProducerOwnedPending(bool countDrop);
  void recordTelemetryDrop();

  ActiveWebSocket active() const { return active_; }
  PendingWebSocketSend pending() const { return pending_; }
  WebSocketLifecycleMetrics metrics() const { return metrics_; }

 private:
  bool transitionSendFailure(int fd, uint32_t generation,
                             bool countSendFailure);
  bool matches(int fd, uint32_t generation) const;

  ActiveWebSocket active_;
  PendingWebSocketSend pending_;
  WebSocketLifecycleMetrics metrics_;
  uint32_t nextGeneration_;
};

}  // namespace wifi
}  // namespace crawler
