#include "websocket_lifecycle.h"

#include <limits.h>

namespace crawler {
namespace wifi {

WebSocketLifecycle::WebSocketLifecycle()
    : active_{WebSocketState::Empty, -1, 0},
      pending_{-1, 0, 0, false},
      metrics_{},
      nextGeneration_(0) {}

bool WebSocketLifecycle::acceptClient(int fd) {
  if (fd < 0 || active_.state != WebSocketState::Empty) return false;
  ++nextGeneration_;
  if (nextGeneration_ == 0) ++nextGeneration_;
  active_ = {WebSocketState::Connected, fd, nextGeneration_};
  ++metrics_.websocketConnectCount;
  return true;
}

bool WebSocketLifecycle::rejectSecondClient(int fd) const {
  (void)fd;
  return active_.state != WebSocketState::Empty;
}

PreparePendingResult WebSocketLifecycle::preparePendingSend(
    uint8_t bufferIndex) {
  if (active_.state != WebSocketState::Connected) {
    return PreparePendingResult::NoConnectedClient;
  }
  if (pending_.occupied) return PreparePendingResult::PendingBusy;
  pending_ = {active_.fd, active_.generation, bufferIndex, true};
  metrics_.httpdPendingSends = 1;
  return PreparePendingResult::Prepared;
}

void WebSocketLifecycle::queueSubmissionSucceeded() {
  // Successful httpd_queue_work() is the ownership-transfer boundary. This
  // operation deliberately performs no write: the producer must not touch the
  // pending operation after the call succeeds.
}

void WebSocketLifecycle::queueSubmissionFailed() {
  if (!pending_.occupied) return;
  ++metrics_.httpdQueueFailures;
  ++metrics_.telemetryDrops;
  pending_ = {-1, 0, 0, false};
  metrics_.httpdPendingSends = 0;
}

bool WebSocketLifecycle::matches(int fd, uint32_t generation) const {
  return active_.fd == fd && active_.generation == generation;
}

bool WebSocketLifecycle::beginQueuedWork(int fd,
                                         uint32_t generation) const {
  return pending_.occupied && pending_.fd == fd &&
         pending_.generation == generation &&
         active_.state == WebSocketState::Connected &&
         matches(fd, generation);
}

bool WebSocketLifecycle::sendSucceeded(int fd, uint32_t generation) const {
  return active_.state == WebSocketState::Connected &&
         matches(fd, generation);
}

bool WebSocketLifecycle::transitionSendFailure(int fd, uint32_t generation,
                                                bool countSendFailure) {
  if (countSendFailure) ++metrics_.websocketSendFailures;
  if (countSendFailure) ++metrics_.telemetryDrops;
  if (active_.state != WebSocketState::Connected ||
      !matches(fd, generation)) {
    return false;
  }
  active_.state = WebSocketState::Closing;
  return true;
}

bool WebSocketLifecycle::sendFailed(int fd, uint32_t generation) {
  return transitionSendFailure(fd, generation, true);
}

bool WebSocketLifecycle::sendTooSlow(int fd, uint32_t generation) {
  return transitionSendFailure(fd, generation, true);
}

bool WebSocketLifecycle::protocolFailed(int fd, uint32_t generation) {
  return transitionSendFailure(fd, generation, false);
}

bool WebSocketLifecycle::closeRequestFailed(int fd, uint32_t generation) {
  ++metrics_.sessionCloseFailures;
  return active_.state == WebSocketState::Closing && matches(fd, generation);
}

bool WebSocketLifecycle::clientClosed(int fd) {
  if (active_.state == WebSocketState::Empty || active_.fd != fd) return false;
  active_.state = WebSocketState::Empty;
  active_.fd = -1;
  ++metrics_.websocketDisconnectCount;
  return true;
}

void WebSocketLifecycle::completeQueuedWork(int fd, uint32_t generation,
                                            uint8_t bufferIndex) {
  if (!pending_.occupied || pending_.fd != fd ||
      pending_.generation != generation ||
      pending_.bufferIndex != bufferIndex) {
    return;
  }
  pending_ = {-1, 0, 0, false};
  metrics_.httpdPendingSends = 0;
}

void WebSocketLifecycle::cancelProducerOwnedPending(bool countDrop) {
  if (!pending_.occupied) return;
  if (countDrop) ++metrics_.telemetryDrops;
  pending_ = {-1, 0, 0, false};
  metrics_.httpdPendingSends = 0;
}

void WebSocketLifecycle::recordTelemetryDrop() {
  ++metrics_.telemetryDrops;
}

}  // namespace wifi
}  // namespace crawler
