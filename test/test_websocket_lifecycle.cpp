#include <cstdio>

#include "websocket_lifecycle.h"

namespace websocket_lifecycle_tests {

void expect(bool condition, const char* message, int& failures) {
  if (!condition) {
    std::fprintf(stderr, "websocket lifecycle: %s\n", message);
    ++failures;
  }
}

}  // namespace websocket_lifecycle_tests

int runWebSocketLifecycleTests() {
  using crawler::wifi::PreparePendingResult;
  using crawler::wifi::WebSocketLifecycle;
  using crawler::wifi::WebSocketState;
  using namespace websocket_lifecycle_tests;
  int failures = 0;

  WebSocketLifecycle queued;
  expect(queued.acceptClient(4), "initial client was not accepted", failures);
  expect(queued.preparePendingSend(0) == PreparePendingResult::Prepared,
         "pending send was not prepared", failures);
  const auto queuedWork = queued.pending();
  queued.queueSubmissionSucceeded();
  expect(queued.pending().occupied &&
             queued.metrics().httpdPendingSends == 1,
         "successful queue submission changed producer state", failures);
  expect(queued.beginQueuedWork(queuedWork.fd, queuedWork.generation),
         "valid queued work was skipped", failures);
  queued.completeQueuedWork(queuedWork.fd, queuedWork.generation,
                            queuedWork.bufferIndex);
  expect(!queued.pending().occupied &&
             queued.metrics().httpdPendingSends == 0,
         "queued callback did not release its slot", failures);

  expect(queued.preparePendingSend(0) == PreparePendingResult::Prepared,
         "queue-failure setup failed", failures);
  queued.queueSubmissionFailed();
  expect(!queued.pending().occupied &&
             queued.metrics().httpdQueueFailures == 1 &&
             queued.metrics().telemetryDrops == 1,
         "producer did not clean up failed queue submission", failures);

  WebSocketLifecycle closeBeforeWork;
  expect(closeBeforeWork.acceptClient(7), "close test client rejected", failures);
  expect(closeBeforeWork.preparePendingSend(0) ==
             PreparePendingResult::Prepared,
         "close test send not prepared", failures);
  const auto stale = closeBeforeWork.pending();
  expect(closeBeforeWork.clientClosed(7), "accepted close not finalized", failures);
  expect(closeBeforeWork.pending().occupied,
         "close callback released queued buffer", failures);
  expect(!closeBeforeWork.beginQueuedWork(stale.fd, stale.generation),
         "closed connection was considered sendable", failures);
  closeBeforeWork.completeQueuedWork(stale.fd, stale.generation,
                                     stale.bufferIndex);
  expect(!closeBeforeWork.pending().occupied,
         "skipped work did not release its buffer", failures);

  expect(closeBeforeWork.acceptClient(7), "reused descriptor was rejected", failures);
  const auto replacement = closeBeforeWork.active();
  expect(replacement.generation != stale.generation,
         "descriptor reuse did not advance generation", failures);
  expect(!closeBeforeWork.beginQueuedWork(stale.fd, stale.generation),
         "old work matched a reused descriptor", failures);
  expect(closeBeforeWork.active().generation == replacement.generation,
         "old work changed the replacement session", failures);

  WebSocketLifecycle slow;
  expect(slow.acceptClient(9), "slow client rejected", failures);
  expect(slow.preparePendingSend(0) == PreparePendingResult::Prepared,
         "slow send setup failed", failures);
  const auto slowWork = slow.pending();
  expect(slow.sendTooSlow(slowWork.fd, slowWork.generation),
         "slow send did not request closure", failures);
  expect(slow.active().state == WebSocketState::Closing &&
             slow.metrics().websocketSendFailures == 1 &&
             slow.metrics().telemetryDrops == 1,
         "slow send did not enter closing with counters", failures);
  expect(slow.preparePendingSend(0) ==
             PreparePendingResult::NoConnectedClient,
         "telemetry was accepted while closing", failures);
  expect(slow.closeRequestFailed(slowWork.fd, slowWork.generation),
         "close failure did not request shutdown fallback", failures);
  expect(slow.active().state == WebSocketState::Closing &&
             slow.metrics().sessionCloseFailures == 1,
         "close failure did not preserve closing state", failures);
  slow.completeQueuedWork(slowWork.fd, slowWork.generation,
                          slowWork.bufferIndex);
  expect(slow.clientClosed(9), "later close did not finalize session", failures);
  expect(!slow.clientClosed(9) &&
             slow.metrics().websocketDisconnectCount == 1,
         "disconnect was counted more than once", failures);

  WebSocketLifecycle second;
  expect(second.acceptClient(11), "second-client setup failed", failures);
  const auto first = second.active();
  expect(second.rejectSecondClient(12) && !second.acceptClient(12),
         "second connected client was accepted", failures);
  expect(second.active().fd == first.fd &&
             second.active().generation == first.generation &&
             second.metrics().websocketConnectCount == 1,
         "second client changed accepted connection", failures);
  expect(second.protocolFailed(first.fd, first.generation),
         "protocol failure did not enter closing", failures);
  expect(second.rejectSecondClient(13) && !second.acceptClient(13),
         "second client was accepted while closing", failures);
  expect(!second.clientClosed(42),
         "asset close changed accepted WebSocket", failures);
  expect(second.metrics().websocketDisconnectCount == 0,
         "asset close changed disconnect counter", failures);
  expect(second.clientClosed(11), "accepted session did not close", failures);
  expect(!second.clientClosed(11),
         "already-invalid descriptor was finalized twice", failures);

  return failures;
}
