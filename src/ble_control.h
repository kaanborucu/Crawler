#pragma once

#include <stddef.h>

#include "crawler_config.h"
#include "crawler_types.h"

#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
class NimBLEServer;
class NimBLECharacteristic;
#endif

#pragma pack(push, 1)
struct BleCommandPacketV1 {
  uint8_t version;
  uint8_t flags;
  int16_t forwardMillimetersPerSecond;
  int16_t lateralMillimetersPerSecond;
  uint16_t sequence;
};

struct BleStatusPacketV1 {
  uint8_t protocolVersion;
  uint8_t robotState;
  uint8_t faultCode;
  uint8_t bleConnected;
  uint16_t lastSequence;
  uint32_t inferenceTimeUs;
  uint32_t missedDeadlines;
};
#pragma pack(pop)

static_assert(sizeof(BleCommandPacketV1) == 8,
              "BLE command packet must remain 8 bytes");

class BleControl {
 public:
  BleControl();

  bool begin();
  crawler::VelocityCommand latestCommand();
  bool connected() const;
  void publishStatus(const crawler::RobotStatus& status);

  // Testable callback boundary. The BLE callback only calls this parser and
  // stores the resulting command snapshot.
  bool acceptPacket(const uint8_t* data, size_t length, uint32_t receivedAtMs);
  void setConnected(bool connected);

 private:
  crawler::VelocityCommand latest_;
  bool hasLatest_;
  bool connected_;
#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
  NimBLEServer* server_;
  NimBLECharacteristic* statusCharacteristic_;
#endif
};

bool decodeBleCommandPacketV1(const uint8_t* data, size_t length,
                              uint32_t receivedAtMs,
                              crawler::VelocityCommand& command);
