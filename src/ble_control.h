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

// Version 2 adds a control mode and three raw joint-position offsets in
// milliradians. The packet remains integer-only for deterministic BLE size.
struct BleCommandPacketV2 {
  uint8_t version;
  uint8_t flags;
  int16_t forwardMillimetersPerSecond;
  int16_t lateralMillimetersPerSecond;
  int16_t rawPositionMilliradians[crawler::kJointCount];
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
static_assert(sizeof(BleCommandPacketV2) == 14,
              "BLE command packet v2 must remain 14 bytes");

namespace ble_flags {
constexpr uint8_t enable = 0x01;
constexpr uint8_t emergencyStop = 0x02;
constexpr uint8_t clearFault = 0x04;
constexpr uint8_t rawPositionMode = 0x08;
constexpr uint8_t scriptedSweepMode = 0x10;
constexpr uint8_t modeMask = rawPositionMode | scriptedSweepMode;
}

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

bool decodeBleCommandPacketV2(const uint8_t* data, size_t length,
                              uint32_t receivedAtMs,
                              crawler::VelocityCommand& command);
