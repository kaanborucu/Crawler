#include "ble_control.h"

#include <string.h>

#if !defined(ARDUINO)
#include <chrono>
#endif

#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#endif

namespace {

crawler::VelocityCommand zeroCommand() {
  crawler::VelocityCommand command = {0.0f, 0.0f, false, false, false,
                                      false, 0, 0};
  return command;
}

uint32_t monotonicMs() {
#if defined(ARDUINO)
  return millis();
#else
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now.time_since_epoch())
                                  .count());
#endif
}

bool sequenceNewer(uint16_t candidate, uint16_t previous) {
  return static_cast<int16_t>(candidate - previous) > 0;
}

#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
portMUX_TYPE gBleMux = portMUX_INITIALIZER_UNLOCKED;

class ServerCallbacks final : public NimBLEServerCallbacks {
 public:
  BleControl* owner = nullptr;

  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    if (owner != nullptr) owner->setConnected(true);
  }

  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    if (owner != nullptr) owner->setConnected(false);
    NimBLEDevice::startAdvertising();
  }
};

class CommandCallbacks final : public NimBLECharacteristicCallbacks {
 public:
  BleControl* owner = nullptr;

  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
    if (owner == nullptr || characteristic == nullptr) return;
    const std::string value = characteristic->getValue();
    owner->acceptPacket(reinterpret_cast<const uint8_t*>(value.data()),
                        value.size(), millis());
  }
};

ServerCallbacks gServerCallbacks;
CommandCallbacks gCommandCallbacks;
#endif

}  // namespace

bool decodeBleCommandPacketV1(const uint8_t* data, size_t length,
                              uint32_t receivedAtMs,
                              crawler::VelocityCommand& command) {
  if (data == nullptr || length != sizeof(BleCommandPacketV1)) return false;
  if (data[0] != crawler::config::ble::protocolVersion ||
      (data[1] & 0xF8u) != 0u) {
    return false;
  }
  const int16_t forward = static_cast<int16_t>(
      static_cast<uint16_t>(data[2]) | (static_cast<uint16_t>(data[3]) << 8));
  const int16_t lateral = static_cast<int16_t>(
      static_cast<uint16_t>(data[4]) | (static_cast<uint16_t>(data[5]) << 8));
  if (forward < -1500 || forward > 1500 || lateral < -1500 || lateral > 1500) {
    return false;
  }
  const uint16_t sequence = static_cast<uint16_t>(data[6]) |
                            (static_cast<uint16_t>(data[7]) << 8);
  command.forwardMetersPerSecond = static_cast<float>(forward) * 0.001f;
  command.lateralMetersPerSecond = static_cast<float>(lateral) * 0.001f;
  command.enableRequested = (data[1] & 0x01u) != 0u;
  command.emergencyStop = (data[1] & 0x02u) != 0u;
  command.clearFaultRequested = (data[1] & 0x04u) != 0u;
  command.valid = true;
  command.sequence = sequence;
  command.receivedAtMs = receivedAtMs;
  return true;
}

BleControl::BleControl()
    : latest_(zeroCommand()), hasLatest_(false), connected_(false)
#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
      , server_(nullptr), statusCharacteristic_(nullptr)
#endif
{
#if CRAWLER_USE_MOCK_HARDWARE
  latest_.valid = true;
  latest_.enableRequested = true;
  connected_ = true;
#endif
}

bool BleControl::begin() {
#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
  NimBLEDevice::init(crawler::config::ble::deviceName);
  server_ = NimBLEDevice::createServer();
  gServerCallbacks.owner = this;
  server_->setCallbacks(&gServerCallbacks);
  NimBLEService* service = server_->createService(
      crawler::config::ble::serviceUuid);
  NimBLECharacteristic* commandCharacteristic = service->createCharacteristic(
      crawler::config::ble::commandUuid,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  gCommandCallbacks.owner = this;
  commandCharacteristic->setCallbacks(&gCommandCallbacks);
  statusCharacteristic_ = service->createCharacteristic(
      crawler::config::ble::statusUuid,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  server_->start();
  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(crawler::config::ble::serviceUuid);
  advertising->start();
#endif
  return true;
}

crawler::VelocityCommand BleControl::latestCommand() {
#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
  portENTER_CRITICAL(&gBleMux);
#endif
  const bool hasLatest = hasLatest_;
  const crawler::VelocityCommand command = latest_;
#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
  portEXIT_CRITICAL(&gBleMux);
#endif

#if CRAWLER_USE_MOCK_HARDWARE
  if (!hasLatest) {
    crawler::VelocityCommand mock = command;
    mock.receivedAtMs = monotonicMs();
    return mock;
  }
#endif
  if (!hasLatest) return zeroCommand();
  crawler::VelocityCommand result = command;
  if (monotonicMs() - result.receivedAtMs >
      crawler::config::safety::commandTimeoutMs) {
    result.valid = false;
  }
  return result;
}

bool BleControl::connected() const {
#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&gBleMux));
  const bool connected = connected_;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&gBleMux));
  return connected;
#else
  return connected_;
#endif
}

void BleControl::publishStatus(const crawler::RobotStatus& status) {
#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
  if (statusCharacteristic_ == nullptr) return;
  BleStatusPacketV1 packet = {
      crawler::config::ble::protocolVersion,
      static_cast<uint8_t>(status.state),
      static_cast<uint8_t>(status.fault),
      static_cast<uint8_t>(connected() ? 1u : 0u),
      status.lastCommandSequence,
      status.inferenceTimeUs,
      status.missedDeadlines,
  };
  statusCharacteristic_->setValue(reinterpret_cast<uint8_t*>(&packet),
                                  sizeof(packet));
  statusCharacteristic_->notify();
#else
  (void)status;
#endif
}

bool BleControl::acceptPacket(const uint8_t* data, size_t length,
                              uint32_t receivedAtMs) {
  crawler::VelocityCommand parsed = zeroCommand();
  if (!decodeBleCommandPacketV1(data, length, receivedAtMs, parsed)) return false;
#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
  portENTER_CRITICAL(&gBleMux);
#endif
  if (hasLatest_ && !sequenceNewer(parsed.sequence, latest_.sequence)) {
#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
    portEXIT_CRITICAL(&gBleMux);
#endif
    return false;
  }
  latest_ = parsed;
  hasLatest_ = true;
#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
  portEXIT_CRITICAL(&gBleMux);
#endif
  return true;
}

void BleControl::setConnected(bool connected) {
#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
  portENTER_CRITICAL(&gBleMux);
#endif
  connected_ = connected;
#if defined(ARDUINO) && CRAWLER_ENABLE_BLE
  portEXIT_CRITICAL(&gBleMux);
#endif
}
