#pragma once

#include <stdint.h>

namespace crawler {
namespace config {
namespace wifi {

constexpr char apSsid[] = "Crawler-Robot";
// Development-only password. Change it before using the AP outside testing.
constexpr char apPassword[] = "crawler123";
constexpr uint8_t apChannel = 1;
constexpr uint8_t apMaxClients = 1;
// Keep Wi-Fi diagnostic traffic low priority and infrequent so it does not
// compete with the control loop. The dashboard remains responsive enough for
// monitoring, but is intentionally not a high-rate control channel.
constexpr uint32_t networkServiceIntervalMs = 10;
constexpr uint32_t telemetryPeriodMs = 250;
constexpr uint32_t jsonBufferSize = 3072;
constexpr uint32_t pendingSendAgeLimitUs = 100000;
constexpr uint32_t taskStackSize = 6144;
constexpr uint8_t taskPriority = 1;
constexpr int8_t taskCore = 0;

static_assert(sizeof(apPassword) - 1 >= 8,
              "SoftAP WPA2 password must contain at least 8 characters");

}  // namespace wifi
}  // namespace config
}  // namespace crawler
