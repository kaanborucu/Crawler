#include <Arduino.h>
#include <SPI.h>

#include <Adafruit_BNO08x.h>

namespace {

// ESP32-S3 FSPI pins on the DevKitC-1.
constexpr uint8_t kSpiSckPin = 12;
constexpr uint8_t kSpiMisoPin = 13;
constexpr uint8_t kSpiMosiPin = 11;
constexpr uint8_t kChipSelectPin = 10;
constexpr uint8_t kInterruptPin = 7;
constexpr int8_t kResetPin = 6;

enum class ReportTestMode : uint8_t {
  kBoth,
  kAccelerometerOnly,
  kGyroscopeOnly,
};

// Normal standalone test: both reports enabled at the highest common request.
constexpr ReportTestMode kTestMode = ReportTestMode::kBoth;

// Maximum individual report rates documented for the BNO08x.
constexpr uint32_t kAccelReportIntervalUs = 4000;  // 250 Hz
constexpr uint32_t kGyroReportIntervalUs = 2500;   // 400 Hz
constexpr uint32_t kPrintIntervalMs = 100;  // 10 serial updates per second
constexpr uint16_t kMaximumEventsPerLoop = 128;

Adafruit_BNO08x bno08x(kResetPin);
sh2_SensorValue_t sensorValue = {};

float accelerationMps2[3] = {};
float angularVelocityRadPerSecond[3] = {};
bool accelerationValid = false;
bool angularVelocityValid = false;
uint32_t lastPrintMs = 0;

struct ReportStats {
  uint32_t received = 0;
  uint32_t sequenceGaps = 0;
  uint32_t sequenceAdvances = 0;
  uint64_t firstSensorTimestampUs = 0;
  uint64_t lastSensorTimestampUs = 0;
  uint32_t timestampBacksteps = 0;
  uint32_t firstHostTimestampUs = 0;
  uint32_t lastHostTimestampUs = 0;
  uint8_t lastSequence = 0;
  uint8_t lastStatus = 0;
  bool hasPrevious = false;
};

ReportStats accelerationStats;
ReportStats gyroStats;

constexpr bool accelerometerEnabled() {
  return kTestMode != ReportTestMode::kGyroscopeOnly;
}

constexpr bool gyroEnabled() {
  return kTestMode != ReportTestMode::kAccelerometerOnly;
}

void resetReportStats(ReportStats& stats) {
  stats = {};
}

void recordReport(ReportStats& stats, const sh2_SensorValue_t& value) {
  const uint32_t hostTimestampUs = micros();

  if (!stats.hasPrevious) {
    stats.firstSensorTimestampUs = value.timestamp;
    stats.lastSensorTimestampUs = stats.firstSensorTimestampUs;
    stats.firstHostTimestampUs = hostTimestampUs;
    stats.hasPrevious = true;
  } else {
    // The sequence is an 8-bit counter, so unsigned subtraction also handles
    // its wrap from 255 back to 0.
    const uint8_t sequenceDelta =
        static_cast<uint8_t>(value.sequence - stats.lastSequence);
    if (sequenceDelta > 1) {
      stats.sequenceGaps += sequenceDelta - 1;
    }
    stats.sequenceAdvances += sequenceDelta;
  }

  ++stats.received;
  stats.lastSequence = value.sequence;
  stats.lastStatus = value.status;
  stats.lastHostTimestampUs = hostTimestampUs;

  // The SH-2 library extends its 32-bit host-time base into this 64-bit
  // timestamp. A backward step means the timestamp source is not valid for
  // this report stream; keep the last valid endpoint and count the problem.
  if (value.timestamp >= stats.lastSensorTimestampUs) {
    stats.lastSensorTimestampUs = value.timestamp;
  } else {
    ++stats.timestampBacksteps;
  }
}

float calculateHostRate(const ReportStats& stats) {
  const uint32_t elapsedUs = stats.lastHostTimestampUs -
                             stats.firstHostTimestampUs;
  if (stats.received < 2 || elapsedUs == 0) {
    return 0.0f;
  }
  return static_cast<float>(stats.received - 1) * 1000000.0f /
         static_cast<float>(elapsedUs);
}

float calculateSensorRate(const ReportStats& stats) {
  const uint64_t elapsedUs = stats.lastSensorTimestampUs -
                             stats.firstSensorTimestampUs;
  if (stats.sequenceAdvances == 0 || elapsedUs == 0) {
    return 0.0f;
  }
  return static_cast<float>(stats.sequenceAdvances) * 1000000.0f /
         static_cast<float>(elapsedUs);
}

float calculateSensorSpanMs(const ReportStats& stats) {
  if (!stats.hasPrevious || stats.lastSensorTimestampUs <
                               stats.firstSensorTimestampUs) {
    return 0.0f;
  }
  return static_cast<float>(stats.lastSensorTimestampUs -
                            stats.firstSensorTimestampUs) * 0.001f;
}

void printReportStats(const char* label, const ReportStats& stats) {
  Serial0.printf(
      "%s host=%.1fHz sensor=%.1fHz recv=%lu gaps=%lu seq=%lu "
      "status=%u ts_back=%lu sensor_ts=%llu sensor_span=%.1fms\n",
      label, calculateHostRate(stats), calculateSensorRate(stats),
      static_cast<unsigned long>(stats.received),
      static_cast<unsigned long>(stats.sequenceGaps),
      static_cast<unsigned long>(stats.sequenceAdvances),
      static_cast<unsigned int>(stats.lastStatus & 0x03),
      static_cast<unsigned long>(stats.timestampBacksteps),
      static_cast<unsigned long long>(stats.lastSensorTimestampUs),
      calculateSensorSpanMs(stats));
}

void printExpectedWiring() {
  Serial0.println("Expected GY-BNO08X SPI wiring:");
  Serial0.println("  VCC -> ESP32 3V3");
  Serial0.println("  GND -> ESP32 GND");
  Serial0.println("  SCL/SCK -> GPIO12");
  Serial0.println("  SDA/MISO -> GPIO13");
  Serial0.println("  DI/ADDR/MOSI -> GPIO11");
  Serial0.println("  CS -> GPIO10");
  Serial0.println("  INT -> GPIO7");
  Serial0.println("  RST -> GPIO6");
  Serial0.println("  PS1/P1 -> 3V3, PS0/P0 -> 3V3");
}

bool configureReports() {
  const bool accelerationConfigured =
      !accelerometerEnabled() ||
      bno08x.enableReport(SH2_ACCELEROMETER, kAccelReportIntervalUs);
  const bool gyroConfigured =
      !gyroEnabled() ||
      bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, kGyroReportIntervalUs);

  if (!accelerationConfigured || !gyroConfigured) {
    Serial0.printf("Report setup failed: accel=%s gyro=%s\n",
                   accelerationConfigured ? "ok" : "failed",
                   gyroConfigured ? "ok" : "failed");
    return false;
  }

  Serial0.printf("Test mode: %s\n",
                 kTestMode == ReportTestMode::kBoth
                     ? "accelerometer + calibrated gyro"
                     : kTestMode == ReportTestMode::kAccelerometerOnly
                         ? "accelerometer only"
                         : "calibrated gyro only");
  Serial0.printf(
      "Reports requested: accelerometer=%s (%.1f Hz), gyro=%s (%.1f Hz)\n",
                 accelerometerEnabled() ? "yes" : "no",
                 1000000.0f / kAccelReportIntervalUs,
                 gyroEnabled() ? "yes" : "no",
                 1000000.0f / kGyroReportIntervalUs);
  return true;
}

void processSensorEvent(const sh2_SensorValue_t& value) {
  switch (value.sensorId) {
    case SH2_ACCELEROMETER:
      recordReport(accelerationStats, value);
      accelerationMps2[0] = value.un.accelerometer.x;
      accelerationMps2[1] = value.un.accelerometer.y;
      accelerationMps2[2] = value.un.accelerometer.z;
      accelerationValid = true;
      break;

    case SH2_GYROSCOPE_CALIBRATED:
      recordReport(gyroStats, value);
      angularVelocityRadPerSecond[0] = value.un.gyroscope.x;
      angularVelocityRadPerSecond[1] = value.un.gyroscope.y;
      angularVelocityRadPerSecond[2] = value.un.gyroscope.z;
      angularVelocityValid = true;
      break;

    default:
      // Other reports are not requested, but ignoring one here makes the
      // diagnostic robust if the sensor emits a system/status event.
      break;
  }
}

void printLatestValues() {
  if (accelerometerEnabled()) {
    Serial0.printf("ACCEL valid=%s mps2=(%+.4f,%+.4f,%+.4f)\n",
                   accelerationValid ? "yes" : "no", accelerationMps2[0],
                   accelerationMps2[1], accelerationMps2[2]);
    printReportStats("ACCEL stats", accelerationStats);
  }

  if (gyroEnabled()) {
    Serial0.printf("GYRO valid=%s rad_s=(%+.4f,%+.4f,%+.4f) | INT=%d\n",
                   angularVelocityValid ? "yes" : "no",
                   angularVelocityRadPerSecond[0],
                   angularVelocityRadPerSecond[1], angularVelocityRadPerSecond[2],
                   digitalRead(kInterruptPin));
    printReportStats("GYRO stats", gyroStats);
  }
}

void printProductIds() {
  Serial0.printf("Product ID entries=%u\n",
                 static_cast<unsigned int>(bno08x.prodIds.numEntries));
  for (uint8_t index = 0; index < bno08x.prodIds.numEntries; ++index) {
    const sh2_ProductId_t& product = bno08x.prodIds.entry[index];
    Serial0.printf(
        "Product[%u] part=%lu version=%u.%u.%u build=%lu resetCause=%u\n",
        static_cast<unsigned int>(index),
        static_cast<unsigned long>(product.swPartNumber),
        static_cast<unsigned int>(product.swVersionMajor),
        static_cast<unsigned int>(product.swVersionMinor),
        static_cast<unsigned int>(product.swVersionPatch),
        static_cast<unsigned long>(product.swBuildNumber),
        static_cast<unsigned int>(product.resetCause));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial0.begin(115200);
  delay(500);

  Serial0.println();
  Serial0.println("GY-BNO08X SPI diagnostic");
  Serial0.printf("SPI SCK=%u MISO=%u MOSI=%u CS=%u INT=%u RST=%d\n",
                 kSpiSckPin, kSpiMisoPin, kSpiMosiPin, kChipSelectPin,
                 kInterruptPin, kResetPin);
  printExpectedWiring();

  pinMode(kChipSelectPin, OUTPUT);
  digitalWrite(kChipSelectPin, HIGH);
  pinMode(kInterruptPin, INPUT_PULLUP);
  SPI.begin(kSpiSckPin, kSpiMisoPin, kSpiMosiPin, kChipSelectPin);

  Serial0.println("Starting BNO08x over SPI...");
  if (!bno08x.begin_SPI(kChipSelectPin, kInterruptPin, &SPI)) {
    Serial0.println("BNO08x initialization FAILED");
    printExpectedWiring();
    while (true) delay(2000);
  }

  Serial0.println("BNO08x initialized successfully");
  printProductIds();
  if (!configureReports()) {
    Serial0.println("Cannot continue without requested reports");
    while (true) delay(2000);
  }

  lastPrintMs = millis();
}

void loop() {
  if (bno08x.wasReset()) {
    Serial0.println("BNO08x reset detected; re-enabling reports");
    accelerationValid = false;
    angularVelocityValid = false;
    resetReportStats(accelerationStats);
    resetReportStats(gyroStats);
    configureReports();
  }

  uint16_t eventsProcessed = 0;
  while (eventsProcessed < kMaximumEventsPerLoop &&
         bno08x.getSensorEvent(&sensorValue)) {
    processSensorEvent(sensorValue);
    ++eventsProcessed;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= kPrintIntervalMs) {
    printLatestValues();
    lastPrintMs = nowMs;
  }

  // The SPI HAL already waits for H_INTN when no packet is available. Avoid
  // adding another millisecond of latency to every polling cycle.
  yield();
}
