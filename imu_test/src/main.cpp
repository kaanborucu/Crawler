#include <Arduino.h>
#include <Wire.h>

namespace {

constexpr uint8_t kSdaPin = 8;
constexpr uint8_t kSclPin = 9;
constexpr uint32_t kSamplePeriodMs = 50;  // 20 Hz serial output

uint8_t adxlAddress = 0;
uint8_t lsm303dAddress = 0;
uint8_t gyroAddress = 0;
const char* gyroName = nullptr;
uint8_t magnetometerAddress = 0;
enum class MagnetometerKind { None, Hmc5883l, Qmc5883l };
MagnetometerKind magnetometerKind = MagnetometerKind::None;
bool bmp180Present = false;

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t* data, size_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  const size_t received = Wire.requestFrom(address, length, true);
  if (received != length) return false;
  for (size_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool probe(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void scanBus() {
  Serial0.println("I2C scan:");
  uint8_t count = 0;
  for (uint8_t address = 0x03; address <= 0x77; ++address) {
    if (probe(address)) {
      Serial0.printf("  found 0x%02X\n", address);
      ++count;
    }
  }
  if (count == 0) Serial0.println("  no devices found");
}

bool setupAdxl345() {
  const uint8_t candidates[] = {0x53, 0x1D};
  for (uint8_t candidate : candidates) {
    uint8_t id = 0;
    if (readRegisters(candidate, 0x00, &id, 1) && id == 0xE5) {
      adxlAddress = candidate;
      writeRegister(adxlAddress, 0x2D, 0x08);  // measurement mode
      writeRegister(adxlAddress, 0x31, 0x08);  // full resolution, +/-2 g
      writeRegister(adxlAddress, 0x2C, 0x0A);  // 100 Hz output data rate
      Serial0.printf("ADXL345 found at 0x%02X, ID=0x%02X\n", adxlAddress, id);
      return true;
    }
  }
  Serial0.println("ADXL345 not detected");
  return false;
}

bool setupLsm303d() {
  const uint8_t candidates[] = {0x1D, 0x1E};
  for (uint8_t candidate : candidates) {
    uint8_t id = 0;
    if (readRegisters(candidate, 0x0F, &id, 1) && id == 0x49) {
      lsm303dAddress = candidate;
      writeRegister(lsm303dAddress, 0x1F, 0x80);  // reboot internal memory
      delay(50);
      writeRegister(lsm303dAddress, 0x20, 0x5F);  // 50 Hz, BDU, XYZ enabled
      writeRegister(lsm303dAddress, 0x21, 0x00);  // +/-2 g
      writeRegister(lsm303dAddress, 0x24, 0x64);  // high-resolution magnetometer
      writeRegister(lsm303dAddress, 0x25, 0x20);  // magnetometer full scale
      writeRegister(lsm303dAddress, 0x26, 0x00);  // continuous magnetometer mode
      Serial0.printf("LSM303D found at 0x%02X, ID=0x%02X (accel+mag)\n",
                     lsm303dAddress, id);
      return true;
    }
  }
  Serial0.println("LSM303D not detected");
  return false;
}

bool setupGyro() {
  struct Candidate {
    uint8_t address;
    uint8_t whoAmI;
    const char* name;
  };
  const Candidate candidates[] = {
      {0x69, 0xD3, "L3G4200D"},
      {0x68, 0xD3, "L3G4200D"},
      {0x6B, 0xD4, "L3GD20"},
      {0x6A, 0xD7, "L3GD20"},
  };
  for (const Candidate& candidate : candidates) {
    uint8_t id = 0;
    if (readRegisters(candidate.address, 0x0F, &id, 1) &&
        id == candidate.whoAmI) {
      gyroAddress = candidate.address;
      gyroName = candidate.name;
      writeRegister(gyroAddress, 0x20, 0x0F);  // 95 Hz, XYZ enabled
      writeRegister(gyroAddress, 0x23, 0x00);  // 250 dps full scale
      Serial0.printf("%s found at 0x%02X, ID=0x%02X\n", gyroName,
                     gyroAddress, id);
      return true;
    }
  }
  Serial0.println("L3G4200D/L3GD20 not detected");
  return false;
}

bool setupMagnetometer() {
  uint8_t id[3] = {};
  if (readRegisters(0x1E, 0x0A, id, sizeof(id)) && id[0] == 'H' &&
      id[1] == '4' && id[2] == '3') {
    magnetometerAddress = 0x1E;
    magnetometerKind = MagnetometerKind::Hmc5883l;
    writeRegister(magnetometerAddress, 0x00, 0x70);  // 8-average, 15 Hz
    writeRegister(magnetometerAddress, 0x01, 0x20);  // gain
    writeRegister(magnetometerAddress, 0x02, 0x00);  // continuous mode
    Serial0.println("HMC5883L found at 0x1E, ID=H43");
    return true;
  }

  uint8_t qmcId = 0;
  if (readRegisters(0x0D, 0x0D, &qmcId, 1)) {
    magnetometerAddress = 0x0D;
    magnetometerKind = MagnetometerKind::Qmc5883l;
    writeRegister(magnetometerAddress, 0x0B, 0x01);  // set/reset period
    writeRegister(magnetometerAddress, 0x09, 0x1D);  // 200 Hz, 8 G, continuous
    writeRegister(magnetometerAddress, 0x0A, 0x00);  // normal mode
    Serial0.printf("QMC5883L found at 0x0D, ID=0x%02X\n", qmcId);
    return true;
  }

  Serial0.println("HMC5883L/QMC5883L not detected");
  return false;
}

bool setupBmp180() {
  if (!probe(0x77)) {
    Serial0.println("BMP180 not detected");
    return false;
  }
  uint8_t calibration[22] = {};
  if (!readRegisters(0x77, 0xAA, calibration, sizeof(calibration))) {
    Serial0.println("BMP180 found but calibration read failed");
    return false;
  }
  bmp180Present = true;
  Serial0.println("BMP180 found at 0x77, calibration readable");
  return true;
}

void printAdxl345() {
  if (adxlAddress == 0) return;
  uint8_t bytes[6] = {};
  if (!readRegisters(adxlAddress, 0x32, bytes, sizeof(bytes))) {
    Serial0.println("ADXL345 read failed");
    return;
  }
  const int16_t x = static_cast<int16_t>(bytes[0] | (bytes[1] << 8));
  const int16_t y = static_cast<int16_t>(bytes[2] | (bytes[3] << 8));
  const int16_t z = static_cast<int16_t>(bytes[4] | (bytes[5] << 8));
  Serial0.printf("ACC raw=%d,%d,%d approx_g=%.3f,%.3f,%.3f\n", x, y, z,
                x / 256.0f, y / 256.0f, z / 256.0f);
}

void printLsm303d() {
  if (lsm303dAddress == 0) return;

  uint8_t accelBytes[6] = {};
  if (!readRegisters(lsm303dAddress, 0x28 | 0x80, accelBytes,
                     sizeof(accelBytes))) {
    Serial0.println("LSM303D accelerometer read failed");
    return;
  }
  const int16_t ax = static_cast<int16_t>(accelBytes[0] |
                                          (accelBytes[1] << 8));
  const int16_t ay = static_cast<int16_t>(accelBytes[2] |
                                          (accelBytes[3] << 8));
  const int16_t az = static_cast<int16_t>(accelBytes[4] |
                                          (accelBytes[5] << 8));
  Serial0.printf("LSM303D_ACC raw=%d,%d,%d approx_g=%.3f,%.3f,%.3f\n",
                 ax, ay, az, ax * 0.000061f, ay * 0.000061f,
                 az * 0.000061f);

  uint8_t magBytes[6] = {};
  if (!readRegisters(lsm303dAddress, 0x08 | 0x80, magBytes,
                     sizeof(magBytes))) {
    Serial0.println("LSM303D magnetometer read failed");
    return;
  }
  const int16_t mx = static_cast<int16_t>(magBytes[0] |
                                          (magBytes[1] << 8));
  const int16_t my = static_cast<int16_t>(magBytes[2] |
                                          (magBytes[3] << 8));
  const int16_t mz = static_cast<int16_t>(magBytes[4] |
                                          (magBytes[5] << 8));
  Serial0.printf("LSM303D_MAG raw=%d,%d,%d\n", mx, my, mz);
}

void printGyro() {
  if (gyroAddress == 0) return;
  uint8_t bytes[6] = {};
  if (!readRegisters(gyroAddress, 0x28 | 0x80, bytes, sizeof(bytes))) {
    Serial0.println("L3GD20 read failed");
    return;
  }
  const int16_t x = static_cast<int16_t>(bytes[0] | (bytes[1] << 8));
  const int16_t y = static_cast<int16_t>(bytes[2] | (bytes[3] << 8));
  const int16_t z = static_cast<int16_t>(bytes[4] | (bytes[5] << 8));
  Serial0.printf("GYRO(%s) raw=%d,%d,%d approx_dps=%.3f,%.3f,%.3f\n",
                gyroName == nullptr ? "unknown" : gyroName, x, y, z,
                x * 0.00875f, y * 0.00875f, z * 0.00875f);
}

void printMagnetometer() {
  if (magnetometerAddress == 0) return;
  uint8_t bytes[6] = {};
  const uint8_t startRegister =
      magnetometerKind == MagnetometerKind::Hmc5883l ? 0x03 : 0x00;
  if (!readRegisters(magnetometerAddress, startRegister, bytes, sizeof(bytes))) {
    Serial0.println("MAG read failed");
    return;
  }

  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  if (magnetometerKind == MagnetometerKind::Hmc5883l) {
    x = static_cast<int16_t>((bytes[0] << 8) | bytes[1]);
    z = static_cast<int16_t>((bytes[2] << 8) | bytes[3]);
    y = static_cast<int16_t>((bytes[4] << 8) | bytes[5]);
  } else {
    x = static_cast<int16_t>(bytes[0] | (bytes[1] << 8));
    y = static_cast<int16_t>(bytes[2] | (bytes[3] << 8));
    z = static_cast<int16_t>(bytes[4] | (bytes[5] << 8));
  }
  Serial0.printf("MAG raw=%d,%d,%d\n", x, y, z);
}

void printBmp180() {
  if (!bmp180Present) return;
  if (!writeRegister(0x77, 0xF4, 0x2E)) return;
  delay(5);
  uint8_t bytes[2] = {};
  if (!readRegisters(0x77, 0xF6, bytes, sizeof(bytes))) return;
  const uint16_t rawTemperature =
      static_cast<uint16_t>((bytes[0] << 8) | bytes[1]);
  Serial0.printf("BMP180 raw_temperature=%u\n", rawTemperature);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial0.begin(115200);
  delay(500);
  Serial0.println("GY-89 serial output OK");
  Serial0.flush();
  Wire.begin(kSdaPin, kSclPin, 400000);
  Serial0.printf("GY-89 test started, SDA=%u SCL=%u\n", kSdaPin, kSclPin);
  scanBus();
  const bool lsm303dPresent = setupLsm303d();
  if (!lsm303dPresent) setupAdxl345();
  setupGyro();
  if (!lsm303dPresent) setupMagnetometer();
  setupBmp180();
}

void loop() {
  if (lsm303dAddress != 0) {
    printLsm303d();
  } else {
    printAdxl345();
  }
  printGyro();
  printMagnetometer();
  printBmp180();
  delay(kSamplePeriodMs);
}
