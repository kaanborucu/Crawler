# ESP32-S3 Crawler

## Project purpose

This is a single-robot ESP32-S3 crawler controller. It combines a generated
FP32 policy, BLE velocity commands, calibrated servo I/O, explicit safety
checks, and a deterministic mock mode for development without a robot.

## Hardware target

The firmware targets an ESP32-S3 DevKitC-1-class board. PlatformIO is
configured for a 16 MB flash layout and the Arduino framework; the build also
keeps the project's PSRAM configuration flags. Servos must use a separate,
adequately rated supply with a common ground. Never power servos from the
ESP32 board.

## Student-policy specification

The generated policy is kept unchanged in `src/student_policy.c`, declared by
`include/student_policy.h`, and accepts `[1,85]` FP32 observations while
returning `[1,3]` FP32 actions. The generated model owns its observation
normalization. The application performs only the documented preprocessing,
action clamp, target scale, and trained target filter around that model.

## Simplified architecture

The runtime has eight concrete pieces:

- `Crawler`: startup, absolute 50 Hz scheduling, safety coordination, and
  reduced-rate telemetry.
- `PolicyPipeline`: preprocessing, five-frame histories, observation flattening,
  generated-policy inference, finite checks, scaling, and filtering.
- `RobotIO`: compile-time-selected deterministic mock I/O or ADC/LEDC hardware
  I/O, including a 500 Hz hardware feedback task and calibration mappings.
- `ImuSensor`: MPU6050 FIFO acquisition at the configured 1 kHz sensor rate;
  the policy consumes a coherent latest snapshot at 50 Hz.
- `BleControl`: packed command parsing, sequence/timestamp handling, BLE
  service callbacks, connection state, and status notifications.
- `Safety`: readiness checks, arming, timeouts, E-stop, latched faults, and
  explicit recovery.
- `WifiTelemetry`: optional read-only SoftAP dashboard on a separate
  low-priority network task; it cannot send robot commands.
- `crawler_config.h` and `crawler_types.h`: shared constants and state types.

No generic hardware interfaces or per-transformation classes remain.

## Folder structure

```text
include/  crawler_config.h  crawler_types.h  imu_sensor.h  student_policy.h
          wifi_config.h
src/      main.cpp, Crawler, BleControl, RobotIO, PolicyPipeline, Safety,
          ImuSensor, WifiTelemetry, student_policy.c
web/      index.html  app.js  style.css
test/     test_pipeline.cpp  test_safety.cpp  test_servo_mapping.cpp
models/   acc_studentV1.onnx  acc_studentV1.json
tools/    check_onnx.py  ble_controller.py
```

## Build environments

- `esp32-s3-mock-wifi-idf-httpd-iram-cache32`: default; deterministic mock
  hardware with the read-only SoftAP dashboard and inference-deadline fault
  enforcement disabled for performance testing.
- `esp32-s3-hardware-wifi-idf-httpd-iram-cache32`: real sensors and actuators
  with the same dashboard configuration and inference-deadline enforcement
  enabled.
- `native-tests`: Windows-host tests without Arduino, ESP32, BLE, GPIO, ADC, or
  PWM dependencies.

Build and run all checks:

```powershell
& 'C:\Users\KAAN\.platformio\penv\Scripts\platformio.exe' test -e native-tests
& 'C:\Users\KAAN\.platformio\penv\Scripts\platformio.exe' run -e esp32-s3-mock-wifi-idf-httpd-iram-cache32
& 'C:\Users\KAAN\.platformio\penv\Scripts\platformio.exe' run -e esp32-s3-hardware-wifi-idf-httpd-iram-cache32
& '.\.venv-onnx\Scripts\python.exe' tools\check_onnx.py
```

## Serial monitor

The firmware uses 115200 baud. In VS Code, run **PlatformIO: Monitor**, or
use:

```powershell
& 'C:\Users\KAAN\.platformio\penv\Scripts\platformio.exe' device monitor -b 115200
```

Choose the board's COM port if PlatformIO does not detect it automatically.
Telemetry is emitted at 10 Hz (every five 50 Hz control cycles).

## BLE protocol

The device advertises as `Crawler-S3` using service UUID
`7f1f0001-9f2e-4c9c-9d53-4e2a4d4b0101`. Commands use characteristic UUID
`7f1f0002-9f2e-4c9c-9d53-4e2a4d4b0101`; status notifications use
`7f1f0003-9f2e-4c9c-9d53-4e2a4d4b0101`.

The packed command is eight bytes, little-endian `<BBhhH`:
`version`, `flags`, forward mm/s, lateral mm/s, and sequence. Flag bits are
enable `0x01`, emergency stop `0x02`, and clear fault `0x04`; reserved bits
are rejected. Each velocity is limited to ±1500 mm/s, and stale commands are
rejected after 300 ms. Sequence numbers must advance. The status packet is
`<BBBBHII`: protocol version, robot state, fault, BLE connection, last
sequence, inference time in microseconds, and missed deadlines.

The optional Windows controller supports scanning, connecting, periodic
commands, enable/dead-man operation, E-stop, status notifications, and sends
a zero/disabled command in its exit path:

```powershell
python -m pip install bleak
python tools\ble_controller.py --scan
python tools\ble_controller.py --name Crawler-S3 --enable --count 0
python tools\ble_controller.py --name Crawler-S3 --stop
```

## Observation layout and action processing

The policy observation is `[1,85]`, flattened oldest to newest with joint order
1, 2, 3:

```text
obs[0:15]   position history, five frames × three joints
obs[15:30]  velocity history, five frames × three joints
obs[30:45]  linear-acceleration history, five frames × three axes
obs[45:60]  angular-velocity history, five frames × three axes
obs[60:75]  previous clamped-action history, five frames × three joints
obs[75:85]  command history, five frames × forward/lateral
```

Position is measured radians minus the configured default, clamped to ±1.5707963
rad. Velocity is clamped to ±20 rad/s and multiplied by 0.1. MPU6050
acceleration is converted to m/s², clamped to ±50, and multiplied by 0.1;
gyro is converted to rad/s, clamped to ±20, and multiplied by 0.25. Commands
are clamped to ±1.5 m/s. At startup, all five sensor/command frames are filled
from the first valid sample and all action frames are zero. Each cycle builds
the observation before inserting the newly inferred action. Actions are
clamped to ±1, scaled by `1.4835298642`, then filtered as
`0.1 * previous + 0.9 * new`.

## MPU6050 wiring

The hardware firmware uses the MPU6050 over I²C on the existing ESP32 pins:

```text
MPU6050 VCC  -> ESP32 3V3
MPU6050 GND  -> ESP32 GND
MPU6050 SDA  -> ESP32 GPIO 8
MPU6050 SCL  -> ESP32 GPIO 9
MPU6050 AD0  -> GND for address 0x68 (or 3V3 for 0x69)
```

The driver accepts address `0x68` or `0x69`, configures the sensor's internal
1 kHz sample rate and FIFO, and drains accel+gyro packets from a dedicated
task. The MPU's DLPF is enabled at approximately 20 Hz; the policy reads the
newest timestamped snapshot at 50 Hz. It does not remove gravity or apply an
attitude transform because this exported policy has no orientation input. The
axis/sign table in `include/crawler_config.h` must match the frame used during
training before real robot motion is attempted.

## Wi-Fi telemetry

Both ESP32 environments start a read-only SoftAP using
`include/wifi_config.h`. The mock profile uses simulated joints and IMU data,
while the hardware profile accesses the connected sensors and actuator
interfaces:

```text
SSID:       Crawler-Robot
Password:   crawler123 (development value)
Dashboard:  http://192.168.4.1
WebSocket:  ws://192.168.4.1/ws
```

In FAULT or DISARMED states, the network task services HTTP/WebSocket traffic
every 10 ms and sends the newest fixed-size JSON telemetry frame every 250 ms.
During active policy control, it waits for a post-inference notification,
performs one HTTP/WebSocket service pass, and sends at most one telemetry frame
in that window. Incoming WebSocket application messages are ignored. Wi-Fi
startup or failure does not enable,
disable, or otherwise control the robot. The dashboard reports radians,
rad/s, m/s², and rad/s; SoftAP client RSSI is reported as JSON `null` because
it is not exposed by the selected simple Arduino API.

## Servo calibration

All three calibration entries are intentionally invalid (`-1` pins and
`valid=false`) until measured on the physical robot. The table contains the
PWM/ADC pins, default position, servo zero, direction, joint limits, pulse
limits, feedback endpoints, and feedback inversion. No pins, offsets,
endpoints, pulse limits, or directions are invented in this repository.

## Safety behavior

Servo output is disabled at boot and whenever the system is not explicitly in
`Running`. Invalid calibration, invalid or non-finite sensors, sensor timeout,
command timeout, BLE disconnect, non-finite policy data, inference deadline
misses, and E-stop requests prevent motion or latch a fault. Faults do not
restart motion automatically: the cause must be gone, a clear-fault command
must be received, and a fresh enable sequence must be requested.

## Hardware bring-up

1. Measure each servo's safe joint range, feedback endpoints, zero offset, and
   direction.
2. Fill the three calibration entries in `include/crawler_config.h` with those
   measured values and actual board pins.
3. Confirm the hardware build starts disarmed and verify feedback while the
   servos remain mechanically safe.
4. Validate PWM endpoints, current limits, BLE timeout behavior, E-stop, and
   recovery with externally powered servos.

## Remaining physical-validation requirements

The successful host tests and firmware builds do not validate BLE radio
behavior, ADC wiring/noise, PSRAM availability, target-board inference timing,
LEDC output, servo movement, current draw, or fail-safe behavior under real
power/cable faults. Those checks require the actual ESP32-S3 board, measured
calibration, and a safe externally powered test setup.
