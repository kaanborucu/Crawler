# ESP32-S3 Crawler Controller

This repository contains the firmware and host-side controller for a three-joint
robot crawler. It runs on an ESP32-S3 and combines:

- a generated 50 Hz reinforcement-learning policy;
- a GY-BN008X/BNO085 IMU over SPI;
- three PWM servos with analog position feedback;
- BLE commands from a PC gamepad;
- a read-only Wi-Fi telemetry dashboard;
- E-stop, calibration, timeout, sensor-freshness, and joint-limit safety logic;
- a deterministic mock-hardware build for development without a robot.

The intended bring-up order is: compile and test the software, test the BNO085
independently, calibrate the servos, test manual position control with the robot
lifted, and only then enable policy control.

## Safety first

Servo power is external to the ESP32. Never power the servos from the ESP32
3V3 pin or USB. Use a supply rated for the servos, connect the supply ground to
ESP32 GND, and use a fuse or current limit during bring-up.

Keep the robot mechanically supported and away from people while testing.
The firmware disables servo output unless all required safety conditions are
healthy. A software E-stop is not a substitute for a physical power cut or
emergency-stop circuit.

## Repository layout

~~~text
include/       Shared configuration, data types, IMU declarations, policy API
src/           Main firmware implementation
tools/         Python BLE/gamepad bridge and policy utilities
bno08x_test/   Standalone BNO085 SPI test application
models/        ONNX policy files and metadata
test/          Host-side C++ test sources
config/        ESP32 partition and SDK configuration files
web/           Dashboard source assets
platformio.ini Main PlatformIO project configuration
~~~

The generated policy used by the firmware is src/student_policy.c, with its
interface in include/student_policy.h. The ONNX files in models/ are useful for
inspection and reference checking; the ESP32 runs the generated C policy.

## Hardware

The main firmware targets an ESP32-S3 DevKitC-1-class board with the project's
16 MB flash layout. The hardware build uses Arduino plus ESP-IDF and enables BLE
and the Wi-Fi telemetry dashboard.

### BNO085 / GY-BN008X SPI wiring

The current firmware uses SPI at 3 MHz:

| GY-BN008X pin/label | ESP32-S3 connection | Purpose |
|---|---:|---|
| VCC or 3V3 | 3V3 | Sensor power; follow the breakout voltage rating |
| GND | GND | Common ground |
| SCK/SCL | GPIO12 | SPI clock |
| MISO/SDA | GPIO13 | SPI data from sensor |
| MOSI/DI/ADDR | GPIO11 | SPI data to sensor |
| CS/NCS | GPIO10 | SPI chip select |
| INT/H_INTN | GPIO7 | Active-low data-ready interrupt |
| RST/RSTN | GPIO6 | Sensor reset |
| PS0/P0 | 3V3 | Selects the configured SPI host mode |
| PS1/P1 | 3V3 | Selects the configured SPI host mode |
| BT, if absent | No connection required | Not used by this firmware |

The labels vary between GY-BN008X boards. The firmware does not use an
MPU6050-style AD0/ADO address pin. Do not substitute the old MPU6050 I2C
wiring. Verify the board silkscreen and schematic before applying power.

The normal BNO085 data path is interrupt-driven:

~~~text
GPIO7 INT
  -> minimal ISR
  -> FreeRTOS task notification
  -> dedicated IMU task
  -> bounded SPI event draining
~~~

The ISR performs no SPI transaction, parsing, allocation, or logging. A small
periodic wake-up remains only as a recovery fallback for a missed interrupt.

The firmware requests:

- calibrated acceleration including gravity: 250 Hz;
- calibrated gyroscope: 400 Hz;
- policy/control loop: 50 Hz.

The policy receives the newest valid IMU snapshot at each 50 Hz cycle. It does
not resample the IMU or intentionally add a delay for phase alignment.

### Servo and position-feedback wiring

The current default calibration entries use these ESP32 pins:

| Joint | PWM output | Analog feedback |
|---:|---:|---:|
| 1 | GPIO14 | GPIO1 / ADC1 |
| 2 | GPIO15 | GPIO2 / ADC2 |
| 3 | GPIO16 | GPIO4 / ADC4 |

Connect each servo signal wire to its PWM pin and each feedback output to its
corresponding ADC pin. Connect ESP32 GND, servo-supply GND, and sensor GND
together. Do not connect a high-voltage servo supply to an ESP32 GPIO.

The current calibration geometry is:

| Quantity | Value |
|---|---:|
| Servo travel | 160 degrees |
| Servo angle minimum | 0 degrees |
| Robot neutral servo angle | 80 degrees |
| Servo angle maximum | 160 degrees |
| Minimum pulse | 631 us |
| Neutral pulse | 1520 us |
| Maximum pulse | 2409 us |
| PWM frequency | 50 Hz |
| Robot joint range | approximately -80 to +80 degrees |

The calibration routine measures 17 positions at 10-degree increments from 0 to
160 degrees. It averages a trimmed set of ADC samples at every point and stores
the resulting tables in ESP32 NVS flash.

## Installing the software

### Windows

Install Git for Windows, Python 3 with the py launcher, and the USB serial/JTAG
driver required by the specific ESP32-S3 board.

~~~powershell
git --version
py --version
git clone https://github.com/kaanborucu/Crawler.git
Set-Location Crawler
py -m pip install --user --upgrade platformio
$pio = "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"
& $pio --version
py -m pip install bleak pygame
~~~

If pio is already on PATH, use pio instead of the full executable path.

### Ubuntu/Debian Linux

~~~bash
sudo apt update
sudo apt install -y git python3 python3-pip python3-venv
git clone https://github.com/kaanborucu/Crawler.git
cd Crawler
python3 -m pip install --user --upgrade platformio
python3 -m pip install --user bleak pygame
~~~

If the board cannot be opened, install PlatformIO's udev rules:

~~~bash
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
  | sudo tee /etc/udev/rules.d/99-platformio-udev.rules >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -a -G dialout "$USER"
~~~

Log out and back in after changing the group membership.

On Linux, use the same PlatformIO commands shown below with pio in place of
the PowerShell variable and use /dev/ttyACM0 or the port reported by pio device
list.

## Building and uploading

From the repository root, the default environment is the mock firmware:

~~~powershell
& $pio run
~~~

The mock build uses simulated joint and IMU data. It is useful for checking
startup, BLE parsing, policy execution, telemetry, and safety logic, but it
does not operate real servos or read a real BNO085.

Build the real hardware firmware:

~~~powershell
& $pio run -e esp32-s3-hardware-wifi-idf-httpd-iram-cache32
~~~

Find the serial port:

~~~powershell
& $pio device list
~~~

Upload the hardware firmware only after checking wiring and keeping the robot
safe:

~~~powershell
& $pio run -e esp32-s3-hardware-wifi-idf-httpd-iram-cache32 -t upload --upload-port COM5
~~~

Replace COM5 with the detected port. On Linux, use a device such as
/dev/ttyACM0.

Open the serial monitor:

~~~powershell
& $pio device monitor --port COM5 --baud 115200
~~~

Close the monitor before uploading if the port is busy. The firmware uses
115200 baud and prints main telemetry at approximately 5 Hz.

The available main environments are:

| Environment | Hardware behavior |
|---|---|
| esp32-s3-mock-wifi-idf-httpd-iram-cache32 | Simulated sensors/joints; default |
| esp32-s3-hardware-wifi-idf-httpd-iram-cache32 | Real BNO085, ADC feedback, and servos |
| native-tests | Host-side C++ test configuration |

## Independent BNO085 test

Before testing the entire crawler, the standalone application in bno08x_test/
can verify sensor wiring, product identification, report setup, timestamps,
sequence gaps, and host-versus-sensor rates.

~~~powershell
Set-Location .\bno08x_test
& $pio run
& $pio run -t upload --upload-port COM5
& $pio device monitor --port COM5 --baud 115200
Set-Location ..
~~~

Healthy output should show valid acceleration and gyro reports, stable sensor
timestamps, zero or very small sequence-gap growth, and rates close to the
requested rates. Do not treat the host print rate as the sensor production
rate.

## PC BLE and gamepad control

The ESP32 advertises as:

~~~text
BLE name:    Crawler-S3
Service:     7f1f0001-9f2e-4c9c-9d53-4e2a4d4b0101
Command:     7f1f0002-9f2e-4c9c-9d53-4e2a4d4b0101
Status:      7f1f0003-9f2e-4c9c-9d53-4e2a4d4b0101
~~~

The Python bridge communicates over BLE and does not open the ESP32 serial
port. For an 8BitDo controller, use the receiver's X mode so Windows exposes
it as an XInput gamepad. DualSense and other standard XInput-compatible devices
use the same mapping.

Check the controller without connecting to the robot:

~~~powershell
py tools\ble_controller.py --list-gamepads
py tools\ble_controller.py --gamepad-test --count 0
~~~

If multiple gamepads are connected, select one explicitly:

~~~powershell
py tools\ble_controller.py --gamepad-test --gamepad-index 0 --count 0
~~~

Scan for the ESP32:

~~~powershell
py tools\ble_controller.py --scan
~~~

Run the continuous bridge:

~~~powershell
py tools\ble_controller.py --gamepad --name Crawler-S3 --count 0 --period 0.02
~~~

The bridge sends commands every 20 ms, or approximately 50 Hz. On exit it sends
a disabled command as a dead-man action. If the BLE name is not reported, use
the address from the scan:

~~~powershell
py tools\ble_controller.py --gamepad --address XX:XX:XX:XX:XX:XX --count 0 --period 0.02
~~~

### Button mapping

| Controller input | Pygame index | Action |
|---|---:|---|
| A / Cross | 0 | Clear the latched E-stop or fault; only clear path |
| B / Circle | 1 | Run the simultaneous calibrated position sweep while held |
| X / Square | 2 | Select and enable policy mode |
| Y / Triangle | 3 | Select and enable manual position mode |
| L1 / left bumper | 4 | E-stop; hold current pose while fresh stop packets arrive |
| R1 / right bumper | 5 | Start calibration, but only from E-stop |
| D-pad Up | hat 0, y=+1 | Command all joints to physical servo 90 degrees |

A only clears a latched E-stop/fault; it does not bypass sensor, calibration,
BLE, or command checks. After clearing, the robot remains disarmed until a
valid enabled mode command is accepted.

The mode selected with X or Y remains selected until another mode is selected.
B is a temporary sweep command. D-pad Up is a position-mode recovery command
for moving away from the calibrated edge position.

### Analog-stick mapping in manual position mode

| Stick | Joint command |
|---|---|
| Left stick X | Joint 1 raw position offset |
| Left stick Y | Joint 2 raw position offset; forward stick is positive |
| Right stick X | Joint 3 raw position offset |

The default bridge dead zone is 0.12. The default full-stick command is plus or
minus 1.0 rad. The command can be changed up to plus or minus 1.571 rad with
the --max-position option, but calibrated physical limits still clamp the final
PWM output:

~~~powershell
py tools\ble_controller.py --gamepad --count 0 --max-position 1.396
~~~

1.396 rad is approximately 80 degrees.

## Calibration procedure

Calibration is required for real hardware. The pulse range, servo zero, joint
limits, and ADC feedback tables are stored together in NVS flash.

1. Put the crawler in E-stop.
2. Press or hold R1 through the gamepad bridge.
3. All three servos move to 0 degrees simultaneously.
4. The firmware waits one second.
5. Joint 1 is sampled at 0, 10, ..., 160 degrees.
6. Joint 1 moves to neutral 80 degrees before joint 2 begins.
7. Joint 2 is sampled, then moves to neutral before joint 3 begins.
8. Joint 3 is sampled.
9. ADC tables are checked for monotonicity.
10. Calibration is saved to NVS and all joints return to neutral 80 degrees.

During calibration, policy and manual motion are disabled. Calibration is
aborted if BLE is lost or E-stop is asserted again.

Expected serial messages include:

~~~text
CAL START: 3 joints, 17 points, 0-160 degrees
CAL initial pose applied: all joints at 0 degrees; waiting 1000 ms
CAL sample joint=... angle_deg=... raw_adc=...
CAL ADC tables validated; saving to NVS
CAL NVS save OK
CAL neutral pose applied: all joints at 80 degrees
Servo calibration saved to NVS and activated
~~~

The current firmware uses calibration NVS version 4. Older tables are
intentionally rejected after pulse-range or angle-table changes, so calibration
must be run again.

After successful calibration, release R1, press A once to clear E-stop, then
select X or Y for the desired control mode.

## Runtime state machine

| State | Meaning |
|---|---|
| BOOTING | Firmware and devices are starting |
| DISARMED | Servo output is disabled; no motion command is applied |
| RUNNING | A valid enabled policy, position, or sweep command is active |
| ESTOP | Latched emergency stop; A is required to clear it |
| FAULT | A persistent safety or data fault is latched |

The safety supervisor checks calibration, BLE connection, command freshness,
joint feedback, IMU validity, finite numeric values, measured joint limits,
and E-stop state. A short IMU freshness failure immediately prevents policy
use and disarms that cycle. The existing 300 ms sensor timeout is needed before
persistent IMU failure becomes a latched SENSOR_INVALID fault.

The per-stream IMU freshness limits are 20 ms for acceleration and 10 ms for
gyro. ImuState.valid becomes false immediately if either stream is stale, has
not produced a valid sample, or reset recovery is active.

Commands older than 300 ms, a BLE disconnect, non-finite data, invalid
calibration, and measured motion outside configured joint limits remain safety
failures. A policy result taking more than 40 ms is counted and skipped; it is
not applied, not added to policy history, and does not create an inference
deadline fault. The measured joint pose is held for that cycle.

## Policy data path

The policy runs at 50 Hz and consumes 85 values. The five history frames are
ordered oldest to newest:

~~~text
obs[ 0:15]  joint position history
obs[15:30]  joint velocity history
obs[30:45]  calibrated acceleration history
obs[45:60]  calibrated gyro history
obs[60:75]  previous accepted action history
obs[75:85]  forward/lateral command history
~~~

Preprocessing currently uses:

- position: measured joint angle minus default, clamped to plus or minus 1.5708 rad;
- velocity: finite-difference joint velocity, clamped to plus or minus 20 rad/s,
  scaled by 0.1; there is no dedicated velocity low-pass filter;
- acceleration: m/s², clamped to plus or minus 50, scaled by 0.1;
- gyro: rad/s, clamped to plus or minus 20, scaled by 0.25;
- command: forward/lateral velocity clamped to plus or minus 1.5 m/s;
- action: clamped to plus or minus 1 and scaled by 1.48353 rad.

The final policy target filter is:

~~~text
filtered_target = 0.1 * previous_target + 0.9 * new_target
~~~

The newest accepted action is committed to the next observation only after the
policy result passes timing and finite-value checks.

## Wi-Fi telemetry dashboard

Both ESP32 environments enable a read-only SoftAP dashboard:

~~~text
SSID:       Crawler-Robot
Password:   crawler123
Dashboard:  http://192.168.4.1
WebSocket:  ws://192.168.4.1/ws
~~~

Connect one PC or phone to the AP and open http://192.168.4.1. The dashboard
reports state, fault, servo status, joint position/velocity, policy targets,
inference time, deadline misses, command age, IMU values, per-stream IMU
rates/ages, reset recovery, SPI drain diagnostics, and network metrics.

Telemetry is diagnostic-only. Incoming WebSocket messages are ignored; Wi-Fi
cannot command the robot. Dashboard telemetry is approximately every 250 ms.
Serial text telemetry is approximately 5 Hz.

## BLE packet formats for custom clients

Version 1 commands are eight bytes, little-endian format BBhhH:

~~~text
uint8   version
uint8   flags
int16   forward millimeters/second
int16   lateral millimeters/second
uint16  sequence
~~~

Version 2 commands are fourteen bytes, little-endian format BBhhhhhH:

~~~text
uint8   version = 2
uint8   flags
int16   forward millimeters/second
int16   lateral millimeters/second
int16   joint 1 raw position in milliradians
int16   joint 2 raw position in milliradians
int16   joint 3 raw position in milliradians
uint16  sequence
~~~

Version 2 flags are: 0x01 enable, 0x02 emergency stop, 0x04 clear fault,
0x08 manual position mode, 0x10 scripted sweep, 0x20 calibration, and 0x40
center/recovery position. Forward/lateral commands are limited to plus or minus
1500 mm/s, raw positions to approximately plus or minus 1.571 rad, and stale
commands are rejected after 300 ms.

## Diagnostics and useful commands

~~~powershell
& $pio run -e esp32-s3-hardware-wifi-idf-httpd-iram-cache32
git diff --check
~~~

Optional ONNX reference check:

~~~powershell
py -m venv .venv-onnx
& .\.venv-onnx\Scripts\python.exe -m pip install --upgrade pip numpy onnxruntime
& .\.venv-onnx\Scripts\python.exe tools\check_onnx.py
~~~

Useful serial fields include:

~~~text
state=RUNNING or DISARMED or ESTOP or FAULT
fault=...
imu=...
accel_valid=... accel_age_us=...
gyro_valid=... gyro_age_us=...
infer_us=... missed=...
drain_us=... budget_hits=... recovery=...
~~~

If the robot will not move, check:

1. ble=1 and a continuously running bridge.
2. Calibration completed and is reported valid.
3. State is not ESTOP or FAULT.
4. Press A once to clear a latched E-stop/fault, then send a fresh enabled mode.
5. imu=1, accel_valid=1, gyro_valid=1, and ages below 20 ms and 10 ms.
6. Servo supply, common ground, PWM wiring, and feedback ADC wiring.
7. Measured joint values are inside the calibrated range.

## Tests and verification limits

The ESP32 hardware build can be compiled with:

~~~powershell
& $pio run -e esp32-s3-hardware-wifi-idf-httpd-iram-cache32
~~~

The repository also contains native test sources for policy, safety, servo
mapping, and WebSocket lifecycle. Native tests do not verify physical BLE
behavior, ADC noise, BNO085 timing, PWM output, servo current, mechanical
limits, or power-fault behavior. Those require the actual board and controlled
robot testing.

Before normal walking:

1. Verify BNO085 output independently.
2. Verify all three servos with the robot unloaded.
3. Run calibration and inspect every ADC table.
4. Test D-pad Up recovery and manual Y mode.
5. Test L1 E-stop and A-only recovery.
6. Disconnect BLE and confirm the robot stops safely.
7. Test the policy with the robot lifted.
8. Only then test supported ground motion.
