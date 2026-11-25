# RSLK ESP32 – PID drive + obstacle avoidance

Firmware for a **TI-RSLK MAX** chassis (Pololu motor driver and power distribution board) driven by a
**Freenove ESP32-WROVER(-E) CAM** board. The camera must be unplugged, because its pins are reused.

- Encoders are counted in hardware by the ESP32 PCNT peripheral (full 4x quadrature, no interrupts).
- Each wheel has a velocity PID with feedforward. A wheel-sync PID keeps the robot driving straight and turning exactly as commanded.
- Autonomous obstacle avoidance uses an HC-SR04 ultrasonic sensor: the robot holds its lane and **overtakes** obstacles like a car does (see [Auto mode](#auto-mode-overtaking)).
- You can control it from the **USB serial console** or from a **web page served by the ESP32 itself**.
- Tuned gains and calibration values are saved in flash (Preferences/NVS), so they survive power cycles.

## Hardware

| Function | ESP32 GPIO |
|---|---|
| Left motor PWM / DIR / SLEEP | 25 / 26 / 27 |
| Right motor PWM / DIR / SLEEP | 32 / 33 / 13 |
| Left encoder A / B | 18 / 19 |
| Right encoder A / B | 23 / 4 |
| HC-SR04 TRIG / ECHO | 14 / 34 |
| BNO055 SDA / SCL (I2C, 0x28 or 0x29) | 21 / 22 |

GPIO 34 is input-only and **not 5 V tolerant**. Feed the HC-SR04's ECHO pin through a 1k/2k voltage
divider, or use the 3.3 V HC-SR04P instead.

The BNO055 is optional and powered from 3.3 V. On bare GY-BNO055 boards, tie **PS0 and PS1 to GND** to select I2C mode. With the BNO055 present, heading comes from its gyro, which wheel slip can't fool. Without it, heading comes from the encoders. All pins are defined in [`config.h`](config.h).

## Building

- Arduino IDE 2.x or `arduino-cli`
- ESP32 board package **3.x** (tested with 3.3.12)
- Board: **ESP32 Wrover Module**
- No extra libraries are needed (only `WiFi`, `WebServer` and `Preferences`, which come with the ESP32 core)

```sh
arduino-cli compile --fqbn esp32:esp32:esp32wrover .
arduino-cli upload  --fqbn esp32:esp32:esp32wrover -p COM5 .
```

Replace `COM5` with the port your board is on.

## First run

1. Open the serial monitor at **115200** baud with the **Newline** line ending.
2. Lift the wheels off the ground and type `cal`. This measures the top speed (`MAX_TPS`) and the encoder directions.
   If a wheel spins backwards, set `FLIP_MOTOR_L` or `FLIP_MOTOR_R` in `config.h`, re-upload, and run `cal` again.
3. Type `save` to keep the calibration in flash.
4. Type `go` to start obstacle avoidance, and `stop` to stop.

## Web control

On boot the ESP32 joins the WiFi network in `wifi_secrets.h`, for example your phone's hotspot. That file is
git-ignored, so copy [`wifi_secrets.example.h`](wifi_secrets.example.h) to `wifi_secrets.h` and fill in your network.
Once it's connected, open **http://rslk.local**, or the IP address printed on Serial or shown in the hotspot's list of
connected devices (as `rslk`). Some Android phones don't resolve `.local` names, so use the IP address there.

If there's no `wifi_secrets.h`, or the robot can't join within 15 s, it creates its own network instead:

- SSID: `RSLK-ESP32`
- Password: `rslk1234`

Join it and open **http://192.168.4.1**. The robot stops whenever the WiFi link drops. The page has:

- Live status: mode, distance, target and actual speed of each wheel, sync error, PWM
- GO / STOP / Calibrate / Gains / Save buttons
- A hold-to-drive pad with a speed slider. The slider also sets the AUTO cruise speed, is saved to flash 3 s after you stop moving it, and is restored when the page loads. The robot stops when you release the button, after 500 ms without
  updates, or when a device disconnects from the WiFi network.
- A command box that accepts any console command and shows its output

## Auto mode (overtaking)

`go` starts AUTO mode. The lane is the line the robot is pointing along at that moment, and wheel odometry keeps it on that line.

1. **Cruise**: drive straight at the slider speed.
2. **Overtake**: when the sonar sees something closer than `PASS_CM` (60 cm), the robot pulls out smoothly to a passing lane `LANE_CM`
   (30 cm) to the side and keeps going. The sonar can't see the obstacle once the robot is beside it, so odometry decides when it
   has been passed: the sonar distance plus `OBST_LEN_CM` plus `ROBOT_LEN_CM`. Heading comes from the BNO055 when it's fitted.
3. **Merge**: the robot eases back into its original lane, taking as long as it spent pulling out and passing. Then it goes back to cruising.

Steering is proportional to the speed, so the robot follows the same path at any cruise speed. The robot never backs up:
every obstacle is overtaken, and one seen late just gets a quicker pull-out.
The sideways offset is shown as *Lane offset* on the web page. The geometry constants and `PASS_SIDE` are in `config.h`.

## Console commands

The same commands work over serial and on the web page.

| Command | Action |
|---|---|
| `go` / `stop` | Start obstacle avoidance / stop |
| `m S T` | Manual drive: speed `S`, turn `T` (-255..255; `T` > 0 veers right) |
| `cal` | Calibrate `MAX_TPS` and encoder directions (blocks for about 3 s) |
| `kp` / `ki` / `kd X` | Wheel velocity PID gains |
| `iz X` | Wheel integral zone in ticks/s: the integrator only runs while the error is below this (0 = always) |
| `acc X` | Forward acceleration and deceleration limit in ticks/s², used for stopping too (0 = off, instant). Turning uses its own fixed ramp |
| `sp` / `si` / `sd X` | Wheel-sync PID gains |
| `sync 0/1` | Turn the wheel-sync PID off / on |
| `cruise X` | Cruise speed in AUTO mode |
| `tele 0/1/N` | Turn serial telemetry off / on, or on every `N` ms (N ≥ 10). Serial Plotter friendly |
| `gains` | Print the current values |
| `imu` | Print the BNO055 heading and calibration. Turning right should increase it; if not, set `IMU_SIGN = -1` |
| `rec` | Print the flight recorder (last drive, up to 60 s at 20 Hz) as CSV. Also a download at `/rec` and the Log button on the web page |
| `save` / `load` | Write the current values to flash / reload the saved ones |
| `defaults` | Erase the saved values and reboot with the `config.h` defaults |

Saved values take priority over `config.h`. If you change a default in `config.h`, run `defaults` to use it.

## PID tuner

[`tools/pid_tuner.py`](tools/pid_tuner.py) runs step tests over USB serial and shows them on a live dashboard.
It needs Python 3 and `pyserial`. Close the Arduino serial monitor first, since only one program can use the port.

```sh
python tools/pid_tuner.py --port COM3      # then open http://localhost:8765
curl "localhost:8765/test?kind=step&speed=150&sync=0"       # wheel step test
curl "localhost:8765/test?kind=sync&speed=120&turn=60"      # sync test (turning)
curl "localhost:8765/cmd?c=kp%200.15"                       # any console command
```

Each run reports rise time, overshoot, settling time, steady-state error, ripple and PWM saturation. Runs are
saved as CSV, JSON and SVG in `tools/tune_runs/`. Opening the port resets the ESP32, so any unsaved gains are lost.

## Code layout

| File | Purpose |
|---|---|
| `rslk_esp32_pid_avoid.ino` | `setup()` / `loop()` only |
| `config.h` | Pins, constants and default tunables |
| `pid.h` | PID controller |
| `motors.*` | DRV8838 motor drivers |
| `encoders.*` | PCNT quadrature counting |
| `control.*` | Wheel and sync PIDs, `drive()`, `calibrate()` |
| `sonar.*` | HC-SR04 with median-of-3 filter |
| `imu.*` | BNO055 heading (IMUPLUS mode, 100 Hz), falls back to encoders |
| `behavior.*` | IDLE / AUTO / MANUAL modes, avoidance state machine |
| `console.*` | Command interpreter (serial and web), telemetry |
| `settings.*` | Saving and loading settings in flash |
| `web.*`, `web_page.h` | WiFi access point, HTTP endpoints, control page |
| `tools/pid_tuner.py` | PC-side step-test runner and live tuning dashboard |
