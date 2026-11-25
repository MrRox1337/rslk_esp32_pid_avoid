# RSLK ESP32 – PID drive + obstacle avoidance

Firmware for a **TI-RSLK MAX** chassis (Pololu motor driver and power distribution board) driven by a
**Freenove ESP32-WROVER(-E) CAM** board. The camera must be unplugged, because its pins are reused.

- Encoders are counted in hardware by the ESP32 PCNT peripheral (full 4x quadrature, no interrupts).
- Each wheel has a velocity PID with feedforward. A wheel-sync PID keeps the robot driving straight and turning exactly as commanded.
- Autonomous obstacle avoidance uses an HC-SR04 ultrasonic sensor: the robot cruises, slows down as it approaches an obstacle, then backs up and spins until the way is clear.
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

GPIO 34 is input-only and **not 5 V tolerant**. Feed the HC-SR04's ECHO pin through a 1k/2k voltage
divider, or use the 3.3 V HC-SR04P instead. All pins are defined in [`config.h`](config.h).

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

On boot the ESP32 creates its own WiFi network:

- SSID: `RSLK-ESP32`
- Password: `rslk1234`

You can change both in `config.h`. Join the network and open **http://192.168.4.1**. The page has:

- Live status: mode, distance, target and actual speed of each wheel, sync error, PWM
- GO / STOP / Calibrate / Gains / Save buttons
- A hold-to-drive pad with a speed slider. The robot stops when you release the button, after 500 ms without
  updates, or when a device disconnects from the WiFi network.
- A command box that accepts any console command and shows its output

## Console commands

The same commands work over serial and on the web page.

| Command | Action |
|---|---|
| `go` / `stop` | Start obstacle avoidance / stop |
| `m S T` | Manual drive: speed `S`, turn `T` (-255..255; `T` > 0 veers right) |
| `cal` | Calibrate `MAX_TPS` and encoder directions (blocks for about 3 s) |
| `kp` / `ki` / `kd X` | Wheel velocity PID gains |
| `iz X` | Wheel integral zone in ticks/s: the integrator only runs while the error is below this (0 = always) |
| `acc X` | Setpoint ramp in ticks/s² (0 = off). `stop` is never ramped |
| `sp` / `si` / `sd X` | Wheel-sync PID gains |
| `sync 0/1` | Turn the wheel-sync PID off / on |
| `cruise X` | Cruise speed in AUTO mode |
| `tele 0/1/N` | Turn serial telemetry off / on, or on every `N` ms (N ≥ 10). Serial Plotter friendly |
| `gains` | Print the current values |
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
| `behavior.*` | IDLE / AUTO / MANUAL modes, avoidance state machine |
| `console.*` | Command interpreter (serial and web), telemetry |
| `settings.*` | Saving and loading settings in flash |
| `web.*`, `web_page.h` | WiFi access point, HTTP endpoints, control page |
| `tools/pid_tuner.py` | PC-side step-test runner and live tuning dashboard |
