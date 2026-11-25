#pragma once
// All pins, constants and default tunables live here.
// Values marked "inline" can also be changed live from the console / web page.
#include <Arduino.h>

// ======================= PINS =======================
constexpr int PWML = 25, DIRL = 26, SLPL = 27;    // left  DRV8838
constexpr int PWMR = 32, DIRR = 33, SLPR = 13;    // right DRV8838
constexpr int ELA  = 18, ELB  = 19;               // left encoder
constexpr int ERA  = 23, ERB  = 4;                // right encoder
constexpr int TRIG = 14, ECHO = 34;               // HC-SR04 (34 is input-only; use divider!)
// spare: I2C SDA 21 / SCL 22

constexpr uint32_t PWM_FREQ = 20000;   // 20 kHz: above hearing, fine for DRV8838
constexpr uint8_t  PWM_BITS = 8;       // duty 0..255

// ============ DIRECTION FIXES (set after running "cal") ============
inline bool FLIP_MOTOR_L = false;
inline bool FLIP_MOTOR_R = false;
inline bool FLIP_ENC_L   = true;
inline bool FLIP_ENC_R   = true;

// ======================= CONTROL =======================
constexpr uint16_t LOOP_MS   = 10;     // 100 Hz control loop
constexpr float    VEL_ALPHA = 0.3f;   // velocity low-pass filter
inline float MAX_TPS = 2825.4f;        // ticks/s at PWM 255 (4x counting) -> "cal" measures it
inline float ACCEL_TPS2 = 3000.0f;     // forward speed ramp, ticks/s per second (0 = off) -> "acc X"
constexpr float TURN_ACCEL_TPS2 = 20000.0f;   // ramp on the turn part only (keeps spins from overshooting)

// PID gains (kp, ki, kd, integral limit) -- see control.cpp
#define WHEEL_PID_GAINS 0.15f, 1.0f, 0.002f, 120.0f   // ticks/s error -> PWM
#define WHEEL_I_ZONE    250.0f   // ticks/s; wheel integrator only runs inside this error band (0 = off)
#define SYNC_PID_GAINS  12.0f, 1.0f,  0.0f, 1200.0f  // tick error -> ticks/s correction

// ======================= GEOMETRY (odometry) =======================
// RSLK MAX / Romi: 70 mm wheels, 1440 ticks per wheel turn (4x counting), 141 mm track.
constexpr float TICKS_PER_CM = 65.5f;
constexpr float TRACK_CM     = 14.1f;   // wheel centre to wheel centre

// ======================= OBSTACLE AVOIDANCE =======================
inline int CRUISE_SPEED = 150;          // web speed slider; saved to flash on change
// Overtake: pull out to a lane LANE_CM to the side, pass, then merge back.
constexpr float PASS_CM      = 60;      // start overtaking when an obstacle is this close
constexpr float LANE_CM      = 30;      // sideways shift of the passing lane
constexpr int   PASS_SIDE    = +1;      // +1 = overtake on the right, -1 = on the left
constexpr float OBST_LEN_CM  = 20;      // assumed obstacle length (sonar can't see it once beside it)
constexpr float ROBOT_LEN_CM = 15;      // robot must clear the obstacle by this much before merging
constexpr float PASS_MARGIN_CM = 10;    // be fully in the passing lane this far before the obstacle
constexpr float STEER_GAIN   = 1.0f;    // turn = speed * gain * heading error (rad)
constexpr float LOOK_CM      = 20;      // lane-keeping look-ahead (bigger = gentler corrections)
constexpr float MAX_HEAD_RAD = 0.7f;    // ~40 deg max heading away from the lane
constexpr float TURN_FRAC    = 0.6f;    // max turn / speed: inner wheel >= 25% of outer, never stops
constexpr float STOP_CM      = 20;      // MANUAL mode only: won't drive forward into something this close
constexpr uint16_t SONAR_MS    = 60;

// ======================= WIFI / WEB =======================
// Joins the network in wifi_secrets.h (git-ignored) if it exists, e.g. a phone hotspot.
// If that fails (or there is no file) it runs its own access point AP_SSID instead.
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif
constexpr char HOSTNAME[] = "rslk";                 // -> http://rslk.local
constexpr uint16_t WIFI_CONNECT_MS = 15000;         // give up joining after this, start own AP
constexpr char AP_SSID[] = "RSLK-ESP32";
constexpr char AP_PASS[] = "rslk1234";              // min 8 chars (WPA2)
constexpr uint16_t WEB_DRIVE_TIMEOUT_MS = 500;      // pad stops robot if no update this long
constexpr uint16_t SPEED_SAVE_MS = 3000;            // save the speed slider once it has been still this long
constexpr uint16_t REC_MS = 50, REC_SECONDS = 60;   // flight recorder: /rec on the web page (CSV)

// ======================= CONSOLE =======================
constexpr uint32_t SERIAL_BAUD  = 115200;
constexpr uint16_t TELEMETRY_MS = 100;
