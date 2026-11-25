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
inline bool FLIP_ENC_L   = false;
inline bool FLIP_ENC_R   = false;

// ======================= CONTROL =======================
constexpr uint16_t LOOP_MS   = 10;     // 100 Hz control loop
constexpr float    VEL_ALPHA = 0.3f;   // velocity low-pass filter
inline float MAX_TPS = 4800.0f;        // ticks/s at PWM 255 (4x counting) -> "cal" measures it
inline float ACCEL_TPS2 = 0.0f;        // setpoint ramp, ticks/s per second (0 = off) -> "acc X"

// PID gains (kp, ki, kd, integral limit) -- see control.cpp
#define WHEEL_PID_GAINS 0.04f, 0.25f, 0.0f, 120.0f    // ticks/s error -> PWM
#define WHEEL_I_ZONE    0.0f    // ticks/s; wheel integrator only runs inside this error band (0 = off)
#define SYNC_PID_GAINS  4.0f,  1.0f,  0.0f, 1200.0f   // tick error -> ticks/s correction

// ======================= OBSTACLE AVOIDANCE =======================
inline int CRUISE_SPEED = 150;
constexpr int   MIN_SPEED  = 70;
constexpr int   BACK_SPEED = 110;
constexpr int   SPIN_SPEED = 110;
constexpr float SLOW_CM    = 50;
constexpr float STOP_CM    = 20;
constexpr float CLEAR_CM   = 45;
constexpr uint16_t BACKUP_MS   = 400;
constexpr uint16_t SPIN_MIN_MS = 250;
constexpr uint16_t SPIN_MAX_MS = 2500;
constexpr uint16_t SONAR_MS    = 60;

// ======================= WIFI / WEB =======================
constexpr char AP_SSID[] = "RSLK-ESP32";
constexpr char AP_PASS[] = "rslk1234";              // min 8 chars (WPA2)
constexpr uint16_t WEB_DRIVE_TIMEOUT_MS = 500;      // pad stops robot if no update this long

// ======================= CONSOLE =======================
constexpr uint32_t SERIAL_BAUD  = 115200;
constexpr uint16_t TELEMETRY_MS = 100;
