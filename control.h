#pragma once
// Per-wheel velocity PID + feedforward, and a wheel-sync PID that keeps the
// wheels' relative travel matching the command (straight lines, exact turns).
#include <Arduino.h>
#include "pid.h"

extern PID pidL, pidR, pidSync;
extern bool syncEnabled;

// Live state (read-only outside control.cpp)
extern float tgtL, tgtR, velL, velR, syncErr;
extern int   outL, outR;

// speed, turn in PWM units (-255..255): turn > 0 veers RIGHT, speed 0 + turn = spin
void drive(float speed, float turn);
void resetSync();
void getPositions(long &l, long &r);   // wheel travel in ticks (+ = forward)
void controlUpdate();             // call every loop; runs the PID at LOOP_MS
void calibrate(Print &out);       // blocking ~3 s; measures MAX_TPS and encoder directions
