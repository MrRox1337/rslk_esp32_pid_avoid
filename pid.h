#pragma once
// Simple PID with clamped integrator and optional integral zone (anti-windup).
#include <Arduino.h>

struct PID {
  float kp, ki, kd, iLim;
  float iZone;                   // integrate only while |err| < iZone (0 = always)
  float iTerm = 0, prevErr = 0;

  PID(float p, float i, float d, float lim, float zone = 0)
    : kp(p), ki(i), kd(d), iLim(lim), iZone(zone) {}

  float update(float err, float dt) {
    if (iZone <= 0 || fabsf(err) < iZone) iTerm += ki * err * dt;
    iTerm = constrain(iTerm, -iLim, iLim);
    float deriv = (dt > 0) ? (err - prevErr) / dt : 0;
    prevErr = err;
    return kp * err + iTerm + kd * deriv;
  }

  void reset() { iTerm = 0; prevErr = 0; }
};
