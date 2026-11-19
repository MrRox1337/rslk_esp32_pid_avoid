#pragma once
// Simple PID with clamped integrator.
#include <Arduino.h>

struct PID {
  float kp, ki, kd, iLim;
  float iTerm = 0, prevErr = 0;

  PID(float p, float i, float d, float lim) : kp(p), ki(i), kd(d), iLim(lim) {}

  float update(float err, float dt) {
    iTerm += ki * err * dt;
    iTerm = constrain(iTerm, -iLim, iLim);
    float deriv = (dt > 0) ? (err - prevErr) / dt : 0;
    prevErr = err;
    return kp * err + iTerm + kd * deriv;
  }

  void reset() { iTerm = 0; prevErr = 0; }
};
