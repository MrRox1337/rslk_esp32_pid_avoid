#include "motors.h"
#include "config.h"

static void setMotor(int pwmPin, int dirPin, int u, bool flip) {
  if (flip) u = -u;
  u = constrain(u, -255, 255);
  digitalWrite(dirPin, u < 0 ? HIGH : LOW);
  ledcWrite(pwmPin, abs(u));
}

void setMotors(int l, int r) {
  setMotor(PWML, DIRL, l, FLIP_MOTOR_L);
  setMotor(PWMR, DIRR, r, FLIP_MOTOR_R);
}

void motorsBegin() {
  pinMode(DIRL, OUTPUT); pinMode(SLPL, OUTPUT);
  pinMode(DIRR, OUTPUT); pinMode(SLPR, OUTPUT);
  ledcAttach(PWML, PWM_FREQ, PWM_BITS);
  ledcAttach(PWMR, PWM_FREQ, PWM_BITS);

  setMotors(0, 0);
  digitalWrite(SLPL, HIGH);   // wake drivers
  digitalWrite(SLPR, HIGH);
}
