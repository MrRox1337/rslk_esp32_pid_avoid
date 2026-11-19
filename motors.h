#pragma once
// DRV8838 motor drivers (PWM + DIR + SLEEP per side).

void motorsBegin();
void setMotors(int left, int right);   // -255..255, FLIP_MOTOR_x applied
