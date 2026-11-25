#pragma once
// BNO055 heading (IMUPLUS fusion: gyro + accelerometer, no magnetometer, so motors can't disturb it).
// If the sensor is missing or stops answering, imuOk goes false and odometry uses the encoders.
#include <Arduino.h>

extern bool imuOk;

void  imuBegin(Print &out);    // call once in setup(); keep the robot still for ~1 s
void  imuUpdate();             // call every loop; reads every IMU_MS
float imuHeading();            // rad since boot, unwrapped, + = turned right (clockwise from above)
void  imuStatus(Print &out);   // heading and calibration levels
