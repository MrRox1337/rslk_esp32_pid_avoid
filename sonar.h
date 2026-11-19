#pragma once
// HC-SR04 ultrasonic sensor, median-of-3 filtered (ECHO through 1k/2k divider, or HC-SR04P).

extern float distCm;   // latest filtered distance, 400 = nothing in range

void sonarBegin();
void sonarUpdate();    // call every loop; pings every SONAR_MS
