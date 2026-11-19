#include "sonar.h"
#include "config.h"

float distCm = 400;

static float hist[3] = {400, 400, 400};
static uint8_t idx = 0;

static float readSonar() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long us = pulseIn(ECHO, HIGH, 12000UL);
  if (us == 0) return 400;
  return us / 58.0;
}

static float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

void sonarBegin() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void sonarUpdate() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < SONAR_MS) return;
  last = now;
  hist[idx] = readSonar();
  idx = (idx + 1) % 3;
  distCm = median3(hist[0], hist[1], hist[2]);
}
