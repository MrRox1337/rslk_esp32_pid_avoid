#include "console.h"
#include "config.h"
#include "control.h"
#include "behavior.h"
#include "sonar.h"

TeePrint con;
bool telemetry = true;

size_t TeePrint::write(uint8_t c) {
  Serial.write(c);
  if (capture && buf.length() < 2048) buf += (char)c;
  return 1;
}

size_t TeePrint::write(const uint8_t *b, size_t n) {
  for (size_t i = 0; i < n; i++) write(b[i]);
  return n;
}

void printHelp() {
  con.println("go | stop | m S T (manual speed,turn) | cal | sync 0/1 | tele 0/1");
  con.println("kp/ki/kd X (wheel PID) | sp/si/sd X (sync PID) | cruise X | gains");
}

static void printGains() {
  con.printf("wheel kp,ki,kd = %.3f, %.3f, %.3f\n", pidL.kp, pidL.ki, pidL.kd);
  con.printf("sync  kp,ki,kd = %.3f, %.3f, %.3f\n", pidSync.kp, pidSync.ki, pidSync.kd);
  con.printf("MAX_TPS = %.1f\n", MAX_TPS);
}

void runCmd(char *s) {
  char *cmd = strtok(s, " ");
  if (!cmd) return;
  char *a = strtok(NULL, " ");
  char *b = strtok(NULL, " ");
  float x = a ? atof(a) : 0, y = b ? atof(b) : 0;

  if      (!strcmp(cmd, "go"))     { startAuto(); }
  else if (!strcmp(cmd, "stop"))   { stopRobot(); }
  else if (!strcmp(cmd, "m"))      { setManual(x, y); }
  else if (!strcmp(cmd, "cal"))    { stopRobot(); calibrate(con); }
  else if (!strcmp(cmd, "sync"))   { syncEnabled = x != 0; resetSync(); }
  else if (!strcmp(cmd, "tele"))   { telemetry = x != 0; }
  else if (!strcmp(cmd, "cruise")) { CRUISE_SPEED = (int)x; }
  else if (!strcmp(cmd, "kp"))     { pidL.kp = pidR.kp = x; }
  else if (!strcmp(cmd, "ki"))     { pidL.ki = pidR.ki = x; pidL.reset(); pidR.reset(); }
  else if (!strcmp(cmd, "kd"))     { pidL.kd = pidR.kd = x; }
  else if (!strcmp(cmd, "sp"))     { pidSync.kp = x; }
  else if (!strcmp(cmd, "si"))     { pidSync.ki = x; pidSync.reset(); }
  else if (!strcmp(cmd, "sd"))     { pidSync.kd = x; }
  else if (!strcmp(cmd, "gains"))  { printGains(); return; }
  else { printHelp(); return; }
  printGains();
}

static void readSerial() {
  static char buf[32];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { buf[n] = 0; runCmd(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

static void printTelemetry() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (!telemetry || now - last < TELEMETRY_MS) return;
  last = now;
  Serial.printf("tgtL:%.0f,velL:%.0f,tgtR:%.0f,velR:%.0f,syncErr:%.0f,dist:%.1f\n",
                tgtL, velL, tgtR, velR, syncErr, distCm);
}

void consoleStep() {
  readSerial();
  printTelemetry();
}
