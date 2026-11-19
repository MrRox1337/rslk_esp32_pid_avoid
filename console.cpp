#include "console.h"
#include "config.h"
#include "control.h"
#include "behavior.h"
#include "sonar.h"
#include "settings.h"

TeePrint con;
bool telemetry = true;
static unsigned long restartAt = 0;   // deferred reboot so the web reply gets sent first

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
  con.println("save (to flash) | load (from flash) | defaults (erase saved + reboot)");
}

static void printGains() {
  con.printf("wheel kp,ki,kd = %.3f, %.3f, %.3f\n", pidL.kp, pidL.ki, pidL.kd);
  con.printf("sync  kp,ki,kd = %.3f, %.3f, %.3f  (%s)\n", pidSync.kp, pidSync.ki, pidSync.kd,
             syncEnabled ? "on" : "off");
  con.printf("MAX_TPS = %.1f  flipEnc L/R = %d/%d  cruise = %d\n",
             MAX_TPS, FLIP_ENC_L, FLIP_ENC_R, CRUISE_SPEED);
  if (settingsDirty) con.println("(unsaved changes - type 'save' to keep them)");
}

void runCmd(char *s) {
  char *cmd = strtok(s, " ");
  if (!cmd) return;
  char *a = strtok(NULL, " ");
  char *b = strtok(NULL, " ");
  float x = a ? atof(a) : 0, y = b ? atof(b) : 0;

  // settings that persist: mark dirty so "gains" reminds you to save
  bool changes = true;
  if      (!strcmp(cmd, "cal"))    { stopRobot(); calibrate(con); }
  else if (!strcmp(cmd, "sync"))   { syncEnabled = x != 0; resetSync(); }
  else if (!strcmp(cmd, "cruise")) { CRUISE_SPEED = (int)x; }
  else if (!strcmp(cmd, "kp"))     { pidL.kp = pidR.kp = x; }
  else if (!strcmp(cmd, "ki"))     { pidL.ki = pidR.ki = x; pidL.reset(); pidR.reset(); }
  else if (!strcmp(cmd, "kd"))     { pidL.kd = pidR.kd = x; }
  else if (!strcmp(cmd, "sp"))     { pidSync.kp = x; }
  else if (!strcmp(cmd, "si"))     { pidSync.ki = x; pidSync.reset(); }
  else if (!strcmp(cmd, "sd"))     { pidSync.kd = x; }
  else changes = false;
  if (changes) { settingsDirty = true; printGains(); return; }

  if      (!strcmp(cmd, "go"))       { startAuto(); }
  else if (!strcmp(cmd, "stop"))     { stopRobot(); }
  else if (!strcmp(cmd, "m"))        { setManual(x, y); }
  else if (!strcmp(cmd, "tele"))     { telemetry = x != 0; }
  else if (!strcmp(cmd, "gains"))    { printGains(); }
  else if (!strcmp(cmd, "save"))     { settingsSave(con); }
  else if (!strcmp(cmd, "load"))     { settingsLoad(con); pidL.reset(); pidR.reset(); resetSync(); printGains(); }
  else if (!strcmp(cmd, "defaults")) { stopRobot(); settingsClear(con); restartAt = millis() + 500; }
  else printHelp();
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
  if (restartAt && (long)(millis() - restartAt) >= 0) ESP.restart();
}
