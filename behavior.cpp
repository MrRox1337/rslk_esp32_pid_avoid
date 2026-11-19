#include "behavior.h"
#include "config.h"
#include "control.h"
#include "sonar.h"

Mode mode = IDLE;
AvoidState avoidState = CRUISE;

static unsigned long stateT0 = 0;
static int spinDir = 1;
static float manSpeed = 0, manTurn = 0;
static bool  deadmanOn = false;
static unsigned long lastManual = 0;

static void setAvoidState(AvoidState s) {
  avoidState = s;
  stateT0 = millis();
  resetSync();
}

void stopRobot() {
  mode = IDLE;
  deadmanOn = false;
}

void startAuto() {
  mode = AUTO;
  deadmanOn = false;
  setAvoidState(CRUISE);
}

void setManual(float speed, float turn, bool deadman) {
  if (mode != MANUAL || speed != manSpeed || turn != manTurn) resetSync();
  mode = MANUAL;
  manSpeed = speed;
  manTurn = turn;
  deadmanOn = deadman;
  lastManual = millis();
}

static void avoidStep() {
  unsigned long t = millis() - stateT0;
  switch (avoidState) {
    case CRUISE: {
      if (distCm < STOP_CM) { setAvoidState(BACKUP); break; }
      float sp = CRUISE_SPEED;
      if (distCm < SLOW_CM)
        sp = MIN_SPEED + (CRUISE_SPEED - MIN_SPEED) * (distCm - STOP_CM) / (SLOW_CM - STOP_CM);
      drive(sp, 0);
    } break;
    case BACKUP:
      drive(-BACK_SPEED, 0);
      if (t > BACKUP_MS) { spinDir = -spinDir; setAvoidState(SPIN); }
      break;
    case SPIN:
      drive(0, spinDir * SPIN_SPEED);
      if (t > SPIN_MIN_MS && distCm > CLEAR_CM) setAvoidState(CRUISE);
      else if (t > SPIN_MAX_MS) { spinDir = -spinDir; setAvoidState(SPIN); }
      break;
  }
}

void behaviorStep() {
  if (deadmanOn && millis() - lastManual > WEB_DRIVE_TIMEOUT_MS) stopRobot();

  if (mode == AUTO) {
    avoidStep();
  } else if (mode == MANUAL) {
    if (distCm < STOP_CM && manSpeed > 0) drive(0, manTurn);   // don't drive into things
    else                                  drive(manSpeed, manTurn);
  } else {
    drive(0, 0);
  }
}

const char *modeName() {
  static const char *names[] = {"IDLE", "AUTO", "MANUAL"};
  static const char *avoid[] = {"AUTO/cruise", "AUTO/backup", "AUTO/spin"};
  return mode == AUTO ? avoid[avoidState] : names[mode];
}
