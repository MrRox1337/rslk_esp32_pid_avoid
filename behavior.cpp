#include "behavior.h"
#include "config.h"
#include "control.h"
#include "sonar.h"

Mode mode = IDLE;
AvoidState avoidState = CRUISE;
float laneY = 0;

static unsigned long stateT0 = 0;
static float manSpeed = 0, manTurn = 0;
static bool  deadmanOn = false;
static unsigned long lastManual = 0;

// ---------- lane odometry ----------
// x = distance along the lane, laneY = sideways offset, psi = heading. cm / rad, + = right.
static float odoX = 0, odoPsi = 0;
static long  odoL = 0, odoR = 0;

static void odoReset() {
  odoX = laneY = odoPsi = 0;
  getPositions(odoL, odoR);
}

static void odoUpdate() {
  long l, r;
  getPositions(l, r);
  float dL = (l - odoL) / TICKS_PER_CM, dR = (r - odoR) / TICKS_PER_CM;
  odoL = l; odoR = r;
  float ds = (dL + dR) / 2, dPsi = (dL - dR) / TRACK_CM;
  float mid = odoPsi + dPsi / 2;
  odoX  += ds * cosf(mid);
  laneY += ds * sinf(mid);
  odoPsi += dPsi;
}

// ---------- overtake ----------
static float passEndX = 0;        // odoX at which the obstacle is cleared
static float outSec = 1, backSec = 1, backFromY = 0;

static float cmPerSec(float speed) { return speed / 255.0f * MAX_TPS / TICKS_PER_CM; }

// smooth 0->1 over u = 0..1, zero slope at both ends
static float ease(float u)  { u = constrain(u, 0.0f, 1.0f); return u * u * (3 - 2 * u); }
static float easeD(float u) { return (u <= 0 || u >= 1) ? 0 : 6 * u * (1 - u); }

// Follow a sideways target yRef (moving at yRate cm/s). The turn is proportional to the
// speed, so the robot traces the same path whatever the cruise speed.
static void steer(float speed, float yRef, float yRate) {
  float v = max(cmPerSec(speed), 1.0f);
  float psiRef = atan2f(yRate, v) + atan2f(yRef - laneY, LOOK_CM);
  psiRef = constrain(psiRef, -MAX_HEAD_RAD, MAX_HEAD_RAD);
  float turn = speed * STEER_GAIN * (psiRef - odoPsi);
  float lim = TURN_FRAC * fabsf(speed);              // inner wheel keeps rolling forwards
  turn = constrain(turn, -lim, lim);
  float sum = fabsf(speed) + fabsf(turn);            // scale down together, never clip one,
  if (sum > 255) { speed *= 255 / sum; turn *= 255 / sum; }   // so the curve is kept
  drive(speed, turn);
}

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
  odoReset();                 // the lane is wherever the robot is pointing now
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
  float t = (millis() - stateT0) * 1e-3f;
  float sp = CRUISE_SPEED;
  switch (avoidState) {
    case CRUISE:
      if (distCm < PASS_CM) {
        // Obstacle ahead: be in the passing lane PASS_MARGIN_CM before reaching it,
        // and stay there until the robot is past its far end.
        passEndX = odoX + distCm + OBST_LEN_CM + ROBOT_LEN_CM;
        outSec = constrain((distCm - PASS_MARGIN_CM) / max(cmPerSec(sp), 1.0f), 0.5f, 5.0f);
        setAvoidState(PASS_OUT);
        break;
      }
      steer(sp, 0, 0);                         // hold the lane
      break;

    case PASS_OUT: {                           // sonar ignored: it can't see the obstacle from here
      float yRef = PASS_SIDE * LANE_CM * ease(t / outSec);
      float rate = PASS_SIDE * LANE_CM * easeD(t / outSec) / outSec;
      steer(sp, yRef, rate);
      if (odoX >= passEndX) {
        backSec = max(t, 0.5f);                // merge back over the same time it took to pass
        backFromY = yRef;
        setAvoidState(PASS_BACK);
      }
    } break;

    case PASS_BACK: {
      float u = t / backSec;
      steer(sp, backFromY * (1 - ease(u)), -backFromY * easeD(u) / backSec);
      if (u >= 1) setAvoidState(CRUISE);
    } break;
  }
}

// ---------- flight recorder ----------
struct Sample {
  uint32_t ms;
  uint8_t  mode, state;
  int16_t  x, y, psi;                    // mm, mm, 0.1 deg
  int16_t  tgtL, velL, tgtR, velR;       // ticks/s
  int16_t  outL, outR, sync, dist;       // PWM, ticks, mm
};
static constexpr int REC_N = REC_SECONDS * 1000 / REC_MS;
static Sample rec[REC_N];
static int recHead = 0, recLen = 0;

static void recordStep() {
  static unsigned long last = 0;
  static Mode lastMode = IDLE;
  unsigned long now = millis();
  if (mode != IDLE && lastMode == IDLE) recHead = recLen = 0;   // new drive, new log
  lastMode = mode;
  if (mode == IDLE || now - last < REC_MS) return;
  last = now;
  Sample &s = rec[recHead];
  s = {now, (uint8_t)mode, (uint8_t)avoidState,
       (int16_t)constrain(odoX * 10, -32000, 32000), (int16_t)constrain(laneY * 10, -32000, 32000),
       (int16_t)constrain(degrees(odoPsi) * 10, -32000, 32000),
       (int16_t)tgtL, (int16_t)velL, (int16_t)tgtR, (int16_t)velR,
       (int16_t)outL, (int16_t)outR, (int16_t)constrain(syncErr, -32000, 32000), (int16_t)(distCm * 10)};
  recHead = (recHead + 1) % REC_N;
  if (recLen < REC_N) recLen++;
}

int recCount() { return recLen; }

void recHeader(char *buf, size_t n) {
  snprintf(buf, n, "t_s,mode,x_cm,y_cm,heading_deg,tgtL,velL,tgtR,velR,outL,outR,syncErr,dist_cm\n");
}

void recLine(int i, char *buf, size_t n) {
  static const char *names[] = {"idle", "cruise", "overtake", "merge", "manual"};
  const Sample &first = rec[(recHead - recLen + REC_N) % REC_N];
  const Sample &s = rec[(recHead - recLen + i + REC_N) % REC_N];
  int m = s.mode == AUTO ? 1 + s.state : s.mode == MANUAL ? 4 : 0;
  snprintf(buf, n, "%.2f,%s,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%.1f\n",
           (s.ms - first.ms) / 1000.0, names[m], s.x / 10.0, s.y / 10.0, s.psi / 10.0,
           s.tgtL, s.velL, s.tgtR, s.velR, s.outL, s.outR, s.sync, s.dist / 10.0);
}

void behaviorStep() {
  odoUpdate();
  if (deadmanOn && millis() - lastManual > WEB_DRIVE_TIMEOUT_MS) stopRobot();

  if (mode == AUTO) {
    avoidStep();
  } else if (mode == MANUAL) {
    if (distCm < STOP_CM && manSpeed > 0) drive(0, manTurn);   // don't drive into things
    else                                  drive(manSpeed, manTurn);
  } else {
    drive(0, 0);
  }
  recordStep();
}

const char *modeName() {
  static const char *names[] = {"IDLE", "AUTO", "MANUAL"};
  static const char *avoid[] = {"AUTO/cruise", "AUTO/overtake", "AUTO/merge"};
  return mode == AUTO ? avoid[avoidState] : names[mode];
}
