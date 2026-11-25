#include "control.h"
#include "config.h"
#include "motors.h"
#include "encoders.h"

PID pidL   (WHEEL_PID_GAINS, WHEEL_I_ZONE);
PID pidR   (WHEEL_PID_GAINS, WHEEL_I_ZONE);
PID pidSync(SYNC_PID_GAINS);
bool syncEnabled = true;

float tgtL = 0, tgtR = 0, velL = 0, velR = 0, syncErr = 0;
int   outL = 0, outR = 0;

static float cmdSpeed = 0, cmdTurn = 0;
static long  posL = 0, posR = 0, lastPosL = 0, lastPosR = 0;
static float expectedDiff = 0;
static float rampS = 0, rampT = 0;     // ramped forward / turn parts of the targets, ticks/s

void drive(float speed, float turn) {
  turn = constrain(turn, -255, 255);
  float room = 255 - fabs(turn);
  cmdSpeed = constrain(speed, -room, room);
  cmdTurn  = turn;
}

void resetSync() {
  pidSync.reset();
  expectedDiff = posL - posR;
  syncErr = 0;
}

void getPositions(long &l, long &r) { l = posL; r = posR; }

static int wheelOut(float setpoint, float vel, PID &pid, float dt) {
  float ff = setpoint / MAX_TPS * 255.0;
  float u  = ff + pid.update(setpoint - vel, dt);
  return (int)constrain(u, -255, 255);
}

static void controlStep(float dt) {
  long l, r;
  readEncoders(l, r);
  posL = FLIP_ENC_L ? -l : l;
  posR = FLIP_ENC_R ? -r : r;

  float rawL = (posL - lastPosL) / dt;
  float rawR = (posR - lastPosR) / dt;
  lastPosL = posL; lastPosR = posR;
  velL += VEL_ALPHA * (rawL - velL);
  velR += VEL_ALPHA * (rawR - velR);

  float wantS = cmdSpeed / 255.0 * MAX_TPS;
  float wantT = cmdTurn  / 255.0 * MAX_TPS;

  // Setpoint ramps: forward speed at ACCEL_TPS2 (gentle, 0 = off), turn at TURN_ACCEL_TPS2.
  // Stopping uses the same ramp, so the robot slows down instead of braking hard.
  // While the speed ramps, the turn is scaled with it so the curve stays the same and the
  // slow wheel never runs backwards (pure spins, wantS == 0, are not scaled).
  float stepS = ACCEL_TPS2 * dt, stepT = TURN_ACCEL_TPS2 * dt;
  float prevS = rampS;
  rampS = ACCEL_TPS2 > 0 ? rampS + constrain(wantS - rampS, -stepS, stepS) : wantS;
  if (wantS == 0 && wantT == 0 && prevS != 0) {
    rampT *= rampS / prevS;                // stopping from a curve: shrink the turn with the speed,
  } else {                                 // both wheels reach 0 together along the same curve
    float turnGoal = wantS != 0 ? wantT * constrain(rampS / wantS, 0.0f, 1.0f) : wantT;
    rampT += constrain(turnGoal - rampT, -stepT, stepT);
  }

  if (rampS == 0 && rampT == 0) {          // fully stopped: motors off, controllers reset
    tgtL = tgtR = 0;
    setMotors(0, 0);
    outL = outR = 0;
    pidL.reset(); pidR.reset();
    resetSync();
    return;
  }

  tgtL = rampS + rampT;
  tgtR = rampS - rampT;

  expectedDiff += (tgtL - tgtR) * dt;
  syncErr = (posL - posR) - expectedDiff;   // + = left wheel has gone too far
  float corr = syncEnabled ? pidSync.update(syncErr, dt) : 0;
  corr = constrain(corr, -0.3 * MAX_TPS, 0.3 * MAX_TPS);

  outL = wheelOut(tgtL - corr, velL, pidL, dt);
  outR = wheelOut(tgtR + corr, velR, pidR, dt);
  setMotors(outL, outR);
}

void controlUpdate() {
  static unsigned long lastUs = micros();
  unsigned long us = micros();
  if (us - lastUs >= LOOP_MS * 1000UL) {
    float dt = (us - lastUs) * 1e-6;
    lastUs = us;
    controlStep(dt);
  }
}

void calibrate(Print &out) {
  out.println("CAL: both motors PWM 200 for 2 s. Wheels off the ground or give it room.");
  delay(1000);
  setMotors(200, 200);
  delay(500);
  long l0, r0, l1, r1;
  readEncoders(l0, r0);
  delay(1500);
  readEncoders(l1, r1);
  setMotors(0, 0);

  float tl = (l1 - l0) / 1.5, tr = (r1 - r0) / 1.5;
  FLIP_ENC_L = tl < 0;
  FLIP_ENC_R = tr < 0;
  MAX_TPS = min(fabs(tl), fabs(tr)) * 255.0 / 200.0;

  out.printf("Left raw ticks/s: %.1f   Right raw ticks/s: %.1f\n", tl, tr);
  out.printf("-> FLIP_ENC_L = %s\n-> FLIP_ENC_R = %s\n-> MAX_TPS = %.1f\n",
             FLIP_ENC_L ? "true" : "false", FLIP_ENC_R ? "true" : "false", MAX_TPS);
  out.println("Applied live. Hard-code these in config.h.");
  out.println("If a wheel spun BACKWARDS, set its FLIP_MOTOR_x = true, re-upload, run cal again.");
  if (tl == 0 || tr == 0) out.println("WARNING: an encoder read 0 ticks - check wiring.");

  readEncoders(l0, r0);
  posL = lastPosL = FLIP_ENC_L ? -l0 : l0;
  posR = lastPosR = FLIP_ENC_R ? -r0 : r0;
  velL = velR = 0;
  resetSync();
}
