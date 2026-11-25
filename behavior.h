#pragma once
// High-level modes: IDLE, AUTO (obstacle avoidance), MANUAL (speed/turn command).

enum Mode { IDLE, AUTO, MANUAL };
// CRUISE: hold the lane. PASS_OUT: pull out and pass. PASS_BACK: merge back. BACKUP/SPIN: fallback.
enum AvoidState { CRUISE, PASS_OUT, PASS_BACK, BACKUP, SPIN };

extern Mode mode;
extern AvoidState avoidState;
extern float laneY;      // odometry: sideways offset from the lane in cm (+ = right)

void stopRobot();
void startAuto();
// deadman = true: robot stops unless setManual is called again within WEB_DRIVE_TIMEOUT_MS
void setManual(float speed, float turn, bool deadman = false);
void behaviorStep();     // call every loop
const char *modeName();  // e.g. "IDLE", "MANUAL", "AUTO/spin"
