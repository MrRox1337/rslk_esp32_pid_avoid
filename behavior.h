#pragma once
// High-level modes: IDLE, AUTO (obstacle avoidance), MANUAL (speed/turn command).

enum Mode { IDLE, AUTO, MANUAL };
enum AvoidState { CRUISE, BACKUP, SPIN };

extern Mode mode;
extern AvoidState avoidState;

void stopRobot();
void startAuto();
// deadman = true: robot stops unless setManual is called again within WEB_DRIVE_TIMEOUT_MS
void setManual(float speed, float turn, bool deadman = false);
void behaviorStep();     // call every loop
const char *modeName();  // e.g. "IDLE", "MANUAL", "AUTO/spin"
