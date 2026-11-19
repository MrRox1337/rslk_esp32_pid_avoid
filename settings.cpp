#include "settings.h"
#include <Preferences.h>
#include "config.h"
#include "control.h"

static Preferences prefs;
static constexpr char NS[] = "rslk";     // NVS namespace (max 15 chars, keys too)

bool settingsDirty = false;

void settingsLoad(Print &out) {
  prefs.begin(NS, true);                 // read-only
  bool any = prefs.isKey("kp");
  // second argument = value to use if the key was never saved (config.h defaults)
  pidL.kp = pidR.kp = prefs.getFloat("kp", pidL.kp);
  pidL.ki = pidR.ki = prefs.getFloat("ki", pidL.ki);
  pidL.kd = pidR.kd = prefs.getFloat("kd", pidL.kd);
  pidSync.kp  = prefs.getFloat("sp", pidSync.kp);
  pidSync.ki  = prefs.getFloat("si", pidSync.ki);
  pidSync.kd  = prefs.getFloat("sd", pidSync.kd);
  syncEnabled = prefs.getBool("sync", syncEnabled);
  MAX_TPS     = prefs.getFloat("maxtps", MAX_TPS);
  FLIP_ENC_L  = prefs.getBool("flipEncL", FLIP_ENC_L);
  FLIP_ENC_R  = prefs.getBool("flipEncR", FLIP_ENC_R);
  CRUISE_SPEED = prefs.getInt("cruise", CRUISE_SPEED);
  prefs.end();
  settingsDirty = false;
  out.println(any ? "Settings loaded from flash." : "No saved settings; using config.h defaults.");
}

void settingsSave(Print &out) {
  prefs.begin(NS, false);
  prefs.putFloat("kp", pidL.kp);
  prefs.putFloat("ki", pidL.ki);
  prefs.putFloat("kd", pidL.kd);
  prefs.putFloat("sp", pidSync.kp);
  prefs.putFloat("si", pidSync.ki);
  prefs.putFloat("sd", pidSync.kd);
  prefs.putBool("sync", syncEnabled);
  prefs.putFloat("maxtps", MAX_TPS);
  prefs.putBool("flipEncL", FLIP_ENC_L);
  prefs.putBool("flipEncR", FLIP_ENC_R);
  prefs.putInt("cruise", CRUISE_SPEED);
  prefs.end();
  settingsDirty = false;
  out.println("Settings saved to flash.");
}

void settingsClear(Print &out) {
  prefs.begin(NS, false);
  prefs.clear();
  prefs.end();
  out.println("Saved settings erased. Rebooting to config.h defaults...");
}
