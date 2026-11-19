#pragma once
// Persistent settings in flash (NVS via Preferences): survive power cycles.
// Stored: wheel PID, sync PID, sync on/off, MAX_TPS, FLIP_ENC_L/R, CRUISE_SPEED.
// Anything never saved falls back to the defaults in config.h.
#include <Arduino.h>

extern bool settingsDirty;          // live values differ from what's saved

void settingsLoad(Print &out);      // call once in setup()
void settingsSave(Print &out);
void settingsClear(Print &out);     // erase saved values (defaults apply after reboot)
