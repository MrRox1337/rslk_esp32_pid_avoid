#pragma once
// WiFi (joins WIFI_SSID, falls back to its own access point) + web control page
//   /          control page (web_page.h)
//   /cmd?c=..  run a console command, returns its text output
//   /drive?s=&t=  manual drive with dead-man timeout (used by the pad)
//   /status    JSON telemetry
//   /speed?v=  speed slider (cruise speed, saved after SPEED_SAVE_MS)
//   /rec       flight recorder CSV of the last drive

void webBegin();
void webStep();   // call every loop
