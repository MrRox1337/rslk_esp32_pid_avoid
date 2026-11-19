#pragma once
// WiFi access point + web control page at http://192.168.4.1
//   /          control page (web_page.h)
//   /cmd?c=..  run a console command, returns its text output
//   /drive?s=&t=  manual drive with dead-man timeout (used by the pad)
//   /status    JSON telemetry

void webBegin();
void webStep();   // call every loop
