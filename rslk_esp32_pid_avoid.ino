/*
  TI-RSLK MAX chassis (Pololu chassis board) on a Freenove ESP32-WROVER(-E) CAM board
  ------------------------------------------------------------------------------------
  CAMERA MUST BE UNPLUGGED: its pins are reused here.
  Arduino-ESP32 core 3.x (ESP-IDF 5). Board: "ESP32 Wrover Module".

  Files:
    config.h      pins, constants, default tunables   <- edit this one
    pid.h         PID controller
    motors.*      DRV8838 drivers
    encoders.*    PCNT hardware quadrature counting
    control.*     wheel velocity PID + wheel-sync PID, drive(), calibrate()
    sonar.*       HC-SR04 with median filter
    behavior.*    IDLE / AUTO (obstacle avoidance) / MANUAL modes
    console.*     text commands (Serial + web), telemetry
    settings.*    save/load tuned values to flash (Preferences / NVS)
    web.*         WiFi AP + web server; web_page.h is the HTML

  Serial: 115200, "Newline" line ending. Type "help".
  Web:    join WiFi AP_SSID / AP_PASS, open http://192.168.4.1
*/

#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "control.h"
#include "sonar.h"
#include "behavior.h"
#include "console.h"
#include "settings.h"
#include "web.h"

void setup() {
  Serial.begin(SERIAL_BAUD);
  settingsLoad(Serial);   // saved gains/calibration override config.h defaults

  motorsBegin();
  encodersBegin();
  sonarBegin();
  webBegin();

  Serial.println("RSLK + ESP32 ready. Type 'cal' first, then 'go'.");
  printHelp();
}

void loop() {
  consoleStep();     // serial commands + telemetry
  webStep();         // web requests
  sonarUpdate();     // distance (every SONAR_MS)
  behaviorStep();    // decides speed/turn
  controlUpdate();   // PID -> motors (every LOOP_MS)
}
