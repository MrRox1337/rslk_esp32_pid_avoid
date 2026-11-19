#include "web.h"
#include <WiFi.h>
#include <WebServer.h>
#include "config.h"
#include "console.h"
#include "control.h"
#include "behavior.h"
#include "sonar.h"
#include "web_page.h"

static WebServer server(80);
static volatile bool clientDropped = false;   // set from WiFi event task

static void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

static void handleCmd() {
  char buf[64];
  strncpy(buf, server.arg("c").c_str(), sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  con.buf = "";
  con.capture = true;
  runCmd(buf);
  con.capture = false;
  server.send(200, "text/plain", con.buf.length() ? con.buf : String("ok\n"));
  con.buf = "";
}

static void handleDrive() {
  setManual(server.arg("s").toFloat(), server.arg("t").toFloat(), true);
  server.send(200, "text/plain", "ok");
}

static void handleStatus() {
  char j[200];
  snprintf(j, sizeof(j),
           "{\"mode\":\"%s\",\"dist\":%.1f,\"tgtL\":%.0f,\"velL\":%.0f,\"tgtR\":%.0f,\"velR\":%.0f,"
           "\"sync\":%.0f,\"outL\":%d,\"outR\":%d}",
           modeName(), distCm, tgtL, velL, tgtR, velR, syncErr, outL, outR);
  server.send(200, "application/json", j);
}

void webBegin() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);                       // lower latency for driving
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) { clientDropped = true; },
               ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/drive", handleDrive);
  server.on("/status", handleStatus);
  server.onNotFound([]() { server.sendHeader("Location", "/"); server.send(302); });
  server.begin();

  Serial.printf("WiFi AP \"%s\" (pass \"%s\") -> open http://%s\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}

void webStep() {
  server.handleClient();
  if (clientDropped) {          // someone left the AP: stop, to be safe
    clientDropped = false;
    stopRobot();
    Serial.println("WiFi client disconnected -> STOP");
  }
}
