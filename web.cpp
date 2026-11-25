#include "web.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "config.h"
#include "console.h"
#include "control.h"
#include "behavior.h"
#include "sonar.h"
#include "settings.h"
#include "web_page.h"

static WebServer server(80);
static volatile bool clientDropped = false;   // set from WiFi event task
static unsigned long speedChangedAt = 0;      // pending slider save (0 = none)

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

// Speed slider: becomes the AUTO cruise speed. Saved to flash SPEED_SAVE_MS after the last
// change, so a burst of adjustments costs one write.
static void handleSpeed() {
  CRUISE_SPEED = constrain(server.arg("v").toInt(), 0, 255);
  speedChangedAt = millis() | 1;
  server.send(200, "text/plain", "ok");
}

static void handleRec() {           // flight recorder as a CSV download, streamed in chunks
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Disposition", "attachment; filename=rslk_run.csv");
  server.send(200, "text/csv", "");
  String chunk;
  char line[120];
  recHeader(line, sizeof(line));
  chunk = line;
  for (int i = 0, n = recCount(); i < n; i++) {
    recLine(i, line, sizeof(line));
    chunk += line;
    if (chunk.length() > 2000) { server.sendContent(chunk); chunk = ""; }
  }
  server.sendContent(chunk);
  server.sendContent("");             // end of chunked reply
}

static void handleStatus() {
  char j[256];
  snprintf(j, sizeof(j),
           "{\"mode\":\"%s\",\"dist\":%.1f,\"tgtL\":%.0f,\"velL\":%.0f,\"tgtR\":%.0f,\"velR\":%.0f,"
           "\"sync\":%.0f,\"outL\":%d,\"outR\":%d,\"cruise\":%d,\"lane\":%.1f}",
           modeName(), distCm, tgtL, velL, tgtR, velR, syncErr, outL, outR, CRUISE_SPEED, laneY);
  server.send(200, "application/json", j);
}

static void startOwnAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("WiFi: own network \"%s\" (pass \"%s\") -> open http://%s\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}

void webBegin() {
  WiFi.setHostname(HOSTNAME);
#ifdef WIFI_SSID
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi: joining \"%s\"", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_MS) { delay(250); Serial.print('.'); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi: connected -> open http://%s or http://%s.local\n",
                  WiFi.localIP().toString().c_str(), HOSTNAME);
  } else {
    Serial.println("WiFi: could not join, starting own network instead");
    WiFi.disconnect(true);
    startOwnAP();
  }
#else
  startOwnAP();
#endif
  WiFi.setSleep(false);                       // lower latency for driving
  MDNS.begin(HOSTNAME);
  MDNS.addService("http", "tcp", 80);

  // Stop to be safe when the link to the phone breaks:
  // a device leaves our own network, or we drop off the phone's hotspot
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) { clientDropped = true; },
               ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) { clientDropped = true; },
               ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/drive", handleDrive);
  server.on("/status", handleStatus);
  server.on("/speed", handleSpeed);
  server.on("/rec", handleRec);
  server.onNotFound([]() { server.sendHeader("Location", "/"); server.send(302); });
  server.begin();
}

void webStep() {
  server.handleClient();
  if (speedChangedAt && millis() - speedChangedAt >= SPEED_SAVE_MS) {
    speedChangedAt = 0;
    settingsSaveCruise();
  }
  if (clientDropped) {          // WiFi link lost: stop, to be safe
    clientDropped = false;
    if (mode != IDLE) Serial.println("WiFi link lost -> STOP");   // (reconnect retries repeat this event)
    stopRobot();
  }
}
