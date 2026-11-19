/*
  TI-RSLK MAX chassis (Pololu chassis board) on a Freenove ESP32-WROVER(-E) CAM board
  ------------------------------------------------------------------------------------
  CAMERA MUST BE UNPLUGGED: its pins are reused here.
  Arduino-ESP32 core 3.x (ESP-IDF 5). Board: "ESP32 Wrover Module".

  - Encoders counted in hardware by the PCNT peripheral (full 4x quadrature, no ISRs)
  - Per-wheel velocity PID + feedforward
  - Wheel-sync PID so the robot drives straight / turns exactly as commanded
  - drive(speed, turn) in PWM units (-255..255):
        turn > 0 veers RIGHT, turn < 0 veers LEFT, speed 0 + turn = spin in place
  - HC-SR04 obstacle avoidance (ECHO through a 1k/2k divider, or use HC-SR04P)
  - Serial console at 115200, "Newline" line ending. Type "help".
  - Web control: the ESP32 makes its own WiFi network (AP_SSID / AP_PASS below).
    Join it, then open http://192.168.4.1 in a browser. Same commands as Serial,
    plus a hold-to-drive pad (robot stops if you let go or the link drops).
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

// ======================= WIFI =======================
const char *AP_SSID = "RSLK-ESP32";
const char *AP_PASS = "rslk1234";       // min 8 chars (WPA2)
const uint16_t WEB_DRIVE_TIMEOUT_MS = 500;   // pad stops robot if no update this long

WebServer server(80);

// Everything printed through `con` goes to Serial AND (while capturing) to the web reply
class TeePrint : public Print {
public:
  String buf;
  bool capture = false;
  size_t write(uint8_t c) override {
    Serial.write(c);
    if (capture && buf.length() < 2048) buf += (char)c;
    return 1;
  }
  size_t write(const uint8_t *b, size_t n) override {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }
};
TeePrint con;

// ======================= PINS =======================
const int PWML = 25, DIRL = 26, SLPL = 27;    // left  DRV8838
const int PWMR = 32, DIRR = 33, SLPR = 13;    // right DRV8838
const int ELA  = 18, ELB  = 19;               // left encoder
const int ERA  = 23, ERB  = 4;                // right encoder
const int TRIG = 14, ECHO = 34;               // HC-SR04 (34 is input-only; use divider!)
// spare: I2C SDA 21 / SCL 22

const uint32_t PWM_FREQ = 20000;   // 20 kHz: above hearing, fine for DRV8838
const uint8_t  PWM_BITS = 8;       // duty 0..255

// ============ DIRECTION FIXES (set after running "cal") ============
bool FLIP_MOTOR_L = false;
bool FLIP_MOTOR_R = false;
bool FLIP_ENC_L   = false;
bool FLIP_ENC_R   = false;

// ======================= TUNING =======================
const uint16_t LOOP_MS = 10;      // 100 Hz control loop
float MAX_TPS   = 4800.0;         // ticks/s at PWM 255 (4x counting) -> "cal" measures it
float VEL_ALPHA = 0.3;

struct PID {
  float kp, ki, kd, iLim;
  float iTerm = 0, prevErr = 0;
  PID(float p, float i, float d, float lim) : kp(p), ki(i), kd(d), iLim(lim) {}
  float update(float err, float dt) {
    iTerm += ki * err * dt;
    iTerm = constrain(iTerm, -iLim, iLim);
    float deriv = (dt > 0) ? (err - prevErr) / dt : 0;
    prevErr = err;
    return kp * err + iTerm + kd * deriv;
  }
  void reset() { iTerm = 0; prevErr = 0; }
};

//            kp     ki     kd   integral limit
PID pidL   (0.04,  0.25,  0.0, 120);   // ticks/s error -> PWM
PID pidR   (0.04,  0.25,  0.0, 120);
PID pidSync(4.0,   1.0,   0.0, 1200);  // tick error -> ticks/s correction

int   CRUISE_SPEED = 150;
int   MIN_SPEED    = 70;
int   BACK_SPEED   = 110;
int   SPIN_SPEED   = 110;
float SLOW_CM  = 50;
float STOP_CM  = 20;
float CLEAR_CM = 45;
const uint16_t BACKUP_MS   = 400;
const uint16_t SPIN_MIN_MS = 250;
const uint16_t SPIN_MAX_MS = 2500;
const uint16_t SONAR_MS    = 60;

// ======================= STATE =======================
pcnt_unit_handle_t pcntL = nullptr, pcntR = nullptr;

float cmdSpeed = 0, cmdTurn = 0;
float tgtL = 0, tgtR = 0;
float velL = 0, velR = 0;
long  posL = 0, posR = 0, lastPosL = 0, lastPosR = 0;
float expectedDiff = 0;
float syncErr = 0;
int   outL = 0, outR = 0;
bool  syncEnabled = true;
bool  telemetry = true;

float distCm = 400;
float sonarHist[3] = {400, 400, 400};
uint8_t sonarIdx = 0;

enum Mode { IDLE, AUTO, MANUAL };
enum AvoidState { CRUISE, BACKUP, SPIN };
Mode mode = IDLE;
AvoidState avoidState = CRUISE;
unsigned long stateT0 = 0;
int spinDir = 1;
float manSpeed = 0, manTurn = 0;
bool  webDrive = false;                 // MANUAL mode driven by the web pad (dead-man)
unsigned long lastWebDrive = 0;
volatile bool clientDropped = false;    // set from WiFi event when a client leaves the AP

// ======================= ENCODERS (PCNT) =======================
const int PCNT_LIMIT = 30000;

pcnt_unit_handle_t setupEncoder(int pinA, int pinB) {
  pcnt_unit_config_t ucfg = {};
  ucfg.low_limit  = -PCNT_LIMIT;
  ucfg.high_limit =  PCNT_LIMIT;
  ucfg.flags.accum_count = 1;                 // keep counting past the 16-bit limit
  pcnt_unit_handle_t unit;
  ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &unit));

  pcnt_glitch_filter_config_t fcfg = {};
  fcfg.max_glitch_ns = 1000;                  // ignore pulses shorter than 1 us
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &fcfg));

  pcnt_chan_config_t ca = {};
  ca.edge_gpio_num = pinA; ca.level_gpio_num = pinB;
  pcnt_channel_handle_t chA;
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &ca, &chA));

  pcnt_chan_config_t cb = {};
  cb.edge_gpio_num = pinB; cb.level_gpio_num = pinA;
  pcnt_channel_handle_t chB;
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &cb, &chB));

  // Full quadrature decoding (both edges of both channels)
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  // Watch points at the limits are required for accum_count to work
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit,  PCNT_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, -PCNT_LIMIT));

  gpio_pullup_en((gpio_num_t)pinA);           // weak pull-ups, harmless if encoder drives the line
  gpio_pullup_en((gpio_num_t)pinB);

  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));
  return unit;
}

void readEncoders(long &l, long &r) {
  int cl = 0, cr = 0;
  pcnt_unit_get_count(pcntL, &cl);
  pcnt_unit_get_count(pcntR, &cr);
  l = cl; r = cr;
}

// ======================= MOTORS =======================
void setMotor(int pwmPin, int dirPin, int u, bool flip) {
  if (flip) u = -u;
  u = constrain(u, -255, 255);
  digitalWrite(dirPin, u < 0 ? HIGH : LOW);
  ledcWrite(pwmPin, abs(u));
}

void setMotors(int l, int r) {
  setMotor(PWML, DIRL, l, FLIP_MOTOR_L);
  setMotor(PWMR, DIRR, r, FLIP_MOTOR_R);
}

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

int wheelOut(float setpoint, float vel, PID &pid, float dt) {
  float ff = setpoint / MAX_TPS * 255.0;
  float u  = ff + pid.update(setpoint - vel, dt);
  return (int)constrain(u, -255, 255);
}

void controlStep(float dt) {
  long l, r;
  readEncoders(l, r);
  posL = FLIP_ENC_L ? -l : l;
  posR = FLIP_ENC_R ? -r : r;

  float rawL = (posL - lastPosL) / dt;
  float rawR = (posR - lastPosR) / dt;
  lastPosL = posL; lastPosR = posR;
  velL += VEL_ALPHA * (rawL - velL);
  velR += VEL_ALPHA * (rawR - velR);

  tgtL = (cmdSpeed + cmdTurn) / 255.0 * MAX_TPS;
  tgtR = (cmdSpeed - cmdTurn) / 255.0 * MAX_TPS;

  if (tgtL == 0 && tgtR == 0) {
    setMotors(0, 0);
    outL = outR = 0;
    pidL.reset(); pidR.reset();
    resetSync();
    return;
  }

  expectedDiff += (tgtL - tgtR) * dt;
  syncErr = (posL - posR) - expectedDiff;   // + = left wheel has gone too far
  float corr = syncEnabled ? pidSync.update(syncErr, dt) : 0;
  corr = constrain(corr, -0.3 * MAX_TPS, 0.3 * MAX_TPS);

  outL = wheelOut(tgtL - corr, velL, pidL, dt);
  outR = wheelOut(tgtR + corr, velR, pidR, dt);
  setMotors(outL, outR);
}

// ======================= ULTRASONIC =======================
float readSonar() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long us = pulseIn(ECHO, HIGH, 12000UL);
  if (us == 0) return 400;
  return us / 58.0;
}

float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

// ======================= AVOIDANCE =======================
void setAvoidState(AvoidState s) {
  avoidState = s;
  stateT0 = millis();
  resetSync();
}

void avoidStep() {
  unsigned long t = millis() - stateT0;
  switch (avoidState) {
    case CRUISE: {
      if (distCm < STOP_CM) { setAvoidState(BACKUP); break; }
      float sp = CRUISE_SPEED;
      if (distCm < SLOW_CM)
        sp = MIN_SPEED + (CRUISE_SPEED - MIN_SPEED) * (distCm - STOP_CM) / (SLOW_CM - STOP_CM);
      drive(sp, 0);
    } break;
    case BACKUP:
      drive(-BACK_SPEED, 0);
      if (t > BACKUP_MS) { spinDir = -spinDir; setAvoidState(SPIN); }
      break;
    case SPIN:
      drive(0, spinDir * SPIN_SPEED);
      if (t > SPIN_MIN_MS && distCm > CLEAR_CM) setAvoidState(CRUISE);
      else if (t > SPIN_MAX_MS) { spinDir = -spinDir; setAvoidState(SPIN); }
      break;
  }
}

// ======================= CALIBRATION =======================
void calibrate() {
  mode = IDLE;
  con.println("CAL: both motors PWM 200 for 2 s. Wheels off the ground or give it room.");
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

  con.printf("Left raw ticks/s: %.1f   Right raw ticks/s: %.1f\n", tl, tr);
  con.printf("-> FLIP_ENC_L = %s\n-> FLIP_ENC_R = %s\n-> MAX_TPS = %.1f\n",
                FLIP_ENC_L ? "true" : "false", FLIP_ENC_R ? "true" : "false", MAX_TPS);
  con.println("Applied live. Hard-code these at the top of the sketch.");
  con.println("If a wheel spun BACKWARDS, set its FLIP_MOTOR_x = true, re-upload, run cal again.");
  if (tl == 0 || tr == 0) con.println("WARNING: an encoder read 0 ticks - check wiring.");

  readEncoders(l0, r0);
  posL = lastPosL = FLIP_ENC_L ? -l0 : l0;
  posR = lastPosR = FLIP_ENC_R ? -r0 : r0;
  velL = velR = 0;
  resetSync();
}

// ======================= SERIAL CONSOLE =======================
void printHelp() {
  con.println("go | stop | m S T (manual speed,turn) | cal | sync 0/1 | tele 0/1");
  con.println("kp/ki/kd X (wheel PID) | sp/si/sd X (sync PID) | cruise X | gains");
}

void printGains() {
  con.printf("wheel kp,ki,kd = %.3f, %.3f, %.3f\n", pidL.kp, pidL.ki, pidL.kd);
  con.printf("sync  kp,ki,kd = %.3f, %.3f, %.3f\n", pidSync.kp, pidSync.ki, pidSync.kd);
  con.printf("MAX_TPS = %.1f\n", MAX_TPS);
}

void runCmd(char *s) {
  char *cmd = strtok(s, " ");
  if (!cmd) return;
  char *a = strtok(NULL, " ");
  char *b = strtok(NULL, " ");
  float x = a ? atof(a) : 0, y = b ? atof(b) : 0;
  webDrive = false;   // any typed command takes over from the web drive pad

  if      (!strcmp(cmd, "go"))     { mode = AUTO; setAvoidState(CRUISE); }
  else if (!strcmp(cmd, "stop"))   { mode = IDLE; }
  else if (!strcmp(cmd, "m"))      { mode = MANUAL; manSpeed = x; manTurn = y; resetSync(); }
  else if (!strcmp(cmd, "cal"))    { calibrate(); }
  else if (!strcmp(cmd, "sync"))   { syncEnabled = x != 0; resetSync(); }
  else if (!strcmp(cmd, "tele"))   { telemetry = x != 0; }
  else if (!strcmp(cmd, "cruise")) { CRUISE_SPEED = (int)x; }
  else if (!strcmp(cmd, "kp"))     { pidL.kp = pidR.kp = x; }
  else if (!strcmp(cmd, "ki"))     { pidL.ki = pidR.ki = x; pidL.reset(); pidR.reset(); }
  else if (!strcmp(cmd, "kd"))     { pidL.kd = pidR.kd = x; }
  else if (!strcmp(cmd, "sp"))     { pidSync.kp = x; }
  else if (!strcmp(cmd, "si"))     { pidSync.ki = x; pidSync.reset(); }
  else if (!strcmp(cmd, "sd"))     { pidSync.kd = x; }
  else if (!strcmp(cmd, "gains"))  { printGains(); return; }
  else { printHelp(); return; }
  printGains();
}

void handleSerial() {
  static char buf[32];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { buf[n] = 0; runCmd(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

// ======================= WEB CONTROL =======================
const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RSLK Control</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--fg:#1d2330;--mut:#6b7280;--line:#d9dce3;--acc:#2563eb;--go:#16a34a;--stop:#dc2626}
@media(prefers-color-scheme:dark){:root{--bg:#111318;--card:#1b1e25;--fg:#e6e8ee;--mut:#9aa1ad;--line:#2c313b;--acc:#60a5fa;--go:#22c55e;--stop:#ef4444}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif}
main{max-width:560px;margin:0 auto;padding:16px;display:grid;gap:12px}
h1{font-size:18px;margin:0}.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
.stat{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.stat div{font-size:12px;color:var(--mut)}
.stat b{display:block;font-size:17px;color:var(--fg);font-variant-numeric:tabular-nums}
.row{display:flex;gap:8px;flex-wrap:wrap}button{flex:1;min-width:64px;padding:12px 8px;font:inherit;font-weight:600;
border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--fg);cursor:pointer;touch-action:none;user-select:none}
button:active,button.on{background:var(--acc);color:#fff;border-color:var(--acc)}
.go{background:var(--go);color:#fff;border:0}.stop{background:var(--stop);color:#fff;border:0}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.pad button{padding:20px 0;font-size:20px}
label{font-size:13px;color:var(--mut);display:flex;align-items:center;gap:8px}input[type=range]{flex:1}
form{display:flex;gap:8px}input[type=text]{flex:1;min-width:0;padding:10px;font:14px ui-monospace,monospace;
border:1px solid var(--line);border-radius:8px;background:var(--bg);color:var(--fg)}form button{flex:0 0 auto}
pre{margin:8px 0 0;height:180px;overflow:auto;font:12px/1.4 ui-monospace,monospace;background:var(--bg);
border-radius:8px;padding:8px;white-space:pre-wrap}#link{font-size:12px;color:var(--mut)}
</style></head><body><main>
<div class="row" style="align-items:center"><h1 style="flex:1">RSLK Control</h1><span id="link">connecting…</span></div>
<div class="card stat">
<div>Mode<b id="mode">–</b></div><div>Distance<b id="dist">–</b></div><div>Sync err<b id="sync">–</b></div>
<div>L tgt / vel<b id="l">–</b></div><div>R tgt / vel<b id="r">–</b></div><div>PWM L / R<b id="pwm">–</b></div>
</div>
<div class="card row"><button class="go" data-c="go">GO (auto)</button><button class="stop" data-c="stop">STOP</button>
<button data-c="cal">Calibrate</button><button data-c="gains">Gains</button></div>
<div class="card"><div class="pad" id="pad">
<button data-s="1" data-t="-.5">↖</button><button data-s="1" data-t="0">↑</button><button data-s="1" data-t=".5">↗</button>
<button data-s="0" data-t="-1">⟲</button><button class="stop" data-c="stop">■</button><button data-s="0" data-t="1">⟳</button>
<button data-s="-1" data-t="-.5">↙</button><button data-s="-1" data-t="0">↓</button><button data-s="-1" data-t=".5">↘</button>
</div><label style="margin-top:10px">Speed <input type="range" id="spd" min="60" max="230" value="140"><span id="spdv">140</span></label>
<div style="font-size:12px;color:var(--mut);margin-top:4px">Hold a button to drive; release to stop.</div></div>
<div class="card"><form id="f"><input type="text" id="c" placeholder="command, e.g. kp 0.05  |  m 120 0  |  help" autocomplete="off" autocapitalize="off"><button>Send</button></form>
<pre id="log"></pre></div>
</main><script>
const $=id=>document.getElementById(id),log=$('log');
function out(t){log.textContent+=t;log.scrollTop=log.scrollHeight}
async function cmd(c){out('> '+c+'\n');try{const r=await fetch('/cmd?c='+encodeURIComponent(c));out(await r.text())}catch(e){out('(no reply)\n')}}
document.querySelectorAll('[data-c]').forEach(b=>b.addEventListener('click',()=>{stopPad();cmd(b.dataset.c)}));
$('f').onsubmit=e=>{e.preventDefault();const c=$('c').value.trim();if(c)cmd(c);$('c').value=''};
const spd=$('spd');spd.oninput=()=>$('spdv').textContent=spd.value;
let timer=null,held=null;
function send(){const v=+spd.value;fetch('/drive?s='+Math.round(held.dataset.s*v)+'&t='+Math.round(held.dataset.t*v)).catch(()=>{})}
function stopPad(){if(!held)return;clearInterval(timer);held.classList.remove('on');held=null;fetch('/cmd?c=stop').catch(()=>{})}
document.querySelectorAll('#pad [data-s]').forEach(b=>{
 b.addEventListener('pointerdown',e=>{e.preventDefault();stopPad();held=b;b.classList.add('on');send();timer=setInterval(send,150)});
 ['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,stopPad))});
document.addEventListener('visibilitychange',stopPad);
let busy=false;setInterval(async()=>{if(busy)return;busy=true;try{const s=await (await fetch('/status')).json();
$('mode').textContent=s.mode;$('dist').textContent=s.dist.toFixed(0)+' cm';$('sync').textContent=s.sync.toFixed(0);
$('l').textContent=s.tgtL.toFixed(0)+' / '+s.velL.toFixed(0);$('r').textContent=s.tgtR.toFixed(0)+' / '+s.velR.toFixed(0);
$('pwm').textContent=s.outL+' / '+s.outR;$('link').textContent='connected'}catch(e){$('link').textContent='no link'}busy=false},300);
</script></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleCmd() {
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

void handleDrive() {
  mode = MANUAL;
  float s = server.arg("s").toFloat(), t = server.arg("t").toFloat();
  if (!webDrive || s != manSpeed || t != manTurn) resetSync();
  manSpeed = s; manTurn = t;
  webDrive = true;
  lastWebDrive = millis();
  server.send(200, "text/plain", "ok");
}

void handleStatus() {
  static const char *modeNames[]  = {"IDLE", "AUTO", "MANUAL"};
  static const char *avoidNames[] = {"cruise", "backup", "spin"};
  char m[24];
  if (mode == AUTO) snprintf(m, sizeof(m), "AUTO/%s", avoidNames[avoidState]);
  else              snprintf(m, sizeof(m), "%s", modeNames[mode]);
  char j[200];
  snprintf(j, sizeof(j),
           "{\"mode\":\"%s\",\"dist\":%.1f,\"tgtL\":%.0f,\"velL\":%.0f,\"tgtR\":%.0f,\"velR\":%.0f,"
           "\"sync\":%.0f,\"outL\":%d,\"outR\":%d}",
           m, distCm, tgtL, velL, tgtR, velR, syncErr, outL, outR);
  server.send(200, "application/json", j);
}

void setupWeb() {
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
  if (webDrive && millis() - lastWebDrive > WEB_DRIVE_TIMEOUT_MS) {   // dead-man release
    webDrive = false;
    mode = IDLE;
  }
  if (clientDropped) {          // someone left the AP: stop, to be safe
    clientDropped = false;
    webDrive = false;
    mode = IDLE;
    Serial.println("WiFi client disconnected -> STOP");
  }
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);

  pinMode(DIRL, OUTPUT); pinMode(SLPL, OUTPUT);
  pinMode(DIRR, OUTPUT); pinMode(SLPR, OUTPUT);
  ledcAttach(PWML, PWM_FREQ, PWM_BITS);
  ledcAttach(PWMR, PWM_FREQ, PWM_BITS);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);

  setMotors(0, 0);
  digitalWrite(SLPL, HIGH);
  digitalWrite(SLPR, HIGH);

  pcntL = setupEncoder(ELA, ELB);
  pcntR = setupEncoder(ERA, ERB);

  setupWeb();

  Serial.println("RSLK + ESP32 ready. Type 'cal' first, then 'go'.");
  printHelp();
}

void loop() {
  static unsigned long lastSonar = 0, lastTele = 0;
  static unsigned long lastCtrlUs = micros();
  unsigned long now = millis();

  handleSerial();
  webStep();

  if (now - lastSonar >= SONAR_MS) {
    lastSonar = now;
    sonarHist[sonarIdx] = readSonar();
    sonarIdx = (sonarIdx + 1) % 3;
    distCm = median3(sonarHist[0], sonarHist[1], sonarHist[2]);
  }

  if (mode == AUTO) {
    avoidStep();
  } else if (mode == MANUAL) {
    if (distCm < STOP_CM && manSpeed > 0) drive(0, manTurn);
    else                                  drive(manSpeed, manTurn);
  } else {
    drive(0, 0);
  }

  unsigned long us = micros();
  if (us - lastCtrlUs >= LOOP_MS * 1000UL) {
    float dt = (us - lastCtrlUs) * 1e-6;
    lastCtrlUs = us;
    controlStep(dt);
  }

  if (telemetry && now - lastTele >= 100) {
    lastTele = now;
    Serial.printf("tgtL:%.0f,velL:%.0f,tgtR:%.0f,velR:%.0f,syncErr:%.0f,dist:%.1f\n",
                  tgtL, velL, tgtR, velR, syncErr, distCm);
  }
}
