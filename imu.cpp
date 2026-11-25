#include "imu.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>

bool imuOk = false;

static Adafruit_BNO055 *bno = nullptr;
static float lastRaw = 0, total = 0;   // degrees
static uint8_t badReads = 0;

void imuBegin(Print &out) {
  // Idle I2C lines must sit HIGH (pull-ups on the sensor board). LOW = unpowered or miswired.
  pinMode(IMU_SDA, INPUT); pinMode(IMU_SCL, INPUT);
  delay(2);
  int sdaIdle = digitalRead(IMU_SDA), sclIdle = digitalRead(IMU_SCL);
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(100000);               // BNO055 stretches the clock; 100 kHz is the safe speed on ESP32
  uint8_t addr = 0;
  for (unsigned long t0 = millis(); !addr && millis() - t0 < 1000; delay(50)) {   // BNO055 boots in ~650 ms
    for (uint8_t a : {0x28, 0x29}) {   // ADR pin low / high
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { addr = a; break; }
    }
  }
  if (!addr) {
    out.print("IMU: no BNO055 at 0x28/0x29, using encoder heading. I2C devices found:");
    int n = 0;
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { out.printf(" 0x%02X", a); n++; }
    }
    out.println(n ? "" : " none (check SDA 21 / SCL 22, 3.3 V, GND)");
    out.printf("IMU: idle line levels SDA(%d)=%s SCL(%d)=%s (both should be HIGH)\n",
               IMU_SDA, sdaIdle ? "HIGH" : "LOW", IMU_SCL, sclIdle ? "HIGH" : "LOW");
    Wire.end();                                  // diagnostic: are SDA and SCL swapped?
    Wire.begin(IMU_SCL, IMU_SDA);
    for (uint8_t a : {0x28, 0x29}) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0)
        out.printf("IMU: found 0x%02X with SDA/SCL SWAPPED - swap the two wires (or IMU_SDA/IMU_SCL)\n", a);
    }
    Wire.end();
    return;
  }

  bno = new Adafruit_BNO055(55, addr, &Wire);
  if (!bno->begin(OPERATION_MODE_IMUPLUS)) {
    out.println("IMU: BNO055 did not start, using encoder heading.");
    return;
  }
  delay(100);
  lastRaw = bno->getVector(Adafruit_BNO055::VECTOR_EULER).x();
  total = 0;
  imuOk = true;
  out.printf("IMU: BNO055 at 0x%02X, IMUPLUS mode. Heading from the gyro.\n", addr);
}

void imuUpdate() {
  static unsigned long last = 0;
  if (!imuOk || millis() - last < IMU_MS) return;
  last = millis();

  float raw = bno->getVector(Adafruit_BNO055::VECTOR_EULER).x();   // 0..360 deg
  float d = raw - lastRaw;
  if (d > 180) d -= 360;
  if (d < -180) d += 360;
  // The robot can't turn more than ~4 deg in 10 ms; a bigger jump is a bad I2C read
  if (fabsf(d) > 20) {
    if (++badReads > 50) { imuOk = false; Serial.println("IMU: lost, falling back to encoder heading."); }
    return;
  }
  badReads = 0;
  lastRaw = raw;
  total += d;
}

float imuHeading() { return radians(total) * IMU_SIGN; }

void imuStatus(Print &out) {
  if (!imuOk) { out.println("IMU: not available (encoder heading in use)"); return; }
  uint8_t sys, gyro, accel, mag;
  bno->getCalibration(&sys, &gyro, &accel, &mag);
  out.printf("IMU heading %.1f deg (turn right -> should increase)  calib gyro %u/3 accel %u/3\n",
             degrees(imuHeading()), gyro, accel);
}
