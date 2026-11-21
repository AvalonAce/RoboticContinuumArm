// RCM sensor board v1 — Adafruit QT Py ESP32. Same firmware on every sensor board:
//   * IMU lines if an ICM-20948 is on STEMMA QT, flex lines always (unconnected pins read noise).
// Checksummed CSV over USB (same "*HH" scheme as the servo board):
//   I,t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,temp*HH   accel m/s^2, gyro rad/s, mag uT, degC (IMU axes), ~100 Hz
//   F,t_ms,mv1,mv2,mv3*HH                        flex divider voltages in mV (A2, A3, TX), ~50 Hz
// Flex wiring: 3V -> flex -> pin -> 10-15 kOhm -> GND. Voltage drops as the sensor bends.
// Filtering/orientation happens on the laptop, so raw data is always available for analysis.
//
// Commands (type `help`): scan | info | rate <hz> | frate <hz> | stream on|off

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ICM20948.h>

static const uint32_t DEFAULT_HZ = 100;

Adafruit_ICM20948 icm;
static bool imuOk = false;
static TwoWire *imuBus = nullptr;
static uint8_t imuAddr = 0;
static uint32_t periodUs = 1000000 / DEFAULT_HZ;
static bool streaming = true;
static uint32_t lastUs = 0;
static bool imuWarned = false;
// Flex inputs: A2, A3, TX — ADC1 on the S2/S3 (GPIO 9, 8, 5)
static const uint8_t FLEX_PINS[3] = {A2, A3, TX};
static const int FLEX_AVG = 8;                       // samples averaged per reading
static uint32_t flexPeriodUs = 1000000 / 50;
static uint32_t lastFlexUs = 0;

static void printChecked(const char *body) {
  uint8_t x = 0;
  for (const char *c = body; *c; c++) x ^= (uint8_t)*c;
  Serial.printf("%s*%02X\n", body, x);
}

static void scanBus(TwoWire &w, const char *name) {
  Serial.printf("%s:", name);
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    w.beginTransmission(a);
    if (w.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.println(n ? "" : " (nothing)");
}

static bool tryImu(TwoWire &w, uint8_t addr) {
  if (!icm.begin_I2C(addr, &w)) return false;
  imuBus = &w; imuAddr = addr;
  icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);       // +-4 g: ample for a slow arm, better resolution
  icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  icm.setAccelRateDivisor(10);                       // 1125/(1+10) ~ 102 Hz
  icm.setGyroRateDivisor(10);                        // 1100/(1+10) = 100 Hz
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  return true;
}

static void findImu() {
  imuOk = false;
  // STEMMA QT port = Wire1 on QT Py boards; castellated SDA/SCL pads = Wire
  const uint8_t addrs[] = {0x69, 0x68};
  for (uint8_t a : addrs) if (tryImu(Wire1, a)) { imuOk = true; break; }
  if (!imuOk) for (uint8_t a : addrs) if (tryImu(Wire, a)) { imuOk = true; break; }
  if (imuOk) { Serial.printf("imu ICM-20948 at 0x%02X on %s\n", imuAddr, imuBus == &Wire1 ? "STEMMA QT (Wire1)" : "SDA/SCL pads (Wire)"); imuWarned = false; }
  else if (!imuWarned) {                     // say it once; a flex-only board has no IMU
    imuWarned = true;
    Serial.println("info: no ICM-20948 found (fine on a flex-only board). I2C scan:");
    scanBus(Wire1, "  Wire1 (STEMMA QT)"); scanBus(Wire, "  Wire (pads)");
  }
}

static void handleLine(String line) {
  line.trim();
  if (!line.length()) return;
  if (line == "help") Serial.println("scan | info | rate <hz 1..200> | frate <hz 1..200> | stream on|off");
  else if (line == "scan") { scanBus(Wire1, "Wire1 (STEMMA QT)"); scanBus(Wire, "Wire (pads)"); }
  else if (line == "info") {
    if (imuOk) Serial.printf("info imu=ICM-20948 addr=0x%02X bus=%s rate=%lu stream=%d\n", imuAddr,
                             imuBus == &Wire1 ? "Wire1" : "Wire", 1000000UL / periodUs, streaming);
    else Serial.println("info imu=none");
    Serial.printf("info flex pins=%d,%d,%d rate=%lu\n", FLEX_PINS[0], FLEX_PINS[1], FLEX_PINS[2], 1000000UL / flexPeriodUs);
  }
  else if (line.startsWith("frate ")) {
    long hz = constrain(line.substring(6).toInt(), 1, 200);
    flexPeriodUs = 1000000UL / hz;
    Serial.printf("frate %ld\n", hz);
  }
  else if (line.startsWith("rate ")) {
    long hz = constrain(line.substring(5).toInt(), 1, 200);
    periodUs = 1000000UL / hz;
    Serial.printf("rate %ld\n", hz);
  }
  else if (line == "stream on") { streaming = true; Serial.println("stream on"); }
  else if (line == "stream off") { streaming = false; Serial.println("stream off"); }
  else Serial.printf("? [%s] (type help)\n", line.c_str());
}

void setup() {
  Serial.begin(921600);                      // native-USB boards ignore the baud; Pico uses it
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  Wire1.begin(SDA1, SCL1);                   // STEMMA QT (explicit: default-pin lookup is unreliable)
  Wire.begin(SDA, SCL);                      // castellated SDA/SCL pads
  analogReadResolution(12);
  for (uint8_t p : FLEX_PINS) analogSetPinAttenuation(p, ADC_11db);   // ~0-3.1 V range
  Wire1.setClock(400000);
  Wire.setClock(400000);
  Serial.println("\nRCM sensors v0. type `help`");
  findImu();
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleLine(buf); buf = ""; }
    else if (ch >= 32 && ch < 127) buf += ch;
  }
  uint32_t now = micros();
  if (streaming && now - lastFlexUs >= flexPeriodUs) {
    lastFlexUs = now;
    uint32_t mv[3];
    for (int k = 0; k < 3; k++) {
      uint32_t acc = 0;
      for (int j = 0; j < FLEX_AVG; j++) acc += analogReadMilliVolts(FLEX_PINS[k]);
      mv[k] = acc / FLEX_AVG;
    }
    char fl[64];
    snprintf(fl, sizeof(fl), "F,%lu,%lu,%lu,%lu", millis(), mv[0], mv[1], mv[2]);
    printChecked(fl);
  }
  if (!imuOk) {                               // keep retrying so hot-plugging the IMU works
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 2000) { lastTry = millis(); findImu(); }
    return;
  }
  now = micros();
  if (streaming && now - lastUs >= periodUs) {
    lastUs = now;
    sensors_event_t a, g, t, m;
    if (!icm.getEvent(&a, &g, &t, &m)) { Serial.println("ERR imu read failed"); imuOk = false; return; }
    char line[160];
    snprintf(line, sizeof(line), "I,%lu,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%.1f", millis(),
             a.acceleration.x, a.acceleration.y, a.acceleration.z,
             g.gyro.x, g.gyro.y, g.gyro.z,
             m.magnetic.x, m.magnetic.y, m.magnetic.z, t.temperature);
    printChecked(line);
  }
}
