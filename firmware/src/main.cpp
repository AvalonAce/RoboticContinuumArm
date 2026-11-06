// RCM firmware v0 — motor bring-up console.
// Text commands over USB serial (115200). Talks to STS3215s on the ST bus.
// Purpose: scan, assign IDs, zero, jog, read feedback — before any ROS work.
//
// Commands (type `help`):
//   scan                      list responding IDs (0..MAX_SCAN_ID)
//   setid <old> <new>         change ID (ONLY ONE servo on bus!)
//   zero <id>                 set current position as center (2048)
//   torque <id> <0|1>         disable / enable torque
//   pos <id> <0-4095> [spd] [acc]   move to raw position
//   sync <spd> <acc> <id> <pos> [<id> <pos>...]  simultaneous move (SyncWrite)
//   read <id>                 pos, speed, load, voltage, temp, current, moving
//   stream <id|all> <hz>      continuous CSV telemetry, `stream off` to stop
//   stop                      torque off on all scanned IDs (soft e-stop)

#include <Arduino.h>
#include <SCServo.h>

// ST bus pins on Waveshare Servo Driver w/ ESP32. If `scan` finds nothing, check schematic.
static const int S_RXD = 18;
static const int S_TXD = 19;
static const uint32_t BUS_BAUD = 1000000;  // STS3215 factory default
static const int MAX_SCAN_ID = 20;

SMS_STS st;

static int foundIds[MAX_SCAN_ID + 1];
static int nFound = 0;

static int streamId = -2;      // -2 off, -1 all, >=0 single
static uint32_t streamPeriodMs = 0;
static uint32_t lastStream = 0;

static void doScan() {
  nFound = 0;
  for (int id = 0; id <= MAX_SCAN_ID; id++) {
    if (st.Ping(id) != -1) foundIds[nFound++] = id;
  }
  Serial.printf("found %d:", nFound);
  for (int i = 0; i < nFound; i++) Serial.printf(" %d", foundIds[i]);
  Serial.println();
}

// Telemetry lines carry "*HH" = XOR of all chars before '*', so the host can drop corrupted lines
static void printChecked(const char *body) {
  uint8_t x = 0;
  for (const char *c = body; *c; c++) x ^= (uint8_t)*c;
  Serial.printf("%s*%02X\n", body, x);
}

static void printState(int id, bool csv) {
  char buf[96];
  if (st.FeedBack(id) == -1) {
    if (csv) { snprintf(buf, sizeof(buf), "%lu,%d,ERR", millis(), id); printChecked(buf); }
    else Serial.printf("id %d: no response\n", id);
    return;
  }
  int pos = st.ReadPos(-1), spd = st.ReadSpeed(-1), load = st.ReadLoad(-1);
  int volt = st.ReadVoltage(-1), temp = st.ReadTemper(-1), cur = st.ReadCurrent(-1);
  int mov = st.ReadMove(-1);
  if (csv) {
    // t_ms,id,pos,speed,load,volt_dV,temp_C,current,moving
    snprintf(buf, sizeof(buf), "%lu,%d,%d,%d,%d,%d,%d,%d,%d", millis(), id, pos, spd, load, volt, temp, cur, mov);
    printChecked(buf);
  } else {
    Serial.printf("id %d: pos=%d spd=%d load=%d volt=%.1fV temp=%dC cur=%d moving=%d\n",
                  id, pos, spd, load, volt / 10.0f, temp, cur, mov);
  }
}

static void doSetId(int oldId, int newId) {
  doScan();
  if (nFound != 1) {
    Serial.println("ERR: exactly one servo must be on the bus for setid");
    return;
  }
  if (newId < 0 || newId > 253) { Serial.println("ERR: id range 0..253"); return; }
  st.unLockEprom(oldId);
  st.writeByte(oldId, SMS_STS_ID, newId);
  st.LockEprom(newId);
  delay(50);
  Serial.printf(st.Ping(newId) != -1 ? "ok: %d -> %d\n" : "ERR: no reply at %d\n", oldId, newId);
}

static void handleLine(String line) {
  line.trim();
  if (!line.length()) return;
  char cmd[16] = {0};
  int a = -1, b = -1, c = 0, d = 0;
  int n = sscanf(line.c_str(), "%15s %d %d %d %d", cmd, &a, &b, &c, &d);
  String s(cmd);

  if (s == "help") {
    Serial.println("scan | baudscan | setid o n | zero id | torque id 0/1 | pos id p [spd] [acc] | sync spd acc id p [id p..] | read id | stream id|all hz | stream off | stop");
  } else if (s == "scan") {
    doScan();
  } else if (s == "baudscan") {
    // Try common ST-bus rates in case a servo isn't at factory 1 Mbps
    const uint32_t rates[] = {1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400};
    for (uint32_t r : rates) {
      Serial1.updateBaudRate(r); delay(20);
      int hits = 0;
      for (int id = 0; id <= MAX_SCAN_ID; id++) if (st.Ping(id) != -1) { Serial.printf("  %lu baud: id %d\n", r, id); hits++; }
      if (!hits) Serial.printf("  %lu baud: none\n", r);
    }
    Serial1.updateBaudRate(BUS_BAUD);
  } else if (s == "setid" && n >= 3) {
    doSetId(a, b);
  } else if (s == "zero" && n >= 2) {
    st.CalibrationOfs(a);
    Serial.printf("zeroed %d (current pos = 2048)\n", a);
  } else if (s == "torque" && n >= 3) {
    st.EnableTorque(a, b ? 1 : 0);
    Serial.printf("torque %d = %d\n", a, b ? 1 : 0);
  } else if (s == "pos" && n >= 3) {
    int spd = (n >= 4) ? c : 1000, acc = (n >= 5) ? d : 50;
    b = constrain(b, 0, 4095);
    st.WritePosEx(a, b, spd, acc);
    Serial.printf("move %d -> %d (spd %d acc %d)\n", a, b, spd, acc);
  } else if (s == "sync" && n >= 5) {
    // sync <spd> <acc> <id> <pos> [<id> <pos> ...]  — one SyncWrite packet, all start together
    static const int MAXN = 8;
    u8 ids[MAXN]; s16 ps[MAXN]; u16 sp[MAXN]; u8 ac[MAXN];
    char tmp[128]; line.toCharArray(tmp, sizeof(tmp));
    strtok(tmp, " ");                                   // "sync"
    int spd = atoi(strtok(NULL, " ")), acc = atoi(strtok(NULL, " "));
    int k = 0; char *ti, *tp;
    while (k < MAXN && (ti = strtok(NULL, " ")) && (tp = strtok(NULL, " "))) {
      ids[k] = atoi(ti); ps[k] = constrain(atoi(tp), 0, 4095); sp[k] = spd; ac[k] = acc; k++;
    }
    if (k == 0) { Serial.println("ERR: sync spd acc id pos [id pos ...]"); return; }
    st.SyncWritePosEx(ids, k, ps, sp, ac);
    Serial.printf("sync %d servos (spd %d acc %d)\n", k, spd, acc);
  } else if (s == "read" && n >= 2) {
    printState(a, false);
  } else if (s == "stream") {
    if (line.indexOf("off") >= 0) { streamId = -2; Serial.println("stream off"); return; }
    char target[8] = {0}; int hz = 20;
    sscanf(line.c_str(), "%*s %7s %d", target, &hz);
    streamId = (String(target) == "all") ? -1 : atoi(target);
    if (streamId == -1 && nFound == 0) doScan();
    streamPeriodMs = 1000 / max(1, min(hz, 200));
    Serial.println("t_ms,id,pos,speed,load,volt_dV,temp_C,current,moving");
  } else if (s == "stop") {
    if (nFound == 0) doScan();
    for (int i = 0; i < nFound; i++) st.EnableTorque(foundIds[i], 0);
    Serial.println("all torque off");   // telemetry keeps streaming (see the arm after e-stop)
  } else {
    Serial.printf("? [%s] (type help)\n", line.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(BUS_BAUD, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  st.IOTimeOut = 10;   // ms; lib default 100 → every absent ID stalled scans/stream by 100 ms
  delay(500);
  Serial.println("\nRCM bring-up console v0. type `help`");
  doScan();
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleLine(buf); buf = ""; }
    else if (ch >= 32 && ch < 127) buf += ch;   // drop stray control bytes
  }
  if (streamId != -2 && millis() - lastStream >= streamPeriodMs) {
    lastStream = millis();
    if (streamId == -1) for (int i = 0; i < nFound; i++) printState(foundIds[i], true);
    else printState(streamId, true);
  }
}
