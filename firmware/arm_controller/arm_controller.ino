/*
  arm_controller.ino - ESP32 low-level executor for the 3-DOF electromagnet sorter arm
  ==============================================================================
  Board : ESP32 dev board (Wi-Fi TCP is the primary host connection)
  Driver: PCA9685 16-channel PWM board (Adafruit or clone), I2C address 0x40
  Lib   : "Adafruit PWM Servo Driver Library" (Library Manager)
  Tool  : KK-P20/15 12 V lifting electromagnet (~0.2 A), switched by a MOSFET

  Wiring
    PCA9685 VCC -> 5V of the board (logic only)    PCA9685 GND -> GND
    PCA9685 SDA -> Nano A4  / ESP32 GPIO21         PCA9685 SCL -> Nano A5 / ESP32 GPIO22
    PCA9685 V+ (green screw terminal) -> separate 5-6 V servo supply, >= 5 A
    Channels 0..4 = base, shoulder, elbow, wrist pitch, wrist roll.

    Electromagnet (12 V supply, NOT the servo supply):
      +12V -----------+---------------- magnet lead A
                      |
                 1N4007 / 1N5819 (stripe towards +12V)
                      |
      MOSFET drain ---+---------------- magnet lead B
      MOSFET source -> GND (12 V GND, servo GND and board GND all joined)
      MOSFET gate  <- 220 ohm <- MAGNET_PIN, plus 10 k from gate to GND
      Logic-level MOSFET: IRLZ44N, AO3400 or a ready-made "MOSFET switch module".
      Never drive the coil straight from a pin, never leave out the diode.

  The PC does all vision and inverse kinematics. This sketch only:
    1. parses packets   <M,seq,a1,a2,a3,a4,a5,mag,ms*CS>   (see host/arm_link.py)
    2. clamps every angle to the limits below
    3. moves all servos together along a minimum-jerk curve, so they start and
       stop gently and arrive at the same moment (no violent snapping)
    4. switches the magnet at the START of a move (mag: 1 = on, 0 = off)
    5. reports <ACK,seq,ms> when accepted and <DONE,seq> when the move ends
    6. drops the magnet if the PC goes silent or it stays on too long

  Every angle is in SERVO degrees (0..SERVO_RANGE_DEG). The PC converts its
  kinematic angles with the zero/direction table in host/config.py.

  For manual testing in the Serial Monitor (115200 baud, "Newline"), keep
  REQUIRE_CHECKSUM at 0 and type e.g.  <M,1,90,90,90,90,90,0,1500>
*/

#if !defined(ESP32)
#error "This sketch is ESP32-only; select an ESP32 board in the Arduino IDE."
#endif
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============================================================================
// USER CONFIG
// ============================================================================
#define BAUD                115200
// WIFI_PATCH_APPLIED_V1
// Leave WIFI_SSID empty to start a direct-access-point network (no router needed).
// For router/phone-hotspot mode, fill WIFI_SSID and WIFI_PASSWORD, then use the
// ESP32 IP printed on Serial as host/config.py SERIAL_PORT.
#define WIFI_SSID ""
#define WIFI_PASSWORD         ""
#define WIFI_AP_SSID          "RobotArm-ESP32"
#define WIFI_AP_PASSWORD      "ArmSort_26!WiFi"  // change to your own strong password
#define TCP_PORT              8888
#define WIFI_CONNECT_TIMEOUT_MS 12000UL
#define NUM_SERVOS          5       // protocol compatibility: base, shoulder, elbow, two inactive slots
#define ACTIVE_SERVOS       3       // only base, shoulder and elbow receive PWM
#define NUM_VALUES          6       // 5 angles + magnet state in every move packet
#define MAGNET_PIN        18      // ESP32 GPIO18 (D18, left pin row): MOSFET module TRIG
#define MAGNET_ACTIVE_HIGH  1       // 0 if your switch module turns on with a LOW input
#define MAGNET_AUTO_OFF     0       // 1 = the two cut-offs below switch the coil off; 0 = only the PC does
#define MAGNET_MAX_ON_MS    30000UL // coil off after this long on (only with MAGNET_AUTO_OFF 1)
#define HOST_TIMEOUT_MS     5000UL  // safety: coil off if the PC goes silent this long
#define REQUIRE_CHECKSUM    0       // 1 = reject packets without *CS (the PC always sends one)
#define UPDATE_INTERVAL_MS  10      // servo update period while moving
#define MIN_MOVE_MS         150
#define START_STAGGER_MS    700     // gap between waking each servo at power-up
#define PCA9685_ADDR        0x40
#define SERVO_FREQ_HZ       50
#define PCA_OSC_HZ          27000000UL   // trim 25-27 MHz if pulses measure off on a scope

const uint8_t  CHANNEL[NUM_SERVOS]         = {15, 14, 13, 12, 11};
// Pulse widths at 0 deg and at SERVO_RANGE_DEG. Full MG996R range: 500-2500 us is
// ~180 deg, so a command of N servo degrees is roughly N real degrees. (The earlier
// 1000-2000 us bench range only gave about half the travel.)
const uint16_t PULSE_MIN_US[NUM_SERVOS]    = {500, 500, 500, 500, 500};
const uint16_t PULSE_MAX_US[NUM_SERVOS]    = {2500, 2500, 2500, 2500, 2500};
const float    SERVO_RANGE_DEG[NUM_SERVOS] = {180, 180, 180, 180, 180};
// Hard limits in servo degrees: keep in sync with JOINTS min/max in host/config.py.
const float    LIMIT_MIN[NUM_SERVOS]       = {5, 10, 10, 10, 10};
const float    LIMIT_MAX[NUM_SERVOS]       = {175, 175, 175, 160, 160};
// Pose written at power-up (keep in sync with HOME_SERVO in host/config.py).
const float    HOME_POSE[NUM_SERVOS]       = {90, 170,  90,  90,  90};   // upright: weight over the base
// Speed caps in deg/s. Big joints carry the most load: keep them slow.
const float    MAX_SPEED_DPS[NUM_SERVOS]   = {45,  35,  35,  120, 180};

// ============================================================================
// STATE
// ============================================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
bool tcpSession = false;
bool wifiApMode = false;

// Replies follow the active TCP session. Serial remains available only as a local
// ESP32 bench/debug path; normal host operation defaults to Wi-Fi.
Stream& protocolOut() {
  if (tcpSession && tcpClient && tcpClient.connected()) return tcpClient;
  return Serial;
}

float startPos[NUM_SERVOS];
float targetPos[NUM_SERVOS];
float curPos[NUM_SERVOS];
float written[NUM_SERVOS];           // last value sent to the PCA9685 (-1 = output off)

bool          magnetOn     = false;
unsigned long magnetOnMs   = 0;
bool          moving       = false;
unsigned long moveStartMs  = 0;
unsigned long moveDurMs    = 0;
unsigned long lastUpdateMs = 0;
unsigned long lastHostMs   = 0;

long activeSeq = -1;                 // move whose DONE is still owed
long lastSeq   = -1;                 // last accepted seq, to spot re-sent packets
bool lastDone  = true;
bool relaxed   = false;

char    rxBuf[96];
uint8_t rxLen   = 0;
bool    inFrame = false;

// ============================================================================
// OUTPUT
// ============================================================================
void setMagnet(bool on) {
  digitalWrite(MAGNET_PIN, (on == (bool)MAGNET_ACTIVE_HIGH) ? HIGH : LOW);
  if (on && !magnetOn) magnetOnMs = millis();
  magnetOn = on;
}

void writeServo(uint8_t i, float deg) {
  // Slots 3 and 4 remain in the packet for protocol compatibility, but this
  // 3-DOF arm has no active wrist pitch/roll; both PCA9685 outputs stay disabled.
  // The magnet is rigidly mounted and its pitch follows shoulder + elbow.
  if (i >= ACTIVE_SERVOS) {
    pwm.setPWM(CHANNEL[i], 0, 4096);  // fully off
    written[i] = deg;
    return;
  }
  float frac = deg / SERVO_RANGE_DEG[i];
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  float us = PULSE_MIN_US[i] + frac * (PULSE_MAX_US[i] - PULSE_MIN_US[i]);
  // PCA9685: 4096 ticks per PWM period (20 ms at 50 Hz)
  uint16_t ticks = (uint16_t)(us * SERVO_FREQ_HZ * 4096.0f / 1000000.0f + 0.5f);
  pwm.setPWM(CHANNEL[i], 0, ticks);
  written[i] = deg;
}

void outputsOff() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    pwm.setPWM(CHANNEL[i], 0, 4096);              // channel fully off
    written[i] = -1;
  }
}

// ============================================================================
// REPLIES  (no checksum on replies; the PC accepts plain frames)
// ============================================================================
void reply2(const char* kind, long seq) {
  Stream& out = protocolOut();
  out.print('<'); out.print(kind); out.print(','); out.print(seq); out.println('>');
}

void replyAck(long seq, long durMs) {
  Stream& out = protocolOut();
  out.print(F("<ACK,")); out.print(seq);
  if (durMs >= 0) { out.print(','); out.print(durMs); }
  out.println('>');
}

void replyErr(long seq, const __FlashStringHelper* code) {
  Stream& out = protocolOut();
  out.print(F("<ERR,")); out.print(seq); out.print(','); out.print(code); out.println('>');
}

// ============================================================================
// MOTION
// ============================================================================
float clampDeg(uint8_t i, float d, bool &clamped) {
  if (d < LIMIT_MIN[i]) { clamped = true; return LIMIT_MIN[i]; }
  if (d > LIMIT_MAX[i]) { clamped = true; return LIMIT_MAX[i]; }
  return d;
}

// Minimum-jerk profile peaks at 1.875x the average speed, so the shortest
// allowed duration for a joint is 1.875 * distance / max_speed.
unsigned long planDuration(const float* target, unsigned long requestedMs) {
  unsigned long dur = requestedMs < MIN_MOVE_MS ? MIN_MOVE_MS : requestedMs;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    float dist = fabs(target[i] - curPos[i]);
    unsigned long need = (unsigned long)(1875.0f * dist / MAX_SPEED_DPS[i]);   // ms
    if (need > dur) dur = need;
  }
  return dur;
}

void finishActive(bool preempted) {
  if (activeSeq < 0) return;
  if (preempted) {
    Stream& out = protocolOut();
    out.print(F("<DONE,")); out.print(activeSeq); out.println(F(",PREEMPTED>"));
  } else {
    reply2("DONE", activeSeq);
  }
  activeSeq = -1;
  lastDone = true;
}

void startMove(const float* target, bool magnet, unsigned long requestedMs, long seq) {
  if (moving) finishActive(true);                     // a new move replaces the old one
  if (relaxed) {                                      // outputs were off: we no longer know the
    relaxed = false;                                  // real position, so restart from target
    for (uint8_t i = 0; i < NUM_SERVOS; i++) curPos[i] = target[i];
  }
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    startPos[i] = curPos[i];
    targetPos[i] = target[i];
  }
  setMagnet(magnet);                                  // magnet switches as the move starts
  moveDurMs = planDuration(target, requestedMs);
  moveStartMs = millis();
  lastUpdateMs = 0;
  moving = true;
  activeSeq = seq;
  lastSeq = seq;
  lastDone = false;
  replyAck(seq, (long)moveDurMs);
}

void updateMotion() {
  if (!moving) return;
  unsigned long now = millis();
  if (lastUpdateMs != 0 && now - lastUpdateMs < UPDATE_INTERVAL_MS) return;
  lastUpdateMs = now;

  float s = moveDurMs ? (float)(now - moveStartMs) / (float)moveDurMs : 1.0f;
  if (s > 1.0f) s = 1.0f;
  float e = s * s * s * (10.0f + s * (-15.0f + 6.0f * s));     // minimum-jerk 0 -> 1

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    curPos[i] = startPos[i] + (targetPos[i] - startPos[i]) * e;
    if (fabs(curPos[i] - written[i]) >= 0.05f || s >= 1.0f) writeServo(i, curPos[i]);
  }
  if (s >= 1.0f) {
    moving = false;
    finishActive(false);
  }
}

// ============================================================================
// PARSING
// ============================================================================
bool parseNum(const char* s, float &out) {
  if (s == NULL || *s == '\0') return false;
  char* end;
  double v = strtod(s, &end);
  if (end == s || *end != '\0') return false;
  out = (float)v;
  return true;
}

bool parseLong(const char* s, long &out) {
  if (s == NULL || *s == '\0') return false;
  char* end;
  long v = strtol(s, &end, 10);
  if (end == s || *end != '\0') return false;
  out = v;
  return true;
}

uint8_t hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 255;
}

void handleFrame(char* frame) {
  lastHostMs = millis();

  // ---- checksum -----------------------------------------------------------
  char* star = strchr(frame, '*');
  if (star != NULL) {
    uint8_t cs = 0;
    for (char* p = frame; p < star; p++) cs ^= (uint8_t)*p;
    uint8_t hi = hexVal(star[1]), lo = (star[1] ? hexVal(star[2]) : 255);
    if (hi == 255 || lo == 255 || ((hi << 4) | lo) != cs) { replyErr(-1, F("CHECKSUM")); return; }
    *star = '\0';
  } else if (REQUIRE_CHECKSUM) {
    replyErr(-1, F("NOCHECKSUM"));
    return;
  }

  // ---- split into fields --------------------------------------------------
  char* tok[12];
  uint8_t n = 0;
  char* t = strtok(frame, ",");
  while (t != NULL && n < 12) { tok[n++] = t; t = strtok(NULL, ","); }
  if (n < 2) { replyErr(-1, F("FORMAT")); return; }
  char cmd = tok[0][0];
  long seq;
  if (!parseLong(tok[1], seq)) { replyErr(-1, F("SEQ")); return; }

  // ---- a packet we already accepted (the PC missed our ACK and re-sent it) --
  if ((cmd == 'M' || cmd == 'H') && seq == lastSeq) {
    replyAck(seq, -1);
    if (lastDone) reply2("DONE", seq);
    return;
  }

  switch (cmd) {
    case 'P':                                     // connection setup resets sequence state
      reply2("PONG", seq);
      break;

    case 'M': {                                   // M,seq,a1..a5,mag,ms
      if (n != 2 + NUM_VALUES + 1) { replyErr(seq, F("FORMAT")); return; }
      float target[NUM_SERVOS];
      bool clamped = false;
      for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        float v;
        if (!parseNum(tok[2 + i], v)) { replyErr(seq, F("NUMBER")); return; }
        target[i] = clampDeg(i, v, clamped);
      }
      float mag;
      if (!parseNum(tok[2 + NUM_SERVOS], mag)) { replyErr(seq, F("NUMBER")); return; }
      long ms;
      if (!parseLong(tok[2 + NUM_VALUES], ms) || ms < 0 || ms > 60000L) { replyErr(seq, F("DURATION")); return; }
      if (clamped) { Stream& out = protocolOut(); out.print(F("<WARN,CLAMP,")); out.print(seq); out.println('>'); }
      startMove(target, mag > 0.5f, (unsigned long)ms, seq);
      break;
    }

    case 'H': {                                   // H,seq[,ms]  home, magnet off
      long ms = 1500;
      if (n >= 3 && !parseLong(tok[2], ms)) { replyErr(seq, F("DURATION")); return; }
      startMove(HOME_POSE, false, (unsigned long)ms, seq);
      break;
    }

    case 'S':                                     // stop and hold here (magnet unchanged)
      if (moving) {
        for (uint8_t i = 0; i < NUM_SERVOS; i++) targetPos[i] = curPos[i];
        moving = false;
        finishActive(true);
      }
      reply2("ACK", seq);
      break;

    case 'R':                                     // relax: PWM off, magnet off, arm goes limp
      if (moving) { moving = false; finishActive(true); }
      outputsOff();
      setMagnet(false);
      relaxed = true;
      reply2("ACK", seq);
      break;

    case 'Q':                                     // POS,seq,a1..a5,mag,moving
      { Stream& out = protocolOut();
        out.print(F("<POS,")); out.print(seq);
        for (uint8_t i = 0; i < NUM_SERVOS; i++) { out.print(','); out.print(curPos[i], 1); }
        out.print(','); out.print(magnetOn ? 1 : 0);
        out.print(','); out.print(moving ? 1 : 0); out.println('>');
      }
      break;

    default:
      replyErr(seq, F("UNKNOWN"));
  }
}

void readActiveInput() {
  Stream& input = (tcpSession && tcpClient && tcpClient.connected())
                    ? (Stream&)tcpClient : (Stream&)Serial;
  while (input.available() > 0) {
    char c = (char)input.read();
    if (c == '<') { inFrame = true; rxLen = 0; continue; }   // '<' always starts a new frame
    if (!inFrame) continue;                                   // ignore noise between frames
    if (c == '>') {
      rxBuf[rxLen] = '\0';
      inFrame = false;
      handleFrame(rxBuf);
      continue;
    }
    if (c == '\r' || c == '\n') continue;
    if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
    else { inFrame = false; replyErr(-1, F("OVERFLOW")); }
  }
}

// ============================================================================
// WI-FI NETWORK
// ============================================================================
void startWiFiNetwork() {
  WiFi.persistent(false);
  WiFi.setSleep(false);

  bool connectedToRouter = false;
  if (strlen(WIFI_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print(F("Connecting to configured Wi-Fi"));
    unsigned long started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < WIFI_CONNECT_TIMEOUT_MS) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    connectedToRouter = (WiFi.status() == WL_CONNECTED);
    if (connectedToRouter) {
      wifiApMode = false;
      Serial.print(F("ESP32 Wi-Fi IP: ")); Serial.println(WiFi.localIP());
    }
  }

  if (!connectedToRouter) {
    WiFi.mode(WIFI_AP);
    wifiApMode = true;
    if (WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD)) {
      Serial.print(F("ESP32 access point: ")); Serial.println(WIFI_AP_SSID);
      Serial.print(F("Connect PC to this Wi-Fi; TCP controller IP: "));
      Serial.println(WiFi.softAPIP());
    } else {
      Serial.println(F("ERROR: ESP32 Wi-Fi AP start failed; check board/core selection."));
    }
  }

  tcpServer.begin();
  Serial.print(F("TCP arm server listening on port "));
  Serial.println(TCP_PORT);
  Serial.println(F("Protocol READY is sent after the host TCP client connects."));
}

void handleTcpDisconnect() {
  // Stop an in-progress trajectory at the latest commanded servo state, and fail
  // safe on the magnet. The servos remain powered/holding unless hardware is reset.
  setMagnet(false);
  if (moving) {
    moving = false;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      startPos[i] = curPos[i];
      targetPos[i] = curPos[i];
    }
    finishActive(true);
  }
  tcpClient.stop();
  tcpSession = false;
  rxLen = 0;
  inFrame = false;
  lastHostMs = millis();
  Serial.println(F("TCP client disconnected: magnet OFF; motion stopped."));
}

void serviceWiFiClient() {
  if (tcpSession && (!tcpClient || !tcpClient.connected())) {
    handleTcpDisconnect();
  }
  if (tcpSession) return;  // one controller at a time

  WiFiClient incoming = tcpServer.available();
  if (!incoming) return;

  tcpClient = incoming;
  tcpClient.setNoDelay(true);
  tcpSession = true;
  rxLen = 0;
  inFrame = false;
  lastHostMs = millis();
  // A fresh TCP client starts a fresh command-sequence session.
  lastSeq = -1;
  lastDone = true;
  activeSeq = -1;
  tcpClient.print(F("<READY,ARM5,1>\r\n"));
  Serial.print(F("TCP controller connected from "));
  Serial.println(tcpClient.remoteIP());
}

// ============================================================================
// SAFETY
// ============================================================================
void checkMagnetSafety() {
  unsigned long now = millis();
  // If a command stops arriving during an active move, halt the trajectory and
  // turn the magnet off. Normal sorter moves are shorter than HOST_TIMEOUT_MS.
  if (((MAGNET_AUTO_OFF && magnetOn) || (tcpSession && moving)) && now - lastHostMs > HOST_TIMEOUT_MS) {
    if (MAGNET_AUTO_OFF) setMagnet(false);
    if (moving) {
      moving = false;
      for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        startPos[i] = curPos[i];
        targetPos[i] = curPos[i];
      }
      finishActive(true);
    }
    { Stream& out = protocolOut(); out.println(F("<WARN,HOST_TIMEOUT,MAGNET_OFF>")); }
    if (tcpSession) {
      tcpClient.stop();
      tcpSession = false;
      rxLen = 0;
      inFrame = false;
    }
  }
  if (MAGNET_AUTO_OFF && magnetOn && now - magnetOnMs > MAGNET_MAX_ON_MS) {
    setMagnet(false);
    Stream& out = protocolOut();
    out.println(F("<WARN,MAGNET_MAX_ON,MAGNET_OFF>"));
  }
}

// ============================================================================
void setup() {
  pinMode(MAGNET_PIN, OUTPUT);
  setMagnet(false);                        // coil off before anything else happens
  Serial.begin(BAUD);
  Wire.begin();                            // Nano: A4/A5, ESP32: GPIO21/22
  Wire.setClock(400000);
  pwm.begin();
  pwm.setOscillatorFrequency(PCA_OSC_HZ);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);

  // Servos jump to HOME on power-up because their real position is unknown.
  // To keep that jump gentle: switch the SERVO supply on first (servos stay limp
  // with no signal), hold the arm near HOME, then reset the ESP32 (EN button).
  // The servos then wake one at a time instead of all slamming together.
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    curPos[i] = startPos[i] = targetPos[i] = HOME_POSE[i];
    pwm.setPWM(CHANNEL[i], 0, 4096);       // all outputs off (limp) to start
    written[i] = -1;
  }
  const uint8_t WAKE_ORDER[NUM_SERVOS] = {0, 2, 1, 3, 4};   // base, elbow, shoulder, (wrist slots)
  for (uint8_t k = 0; k < NUM_SERVOS; k++) {
    writeServo(WAKE_ORDER[k], HOME_POSE[WAKE_ORDER[k]]);
    if (WAKE_ORDER[k] < ACTIVE_SERVOS) delay(START_STAGGER_MS);
  }
  lastHostMs = millis();
  startWiFiNetwork();
}

void loop() {
  serviceWiFiClient();
  readActiveInput();
  updateMotion();
  checkMagnetSafety();
}
