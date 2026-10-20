/*
  arm_controller.ino - low-level servo executor for the 5-DOF sorter arm
  ======================================================================
  Board : Arduino Nano (ATmega328P) or ESP32 dev board
  Driver: PCA9685 16-channel PWM board (Adafruit or clone), I2C address 0x40
  Lib   : "Adafruit PWM Servo Driver Library" (Library Manager)

  Wiring
    PCA9685 VCC -> 5V of the board (logic only)    PCA9685 GND -> GND
    PCA9685 SDA -> Nano A4  / ESP32 GPIO21         PCA9685 SCL -> Nano A5 / ESP32 GPIO22
    PCA9685 V+ (green screw terminal) -> separate 5-6 V servo supply, >= 5 A
    Servo supply GND must be joined to the board's GND.
    Channels 0..5 = base, shoulder, elbow, wrist pitch, wrist roll, gripper.

  The PC does all vision and inverse kinematics. This sketch only:
    1. parses packets   <M,seq,a1,a2,a3,a4,a5,g,ms*CS>   (see arm_link.py on the PC)
    2. clamps every angle to the limits below
    3. moves all servos together along a minimum-jerk curve, so they start and
       stop gently and arrive at the same moment (no violent snapping)
    4. reports <ACK,seq,ms> when accepted and <DONE,seq> when the move ends

  Every angle is in SERVO degrees (0..SERVO_RANGE_DEG). The PC converts its
  kinematic angles with the zero/direction table in config.py.

  For manual testing in the Serial Monitor (115200 baud, "Newline"), set
  REQUIRE_CHECKSUM to 0 and type e.g.  <M,1,90,90,90,90,90,60,1500>
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============================================================================
// USER CONFIG
// ============================================================================
#define BAUD                115200
#define NUM_SERVOS          6       // base, shoulder, elbow, wrist pitch, wrist roll, gripper
#define GRIPPER_INDEX       5
#define GRIPPER_IS_MAGNET   0       // 1 = channel of index 5 drives a MOSFET for an electromagnet
#define REQUIRE_CHECKSUM    0       // 1 = reject packets without *CS (the PC always sends one)
#define HOST_TIMEOUT_MS     5000UL  // magnet mode: release if the PC goes silent this long
#define UPDATE_INTERVAL_MS  10      // servo update period while moving
#define MIN_MOVE_MS         150
#define PCA9685_ADDR        0x40
#define SERVO_FREQ_HZ       50
#define PCA_OSC_HZ          27000000UL   // trim 25-27 MHz if pulses measure off on a scope

const uint8_t  CHANNEL[NUM_SERVOS]         = {0, 1, 2, 3, 4, 5};
// Pulse widths at 0 deg and at SERVO_RANGE_DEG. MG996R / SG90 are usually ~500-2500 us.
// Start narrower (600-2400) and widen only if the servo does not reach its end stops.
const uint16_t PULSE_MIN_US[NUM_SERVOS]    = {500, 500, 500, 500, 500, 500};
const uint16_t PULSE_MAX_US[NUM_SERVOS]    = {2500, 2500, 2500, 2500, 2500, 2500};
const float    SERVO_RANGE_DEG[NUM_SERVOS] = {180, 180, 180, 180, 180, 180};
// Hard limits in servo degrees: keep in sync with JOINTS min/max in config.py.
const float    LIMIT_MIN[NUM_SERVOS]       = {0,   15,  0,   0,   0,   20};
const float    LIMIT_MAX[NUM_SERVOS]       = {180, 165, 180, 180, 180, 120};
// Pose written at power-up (keep in sync with HOME_SERVO in config.py).
const float    HOME_POSE[NUM_SERVOS]       = {90,  90,  90,  90,  90,  60};
// Speed caps in deg/s. Big joints carry the most load: keep them slow.
const float    MAX_SPEED_DPS[NUM_SERVOS]   = {90,  60,  75,  120, 180, 240};

// ============================================================================
// STATE
// ============================================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

float startPos[NUM_SERVOS];
float targetPos[NUM_SERVOS];
float curPos[NUM_SERVOS];
float written[NUM_SERVOS];           // last value sent to the PCA9685 (-1 = output off)

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
void writeServo(uint8_t i, float deg) {
  if (GRIPPER_IS_MAGNET && i == GRIPPER_INDEX) {
    bool on = deg > 0.5f;
    if (on) pwm.setPWM(CHANNEL[i], 4096, 0);      // fully on
    else    pwm.setPWM(CHANNEL[i], 0, 4096);      // fully off
    written[i] = on ? 1 : 0;
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
    pwm.setPWM(CHANNEL[i], 0, 4096);
    written[i] = -1;
  }
}

// ============================================================================
// REPLIES  (no checksum on replies; the PC accepts plain frames)
// ============================================================================
void reply2(const char* kind, long seq) {
  Serial.print('<'); Serial.print(kind); Serial.print(','); Serial.print(seq); Serial.println('>');
}

void replyAck(long seq, long durMs) {
  Serial.print(F("<ACK,")); Serial.print(seq);
  if (durMs >= 0) { Serial.print(','); Serial.print(durMs); }
  Serial.println('>');
}

void replyErr(long seq, const __FlashStringHelper* code) {
  Serial.print(F("<ERR,")); Serial.print(seq); Serial.print(','); Serial.print(code); Serial.println('>');
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
    if (GRIPPER_IS_MAGNET && i == GRIPPER_INDEX) continue;
    float dist = fabs(target[i] - curPos[i]);
    unsigned long need = (unsigned long)(1875.0f * dist / MAX_SPEED_DPS[i]);   // ms
    if (need > dur) dur = need;
  }
  return dur;
}

void finishActive(bool preempted) {
  if (activeSeq < 0) return;
  if (preempted) {
    Serial.print(F("<DONE,")); Serial.print(activeSeq); Serial.println(F(",PREEMPTED>"));
  } else {
    reply2("DONE", activeSeq);
  }
  activeSeq = -1;
  lastDone = true;
}

void startMove(const float* target, unsigned long requestedMs, long seq) {
  if (moving) finishActive(true);                     // a new move replaces the old one
  if (relaxed) {                                      // outputs were off: we no longer know the
    relaxed = false;                                  // real position, so restart from target
    for (uint8_t i = 0; i < NUM_SERVOS; i++) curPos[i] = target[i];
  }
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    startPos[i] = curPos[i];
    targetPos[i] = target[i];
  }
  if (GRIPPER_IS_MAGNET) {                            // magnet switches at the start of the move
    curPos[GRIPPER_INDEX] = startPos[GRIPPER_INDEX] = target[GRIPPER_INDEX];
    writeServo(GRIPPER_INDEX, target[GRIPPER_INDEX]);
  }
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
    if (GRIPPER_IS_MAGNET && i == GRIPPER_INDEX) continue;
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
    case 'P':                                     // the PC pings once per (re)connect, and its
      lastSeq = -1;                               // seq counter restarts, so forget the old seq
      reply2("PONG", seq);
      break;

    case 'M': {                                   // M,seq,a1..a6,ms
      if (n != 2 + NUM_SERVOS + 1) { replyErr(seq, F("FORMAT")); return; }
      float target[NUM_SERVOS];
      bool clamped = false;
      for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        float v;
        if (!parseNum(tok[2 + i], v)) { replyErr(seq, F("NUMBER")); return; }
        target[i] = clampDeg(i, v, clamped);
      }
      long ms;
      if (!parseLong(tok[2 + NUM_SERVOS], ms) || ms < 0 || ms > 60000L) { replyErr(seq, F("DURATION")); return; }
      if (clamped) { Serial.print(F("<WARN,CLAMP,")); Serial.print(seq); Serial.println('>'); }
      startMove(target, (unsigned long)ms, seq);
      break;
    }

    case 'H': {                                   // H,seq[,ms]
      long ms = 1500;
      if (n >= 3 && !parseLong(tok[2], ms)) { replyErr(seq, F("DURATION")); return; }
      startMove(HOME_POSE, (unsigned long)ms, seq);
      break;
    }

    case 'S':                                     // stop and hold here
      if (moving) {
        for (uint8_t i = 0; i < NUM_SERVOS; i++) targetPos[i] = curPos[i];
        moving = false;
        finishActive(true);
      }
      reply2("ACK", seq);
      break;

    case 'R':                                     // relax: PWM off, arm goes limp
      if (moving) { moving = false; finishActive(true); }
      outputsOff();
      relaxed = true;
      reply2("ACK", seq);
      break;

    case 'Q':                                     // POS,seq,a1..a6,moving
      Serial.print(F("<POS,")); Serial.print(seq);
      for (uint8_t i = 0; i < NUM_SERVOS; i++) { Serial.print(','); Serial.print(curPos[i], 1); }
      Serial.print(','); Serial.print(moving ? 1 : 0); Serial.println('>');
      break;

    default:
      replyErr(seq, F("UNKNOWN"));
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
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
// SAFETY
// ============================================================================
void checkHostTimeout() {
  if (!GRIPPER_IS_MAGNET) return;          // a servo gripper simply keeps holding
  if (written[GRIPPER_INDEX] > 0.5f && millis() - lastHostMs > HOST_TIMEOUT_MS) {
    curPos[GRIPPER_INDEX] = targetPos[GRIPPER_INDEX] = 0;
    writeServo(GRIPPER_INDEX, 0);
    Serial.println(F("<WARN,HOST_TIMEOUT,MAGNET_OFF>"));
  }
}

// ============================================================================
void setup() {
  Serial.begin(BAUD);
  Wire.begin();                            // Nano: A4/A5, ESP32: GPIO21/22
  Wire.setClock(400000);
  pwm.begin();
  pwm.setOscillatorFrequency(PCA_OSC_HZ);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);

  // Servos jump to HOME on power-up because their real position is unknown.
  // Park the arm near HOME by hand before switching the servo supply on.
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    curPos[i] = startPos[i] = targetPos[i] = HOME_POSE[i];
    writeServo(i, HOME_POSE[i]);
  }
  lastHostMs = millis();
  Serial.println(F("<READY,ARM5,1>"));
}

void loop() {
  readSerial();
  updateMotion();
  checkHostTimeout();
}
