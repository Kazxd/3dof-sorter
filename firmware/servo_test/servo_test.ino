/*
  servo_test.ino - first power-up test for the sorter arm, no PC software needed
  ==============================================================================
  Flash this BEFORE arm_controller.ino on a newly built arm. It drives each servo on
  its own through the PCA9685, slowly, so you can check wiring, direction and range
  and find each servo's limits before the real firmware is used.

  Same wiring as arm_controller.ino (PCA9685 on I2C, servos on channels 15-11,
  magnet MOSFET on D7 / ESP32 GPIO25).

  Open the Serial Monitor at 115200 baud, line ending "Newline", and type:
    0..4        select a servo (0 base, 1 shoulder, 2 elbow, 3 wrist pitch, 4 wrist roll)
    + / -       move the selected servo by the step size
    s 10        set the step size (degrees)          a 120   go to an absolute angle
    w           sweep the selected servo slowly: centre -> +range -> -range -> centre
    r 30        set the sweep range (degrees either side of centre)
    h           all servos to 90 (centre), slowly
    m           pulse the magnet on for 1 s
    p           print every servo's current angle
    o           all PWM outputs off (arm goes limp, magnet off)

  Write down for each servo: the angle where the link is at its zero, which way a
  bigger angle moves it, and the safe min / max. Those go into host/config.py
  (JOINTS) and the LIMIT_MIN / LIMIT_MAX arrays of arm_controller.ino.

  Start with the horns OFF the servos: power up, send 'h', then fit the horns so
  the arm stands straight up. Then test one joint at a time with small ranges.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define BAUD           115200
#define NUM_SERVOS     5
#if defined(ESP32)
  #define MAGNET_PIN   25
#else
  #define MAGNET_PIN   7
#endif
#define SERVO_FREQ_HZ  50
#define PCA_OSC_HZ     27000000UL
#define STEP_DELAY_MS  15          // ms per degree: ~65 deg/s, gentle

const uint8_t  CHANNEL[NUM_SERVOS]      = {15, 14, 13, 12, 11};   // as wired on our arm
const uint16_t PULSE_MIN_US[NUM_SERVOS] = {1000, 1000, 1000, 1000, 1000};   // bench-tested range
const uint16_t PULSE_MAX_US[NUM_SERVOS] = {2000, 2000, 2000, 2000, 2000};
const char*    NAMES[NUM_SERVOS]        = {"base", "shoulder", "elbow", "wrist pitch", "wrist roll"};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
float   angle[NUM_SERVOS];
bool    active[NUM_SERVOS];        // false until a servo is first commanded (stays limp)
uint8_t sel = 0;
float   stepDeg = 5;
float   rangeDeg = 20;
char    line[32];
uint8_t len = 0;

void writeServo(uint8_t i, float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  float us = PULSE_MIN_US[i] + deg / 180.0f * (PULSE_MAX_US[i] - PULSE_MIN_US[i]);
  pwm.setPWM(CHANNEL[i], 0, (uint16_t)(us * SERVO_FREQ_HZ * 4096.0f / 1000000.0f + 0.5f));
  angle[i] = deg;
  active[i] = true;
}

// Move one servo to `target` one degree at a time.
void glide(uint8_t i, float target) {
  if (target < 0) target = 0;
  if (target > 180) target = 180;
  if (!active[i]) { writeServo(i, target); return; }   // first command: position unknown, go direct
  float a = angle[i];
  while (fabs(target - a) > 0.5f) {
    a += (target > a) ? 1.0f : -1.0f;
    writeServo(i, a);
    delay(STEP_DELAY_MS);
    if (Serial.available() && Serial.peek() == 'o') return;   // 'o' interrupts a move
  }
  writeServo(i, target);
}

void printState() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(i == sel ? F("> ") : F("  "));
    Serial.print(i); Serial.print(' '); Serial.print(NAMES[i]); Serial.print(F(": "));
    if (active[i]) Serial.println(angle[i], 0); else Serial.println(F("off"));
  }
  Serial.print(F("  step ")); Serial.print(stepDeg, 0);
  Serial.print(F("  sweep range +/-")); Serial.println(rangeDeg, 0);
}

void handle(char* cmd) {
  char c = cmd[0];
  float v = atof(cmd + 1);
  if (c >= '0' && c <= '4') {
    sel = c - '0';
    Serial.print(F("selected ")); Serial.println(NAMES[sel]);
  } else if (c == '+' || c == '-') {
    float from = active[sel] ? angle[sel] : 90;
    glide(sel, from + (c == '+' ? stepDeg : -stepDeg));
    Serial.print(NAMES[sel]); Serial.print(F(" = ")); Serial.println(angle[sel], 0);
  } else if (c == 's' && v > 0) {
    stepDeg = v;
  } else if (c == 'r' && v > 0) {
    rangeDeg = v;
  } else if (c == 'a') {
    glide(sel, v);
    Serial.print(NAMES[sel]); Serial.print(F(" = ")); Serial.println(angle[sel], 0);
  } else if (c == 'w') {
    float centre = active[sel] ? angle[sel] : 90;
    Serial.print(F("sweeping ")); Serial.println(NAMES[sel]);
    glide(sel, centre);
    glide(sel, centre + rangeDeg); delay(300);
    glide(sel, centre - rangeDeg); delay(300);
    glide(sel, centre);
    Serial.println(F("sweep done - did a BIGGER angle move it the way you expected?"));
  } else if (c == 'h') {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) glide(i, 90);
    Serial.println(F("all at 90"));
  } else if (c == 'm') {
    digitalWrite(MAGNET_PIN, HIGH); delay(1000); digitalWrite(MAGNET_PIN, LOW);
    Serial.println(F("magnet pulsed"));
  } else if (c == 'p') {
    printState();
  } else if (c == 'o') {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) { pwm.setPWM(CHANNEL[i], 0, 4096); active[i] = false; }
    digitalWrite(MAGNET_PIN, LOW);
    Serial.println(F("outputs off"));
  } else {
    Serial.println(F("? commands: 0-4  + -  s<n>  a<n>  w  r<n>  h  m  p  o"));
  }
}

void setup() {
  pinMode(MAGNET_PIN, OUTPUT);
  digitalWrite(MAGNET_PIN, LOW);
  Serial.begin(BAUD);
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(PCA_OSC_HZ);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  for (uint8_t i = 0; i < NUM_SERVOS; i++) { pwm.setPWM(CHANNEL[i], 0, 4096); active[i] = false; }
  delay(200);
  Serial.println(F("servo_test ready. All servos are OFF until you command them."));
  Serial.println(F("Type h to centre everything, or pick a servo (0-4) and use + / - / w."));
  printState();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len) { line[len] = 0; handle(line); len = 0; }
    } else if (ch != ' ' && len < sizeof(line) - 1) {
      line[len++] = ch;
    }
  }
}
