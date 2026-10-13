#include <SoftwareSerial.h>
#include <Servo.h>

// v1.3: wheel trim (#w), ultrasonic stop (#u), servo sweep (#s / #x), U,<cm> telemetry.
SoftwareSerial camSerial(3, 12);  // RX=3 <- CAM GPIO14; TX=12 -> divider -> CAM GPIO15

#define Lpwm_pin 5   // Timer0 PWM (unaffected by Servo lib, which takes Timer1 / D9,D10 PWM)
#define Rpwm_pin 6
#define pinLB 2
#define pinLF 4
#define pinRB 7
#define pinRF 8

#define US_TRIG A0
#define US_ECHO A1
#define SERVO_PIN 9

unsigned char speedPwm = 70;  // crawl default until PC sends 1-9

// "#k<pwm>,<ms>\n"  e.g. "#k200,70\n"  (pwm 0 = kick off)
unsigned char kickPwm = 200;
unsigned int kickMs = 70;

// "#t<pwm>\n" max PWM during soft L/R. 0 = use full cruise speedPwm.
unsigned char turnPwmCap = 56;

// "#wL,R\n" per-wheel trim percent 50-100, scales final PWM of each wheel.
unsigned char trimL = 100;
unsigned char trimR = 100;

// "#u<cm>\n" ultrasonic forward stop distance. 0 = disabled.
unsigned int stopCm = 20;

bool leftDrive = false;
bool rightDrive = false;
bool forwardActive = false;   // last motion command was F
bool usBlocked = false;       // F refused until clear

char lineBuf[24];
uint8_t lineLen = 0;
bool inLineCmd = false;

// ---- ultrasonic ----
const unsigned long US_TIMEOUT_US = 25000UL;  // ~4 m max, bounds blocking
const unsigned long US_PERIOD_MS = 60;
const unsigned long U_REPORT_MS = 500;
unsigned int usHist[3] = {0, 0, 0};   // 0 = no echo / clear
uint8_t usIdx = 0;
unsigned long lastPing = 0, lastReport = 0;

// ---- sweep ----
Servo scanServo;
bool sweeping = false;
int sweepDeg = 0;
unsigned long sweepStepAt = 0;
const unsigned long SWEEP_SETTLE_MS = 80;

unsigned char trimmed(unsigned char pwm, unsigned char pct) {
  return (unsigned char)(((unsigned int)pwm * pct) / 100);
}

unsigned char cruiseFor(bool left, bool right) {
  bool turning = (left != right);
  unsigned char p = speedPwm;
  if (turning && turnPwmCap > 0 && p > turnPwmCap) p = turnPwmCap;
  return p;
}

void writeWheels(unsigned char lp, unsigned char rp) {
  analogWrite(Lpwm_pin, trimmed(lp, trimL));
  analogWrite(Rpwm_pin, trimmed(rp, trimR));
}

void applyDrive(bool left, bool right) {
  unsigned char cruise = cruiseFor(left, right);
  unsigned char lp = left ? cruise : 0;
  unsigned char rp = right ? cruise : 0;

  bool kickL = left && !leftDrive && cruise > 0 && kickPwm > 0 && cruise < kickPwm;
  bool kickR = right && !rightDrive && cruise > 0 && kickPwm > 0 && cruise < kickPwm;

  writeWheels(kickL ? kickPwm : lp, kickR ? kickPwm : rp);
  if (kickL || kickR) {
    delay(kickMs);
    writeWheels(lp, rp);
  }
  leftDrive = left;
  rightDrive = right;
}

void advance() {
  digitalWrite(pinRB, LOW); digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW); digitalWrite(pinLF, HIGH);
  applyDrive(true, true);
  forwardActive = true;
}

void back() {
  digitalWrite(pinRB, HIGH); digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH); digitalWrite(pinLF, LOW);
  applyDrive(true, true);
  forwardActive = false;
}

void turnR() {
  digitalWrite(pinLB, LOW); digitalWrite(pinLF, HIGH);
  digitalWrite(pinRB, HIGH); digitalWrite(pinRF, HIGH);
  applyDrive(true, false);
  forwardActive = false;
}

void turnL() {
  digitalWrite(pinRB, LOW); digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH); digitalWrite(pinLF, HIGH);
  applyDrive(false, true);
  forwardActive = false;
}

void stopp() {
  digitalWrite(pinRB, HIGH); digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH); digitalWrite(pinLF, HIGH);
  analogWrite(Lpwm_pin, 0);
  analogWrite(Rpwm_pin, 0);
  leftDrive = false;
  rightDrive = false;
  forwardActive = false;
}

// Both USB Serial and the CAM link.
void sendLine(const char *s) {
  Serial.println(s);
  camSerial.print(s);
  camSerial.print('\n');
}

// Returns cm, or 0 for no echo (timeout / no sensor). Blocks at most ~25 ms.
unsigned int pingCm() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  unsigned long us = pulseIn(US_ECHO, HIGH, US_TIMEOUT_US);
  if (us == 0) return 0;
  return (unsigned int)(us / 58UL);
}

unsigned int median3(unsigned int a, unsigned int b, unsigned int c) {
  if (a > b) { unsigned int t = a; a = b; b = t; }
  if (b > c) { unsigned int t = b; b = c; c = t; }
  if (a > b) { unsigned int t = a; a = b; b = t; }
  return b;
}

// Median of the last 3; 0 readings count as "far" so a missing sensor is always clear.
unsigned int usMedian() {
  unsigned int v[3];
  for (uint8_t i = 0; i < 3; i++) v[i] = usHist[i] ? usHist[i] : 9999;
  return median3(v[0], v[1], v[2]);
}

void serviceUltrasonic() {
  unsigned long now = millis();
  if (now - lastPing < US_PERIOD_MS) return;
  lastPing = now;
  usHist[usIdx] = pingCm();
  usIdx = (usIdx + 1) % 3;
  unsigned int m = usMedian();

  bool close = (stopCm > 0 && m < stopCm);
  if (close && forwardActive) {
    stopp();
    usBlocked = true;
    sendLine("USTOP");
  }
  if (!close && usBlocked) {
    usBlocked = false;
    sendLine("UCLEAR");
  }

  if (now - lastReport >= U_REPORT_MS) {
    lastReport = now;
    char buf[16];
    snprintf(buf, sizeof(buf), "U,%u", m >= 9999 ? 0 : m);  // U,0 = no echo
    sendLine(buf);
  }
}

void startSweep() {
  stopp();
  if (!scanServo.attached()) scanServo.attach(SERVO_PIN);
  sweeping = true;
  sweepDeg = 0;
  scanServo.write(sweepDeg);
  sweepStepAt = millis() + 300;  // extra settle to reach 0 from wherever
  sendLine("D,START");
}

void serviceSweep() {
  if (!sweeping) return;
  if ((long)(millis() - sweepStepAt) < 0) return;
  unsigned int cm = pingCm();
  char buf[20];
  snprintf(buf, sizeof(buf), "D,%d,%u", sweepDeg, cm);
  sendLine(buf);
  sweepDeg += 5;
  if (sweepDeg > 180) {
    sweeping = false;
    scanServo.write(90);
    sendLine("D,END");
    return;
  }
  scanServo.write(sweepDeg);
  sweepStepAt = millis() + SWEEP_SETTLE_MS;
}

void applyLineCmd(const char *s) {
  if (s[0] != '#') return;
  char kind = s[1];
  if (kind >= 'A' && kind <= 'Z') kind += 32;

  if (kind == 'k') {
    int pwm = 0, ms = 0;
    if (sscanf(s + 2, "%d,%d", &pwm, &ms) < 2) return;
    pwm = constrain(pwm, 0, 255);
    ms = constrain(ms, 0, 500);
    kickPwm = (unsigned char)pwm;
    kickMs = (unsigned int)ms;
    Serial.print("Kick "); Serial.print(kickPwm);
    Serial.print(" / "); Serial.print(kickMs); Serial.println(" ms");
    return;
  }
  if (kind == 't') {
    int pwm = 0;
    if (sscanf(s + 2, "%d", &pwm) < 1) return;
    turnPwmCap = (unsigned char)constrain(pwm, 0, 255);
    Serial.print("TurnCap "); Serial.println(turnPwmCap);
    return;
  }
  if (kind == 'w') {
    int l = 100, r = 100;
    if (sscanf(s + 2, "%d,%d", &l, &r) < 2) return;
    trimL = (unsigned char)constrain(l, 50, 100);
    trimR = (unsigned char)constrain(r, 50, 100);
    if (leftDrive || rightDrive) {
      unsigned char c = cruiseFor(leftDrive, rightDrive);
      writeWheels(leftDrive ? c : 0, rightDrive ? c : 0);
    }
    Serial.print("Trim "); Serial.print(trimL); Serial.print(","); Serial.println(trimR);
    return;
  }
  if (kind == 'u') {
    int cm = 0;
    if (sscanf(s + 2, "%d", &cm) < 1) return;
    stopCm = (unsigned int)constrain(cm, 0, 400);
    if (stopCm == 0) usBlocked = false;
    Serial.print("StopCm "); Serial.println(stopCm);
    return;
  }
  if (kind == 's') { startSweep(); return; }
  if (kind == 'x') {
    if (sweeping) { sweeping = false; scanServo.write(90); sendLine("D,ABORT"); }
    return;
  }
}

void doCmd(char c) {
  if (c >= 'a' && c <= 'z') c -= 32;
  if (c >= '0' && c <= '9') {
    speedPwm = (c - '0') * 28;
    if (speedPwm < 40 && c != '0') speedPwm = 40;
    if (leftDrive || rightDrive) {
      unsigned char cruise = cruiseFor(leftDrive, rightDrive);
      writeWheels(leftDrive ? cruise : 0, rightDrive ? cruise : 0);
    }
    Serial.print("Speed "); Serial.println(speedPwm);
    return;
  }
  if (sweeping && c != 'S') return;  // motors locked during sweep (S still allowed)
  switch (c) {
    case 'F':
      if (usBlocked) { Serial.println("F refused (obstacle)"); stopp(); break; }
      advance(); Serial.println("F"); break;
    case 'B': back();  Serial.println("B"); break;
    case 'L': turnL(); Serial.println("L soft"); break;
    case 'R': turnR(); Serial.println("R soft"); break;
    case 'S': stopp(); Serial.println("S"); break;
  }
}

void feedChar(char c) {
  if (!inLineCmd) {
    if (c == '#') {
      inLineCmd = true;
      lineLen = 0;
      lineBuf[lineLen++] = c;
      return;
    }
    doCmd(c);
    return;
  }
  if (c == '\r') return;
  if (c == '\n' || lineLen >= sizeof(lineBuf) - 1) {
    lineBuf[lineLen] = 0;
    applyLineCmd(lineBuf);
    inLineCmd = false;
    lineLen = 0;
    return;
  }
  lineBuf[lineLen++] = c;
}

void setup() {
  pinMode(pinLB, OUTPUT); pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT); pinMode(pinRF, OUTPUT);
  pinMode(Lpwm_pin, OUTPUT); pinMode(Rpwm_pin, OUTPUT);
  pinMode(US_TRIG, OUTPUT); digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO, INPUT);
  stopp();
  Serial.begin(9600);
  camSerial.begin(9600);
  Serial.println("Uno v1.3: trim + ultrasonic stop + sweep");
}

void loop() {
  while (Serial.available()) feedChar(Serial.read());
  while (camSerial.available()) feedChar(camSerial.read());
  if (sweeping) serviceSweep();
  else serviceUltrasonic();
}
