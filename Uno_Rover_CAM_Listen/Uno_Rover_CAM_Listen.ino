#include <SoftwareSerial.h>

SoftwareSerial camSerial(3, 12);  // RX=3 <- CAM GPIO14; TX=12 unused

#define Lpwm_pin 5
#define Rpwm_pin 6
#define pinLB 2
#define pinLF 4
#define pinRB 7
#define pinRF 8

unsigned char speedPwm = 70;  // crawl default until PC sends 1-9

// Tunable: "#k<pwm>,<ms>\n"  e.g. "#k200,70\n"  (pwm 0 = kick off)
unsigned char kickPwm = 200;
unsigned int kickMs = 70;

// Tunable: "#t<pwm>\n" max PWM during soft L/R. 0 = use full cruise speedPwm.
// Default 56 ~= UI speed 2, which felt good on turns.
unsigned char turnPwmCap = 56;

bool leftDrive = false;
bool rightDrive = false;

char lineBuf[24];
uint8_t lineLen = 0;
bool inLineCmd = false;

unsigned char cruiseFor(bool left, bool right) {
  bool turning = (left != right);
  unsigned char p = speedPwm;
  if (turning && turnPwmCap > 0 && p > turnPwmCap) p = turnPwmCap;
  return p;
}

void applyDrive(bool left, bool right) {
  unsigned char cruise = cruiseFor(left, right);
  unsigned char lp = left ? cruise : 0;
  unsigned char rp = right ? cruise : 0;

  bool kickL = left && !leftDrive && cruise > 0 && kickPwm > 0 && cruise < kickPwm;
  bool kickR = right && !rightDrive && cruise > 0 && kickPwm > 0 && cruise < kickPwm;

  analogWrite(Lpwm_pin, kickL ? kickPwm : lp);
  analogWrite(Rpwm_pin, kickR ? kickPwm : rp);

  if (kickL || kickR) {
    delay(kickMs);
    analogWrite(Lpwm_pin, lp);
    analogWrite(Rpwm_pin, rp);
  }

  leftDrive = left;
  rightDrive = right;
}

void advance() {
  digitalWrite(pinRB, LOW); digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW); digitalWrite(pinLF, HIGH);
  applyDrive(true, true);
}

void back() {
  digitalWrite(pinRB, HIGH); digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH); digitalWrite(pinLF, LOW);
  applyDrive(true, true);
}

void turnR() {
  digitalWrite(pinLB, LOW); digitalWrite(pinLF, HIGH);
  digitalWrite(pinRB, HIGH); digitalWrite(pinRF, HIGH);
  applyDrive(true, false);
}

void turnL() {
  digitalWrite(pinRB, LOW); digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH); digitalWrite(pinLF, HIGH);
  applyDrive(false, true);
}

void stopp() {
  digitalWrite(pinRB, HIGH); digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH); digitalWrite(pinLF, HIGH);
  analogWrite(Lpwm_pin, 0);
  analogWrite(Rpwm_pin, 0);
  leftDrive = false;
  rightDrive = false;
}

void applyLineCmd(const char *s) {
  if (s[0] != '#') return;
  char kind = s[1];
  if (kind >= 'A' && kind <= 'Z') kind += 32;

  if (kind == 'k') {
    int pwm = 0, ms = 0;
    if (sscanf(s + 2, "%d,%d", &pwm, &ms) < 2) return;
    if (pwm < 0) pwm = 0;
    if (pwm > 255) pwm = 255;
    if (ms < 0) ms = 0;
    if (ms > 500) ms = 500;
    kickPwm = (unsigned char)pwm;
    kickMs = (unsigned int)ms;
    Serial.print("Kick ");
    Serial.print(kickPwm);
    Serial.print(" / ");
    Serial.print(kickMs);
    Serial.println(" ms");
    return;
  }

  if (kind == 't') {
    int pwm = 0;
    if (sscanf(s + 2, "%d", &pwm) < 1) return;
    if (pwm < 0) pwm = 0;
    if (pwm > 255) pwm = 255;
    turnPwmCap = (unsigned char)pwm;
    Serial.print("TurnCap ");
    Serial.println(turnPwmCap);
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
      if (leftDrive) analogWrite(Lpwm_pin, cruise);
      if (rightDrive) analogWrite(Rpwm_pin, cruise);
    }
    Serial.print("Speed "); Serial.println(speedPwm);
    return;
  }
  switch (c) {
    case 'F': advance(); Serial.println("F"); break;
    case 'B': back();    Serial.println("B"); break;
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
  stopp();
  Serial.begin(9600);
  camSerial.begin(9600);
  Serial.println("Uno soft-turn + kick + turnCap, default PWM 70");
}

void loop() {
  while (Serial.available()) feedChar(Serial.read());
  while (camSerial.available()) feedChar(camSerial.read());
}
