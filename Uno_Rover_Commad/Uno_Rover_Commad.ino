// Rover Lesson-12 motors + Serial commands for ESP32 later
// Commands (one letter, then Enter): F B L R S
// Optional: 0-9 sets speed (0=stop PWM, 9=near full)

#define Lpwm_pin 5   // ENA
#define Rpwm_pin 6   // ENB
#define pinLB 2      // IN1
#define pinLF 4      // IN2
#define pinRB 7      // IN3
#define pinRF 8      // IN4

unsigned char speedPwm = 120;

void Set_Speed(unsigned char pwm) {
  analogWrite(Lpwm_pin, pwm);
  analogWrite(Rpwm_pin, pwm);
}

void advance() {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
}

void back() {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
}

void turnR() {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
}

void turnL() {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
}

void stopp() {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, HIGH);
  Set_Speed(0);
}

void setup() {
  pinMode(pinLB, OUTPUT);
  pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinRF, OUTPUT);
  pinMode(Lpwm_pin, OUTPUT);
  pinMode(Rpwm_pin, OUTPUT);
  stopp();

  Serial.begin(9600);
  Serial.println("Rover ready. Commands: F B L R S, speed 0-9");
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c >= 'a' && c <= 'z') c -= 32;  // uppercase

  if (c >= '0' && c <= '9') {
    speedPwm = (c - '0') * 28;  // 0..252
    Set_Speed(speedPwm);
    Serial.print("Speed ");
    Serial.println(speedPwm);
    return;
  }

  switch (c) {
    case 'F':
      Set_Speed(speedPwm);
      advance();
      Serial.println("F");
      break;
    case 'B':
      Set_Speed(speedPwm);
      back();
      Serial.println("B");
      break;
    case 'L':
      Set_Speed(speedPwm);
      turnL();
      Serial.println("L");
      break;
    case 'R':
      Set_Speed(speedPwm);
      turnR();
      Serial.println("R");
      break;
    case 'S':
      stopp();
      Serial.println("S");
      break;
    default:
      break;
  }
}