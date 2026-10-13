// ESP32-CAM AI Thinker — onboard LED blink test (GPIO 33)
#define LED_PIN 33

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED off
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32-CAM LED test");
}

void loop() {
  digitalWrite(LED_PIN, LOW);   // on
  Serial.println("LED ON");
  delay(500);
  digitalWrite(LED_PIN, HIGH);  // off
  Serial.println("LED OFF");
  delay(500);
}