#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>

#include "secrets.h"
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

// UART to Uno (not the USB pins)
HardwareSerial RoverSerial(1);
#define ROVER_RX 15   // CAM RX  <- Uno pin 11 (via level shift)
#define ROVER_TX 14   // CAM TX  -> Uno pin 10

WebServer server(80);

void sendMove(char dir) {
  RoverSerial.write(dir);
  Serial.print("Sent to Uno: ");
  Serial.println(dir);
}

void handleRoot() {
  server.send(200, "text/plain",
    "OK\nGET /move?dir=F|B|L|R|S\n");
}

void handleMove() {
  if (!server.hasArg("dir")) {
    server.send(400, "text/plain", "missing dir");
    return;
  }
  String d = server.arg("dir");
  d.toUpperCase();
  char c = d.charAt(0);
  if (c != 'F' && c != 'B' && c != 'L' && c != 'R' && c != 'S') {
    server.send(400, "text/plain", "dir must be F B L R S");
    return;
  }
  sendMove(c);
  server.send(200, "text/plain", String("sent ") + c);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  RoverSerial.begin(9600, SERIAL_8N1, ROVER_RX, ROVER_TX);

  WiFi.begin(ssid, password);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.begin();
  Serial.println("HTTP ready");
}

void loop() {
  server.handleClient();
}