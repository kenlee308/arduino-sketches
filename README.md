# arduino-sketches

Arduino and ESP32-CAM sketches behind Kenny's camera rover: an Arduino Uno drives the motors and ultrasonic sensor, and an ESP32-CAM streams video over Wi-Fi and relays commands to it.

![Demo GIF coming soon](docs/demo-placeholder.gif)
*Demo GIF coming soon.*

## Features
- ESP32-CAM web server with live MJPEG streaming (`CameraWebServer`)
- Single-JPEG capture endpoint (`CAM_JPEG`)
- Wi-Fi command channel from the ESP32-CAM to the Uno (`CAM_Command_Test`, `Uno_Rover_CAM_Listen`)
- Serial-driven rover motor control on the Uno (`Uno_Rover_Commad`)
- Ultrasonic obstacle-avoidance car (`Lesson_12_Ultrasound_Obstacle_Avoidance_Car`)
- On-board LED test for the ESP32-CAM (`CAM_LED_Test`)

## Hardware
- ESP32-CAM (AI-Thinker style, OV2640) and a USB-to-serial/FTDI programmer
- Arduino Uno with a motor driver shield and a 2WD/4WD car chassis
- HC-SR04 ultrasonic sensor
- Battery pack for the motors; jumper wires between the ESP32-CAM UART and the Uno

## Setup
1. Install the Arduino IDE (2.x) and add the **esp32** board package (Espressif) via Boards Manager.
2. **Wi-Fi credentials:** in each ESP32 sketch folder (`CameraWebServer`, `CAM_JPEG`, `CAM_Command_Test`), copy `secrets.h.example` to `secrets.h` and fill in your SSID and password. `secrets.h` is gitignored; never commit it.
3. Open the sketch folder in the Arduino IDE.

## Build / flash
- **ESP32-CAM:** board "AI Thinker ESP32-CAM" (or "ESP32 Dev Module" with PSRAM on). Hold IO0 to GND, reset, upload, then remove IO0 and reset. Open the Serial Monitor at 115200 to see the IP address.
- **Uno:** board "Arduino Uno", pick the COM port, upload. Disconnect the ESP32 UART wires while uploading to the Uno.

## License
Personal hobby project; `CameraWebServer` is derived from the Espressif example (Apache-2.0).