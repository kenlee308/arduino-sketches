#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include <HardwareSerial.h>
#include <ctype.h>

#include "secrets.h"
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

HardwareSerial RoverSerial(1);
#define ROVER_RX 15
#define ROVER_TX 14

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t server = NULL;

bool initCamera() {
  camera_config_t config;
  memset(&config, 0, sizeof(config));
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 20;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return false;
  }
  for (int i = 0; i < 3; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }
  return true;
}

void sendMove(char dir) {
  RoverSerial.write(dir);
}

void sendKick(int pwm, int ms) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  if (ms < 0) ms = 0;
  if (ms > 500) ms = 500;
  char buf[24];
  snprintf(buf, sizeof(buf), "#k%d,%d\n", pwm, ms);
  RoverSerial.print(buf);
}

void sendTrim(int l, int r) {
  if (l < 50) l = 50;
  if (l > 100) l = 100;
  if (r < 50) r = 50;
  if (r > 100) r = 100;
  char buf[20];
  snprintf(buf, sizeof(buf), "#w%d,%d\n", l, r);
  RoverSerial.print(buf);
}

// ---- Uno -> CAM telemetry (GPIO15 via resistor divider) ----
#define SCAN_MAX 40
static portMUX_TYPE telMux = portMUX_INITIALIZER_UNLOCKED;
static int16_t scanDeg[SCAN_MAX];
static int16_t scanCm[SCAN_MAX];
static volatile int scanCount = 0;
static volatile bool scanDone = false;
static volatile bool scanActive = false;
static volatile int lastDistCm = -1;          // -1 = never received
static volatile uint32_t lastDistMs = 0;
static char rxLine[32];
static int rxLen = 0;

void handleRoverLine(const char *l) {
  if (l[0] == 'U' && l[1] == ',') {
    lastDistCm = atoi(l + 2);
    lastDistMs = millis();
    return;
  }
  if (l[0] == 'D' && l[1] == ',') {
    const char *v = l + 2;
    portENTER_CRITICAL(&telMux);
    if (strncmp(v, "START", 5) == 0) {
      scanCount = 0; scanDone = false; scanActive = true;
    } else if (strncmp(v, "END", 3) == 0 || strncmp(v, "ABORT", 5) == 0) {
      scanDone = true; scanActive = false;
    } else {
      int deg = 0, cm = 0;
      if (sscanf(v, "%d,%d", &deg, &cm) == 2 && scanCount < SCAN_MAX) {
        scanDeg[scanCount] = deg;
        scanCm[scanCount] = cm;
        scanCount++;
      }
    }
    portEXIT_CRITICAL(&telMux);
  }
}

void pollRoverSerial() {
  while (RoverSerial.available()) {
    char c = (char)RoverSerial.read();
    if (c == '\r') continue;
    if (c == '\n' || rxLen >= (int)sizeof(rxLine) - 1) {
      rxLine[rxLen] = 0;
      if (rxLen > 0) handleRoverLine(rxLine);
      rxLen = 0;
      continue;
    }
    rxLine[rxLen++] = c;
  }
}

void sendTurnCap(int pwm) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  char buf[16];
  snprintf(buf, sizeof(buf), "#t%d\n", pwm);
  RoverSerial.print(buf);
}

static esp_err_t root_handler(httpd_req_t *req) {
  const char *msg =
    "GET /move?dir=F|B|L|R|S|0-9\n"
    "GET /kick?pwm=0-255&ms=0-500\n"
    "GET /turncap?pwm=0-255\n"
    "GET /trim?l=50-100&r=50-100\n"
    "GET /stopcm?cm=0-400\n"
    "GET /scan/start\n"
    "GET /scan\n"
    "GET /dist\n"
    "GET /capture\n"
    "GET /stream\n"
    "GET /status\n";
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, msg, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req) {
  char buf[192];
  snprintf(buf, sizeof(buf),
           "{\"ip\":\"%s\",\"rssi\":%d,\"heap\":%u,\"ssid\":\"%s\"}",
           WiFi.localIP().toString().c_str(),
           WiFi.RSSI(),
           (unsigned)ESP.getFreeHeap(),
           ssid);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t move_handler(httpd_req_t *req) {
  char q[64];
  if (httpd_req_get_url_query_len(req) == 0 ||
      httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing dir");
    return ESP_FAIL;
  }
  char dir[8] = {0};
  if (httpd_query_key_value(q, "dir", dir, sizeof(dir)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing dir");
    return ESP_FAIL;
  }
  char c = toupper(dir[0]);
  bool ok = (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S' ||
             (c >= '0' && c <= '9'));
  if (!ok) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad dir");
    return ESP_FAIL;
  }
  sendMove(c);
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "ok", 2);
}

static esp_err_t turncap_handler(httpd_req_t *req) {
  char q[64];
  if (httpd_req_get_url_query_len(req) == 0 ||
      httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing params");
    return ESP_FAIL;
  }
  char pwm_s[8] = {0};
  if (httpd_query_key_value(q, "pwm", pwm_s, sizeof(pwm_s)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need pwm");
    return ESP_FAIL;
  }
  sendTurnCap(atoi(pwm_s));
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "ok", 2);
}

static esp_err_t kick_handler(httpd_req_t *req) {
  char q[64];
  if (httpd_req_get_url_query_len(req) == 0 ||
      httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing params");
    return ESP_FAIL;
  }
  char pwm_s[8] = {0};
  char ms_s[8] = {0};
  if (httpd_query_key_value(q, "pwm", pwm_s, sizeof(pwm_s)) != ESP_OK ||
      httpd_query_key_value(q, "ms", ms_s, sizeof(ms_s)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need pwm and ms");
    return ESP_FAIL;
  }
  int pwm = atoi(pwm_s);
  int ms = atoi(ms_s);
  sendKick(pwm, ms);
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "ok", 2);
}

static esp_err_t trim_handler(httpd_req_t *req) {
  char q[64];
  char l_s[8] = {0}, r_s[8] = {0};
  if (httpd_req_get_url_query_len(req) == 0 ||
      httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK ||
      httpd_query_key_value(q, "l", l_s, sizeof(l_s)) != ESP_OK ||
      httpd_query_key_value(q, "r", r_s, sizeof(r_s)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need l and r");
    return ESP_FAIL;
  }
  sendTrim(atoi(l_s), atoi(r_s));
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "ok", 2);
}

static esp_err_t stopcm_handler(httpd_req_t *req) {
  char q[64];
  char cm_s[8] = {0};
  if (httpd_req_get_url_query_len(req) == 0 ||
      httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK ||
      httpd_query_key_value(q, "cm", cm_s, sizeof(cm_s)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need cm");
    return ESP_FAIL;
  }
  int cm = atoi(cm_s);
  if (cm < 0) cm = 0;
  if (cm > 400) cm = 400;
  char buf[16];
  snprintf(buf, sizeof(buf), "#u%d\n", cm);
  RoverSerial.print(buf);
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "ok", 2);
}

static esp_err_t scan_start_handler(httpd_req_t *req) {
  portENTER_CRITICAL(&telMux);
  scanCount = 0; scanDone = false; scanActive = true;
  portEXIT_CRITICAL(&telMux);
  RoverSerial.print("#s\n");
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "ok", 2);
}

static esp_err_t scan_handler(httpd_req_t *req) {
  static char out[SCAN_MAX * 16 + 48];
  int16_t d[SCAN_MAX], c[SCAN_MAX];
  int n; bool done;
  portENTER_CRITICAL(&telMux);
  n = scanCount; done = scanDone;
  for (int i = 0; i < n; i++) { d[i] = scanDeg[i]; c[i] = scanCm[i]; }
  portEXIT_CRITICAL(&telMux);
  int len = snprintf(out, sizeof(out), "{\"done\":%s,\"points\":[", done ? "true" : "false");
  for (int i = 0; i < n && len < (int)sizeof(out) - 20; i++) {
    len += snprintf(out + len, sizeof(out) - len, "%s[%d,%d]", i ? "," : "", d[i], c[i]);
  }
  len += snprintf(out + len, sizeof(out) - len, "]}");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, out, len);
}

static esp_err_t dist_handler(httpd_req_t *req) {
  char buf[64];
  int cm = lastDistCm;
  long age = (cm < 0) ? -1 : (long)(millis() - lastDistMs);
  snprintf(buf, sizeof(buf), "{\"cm\":%d,\"age_ms\":%ld}", cm, age);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  char part_buf[64];
  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
      break;
    }
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    }
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
    // ~5 fps — leave airtime for /move and /capture
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.max_uri_handlers = 16;
  config.max_open_sockets = 7;
  config.lru_purge_enable = true;

  httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_handler };
  httpd_uri_t move = { .uri = "/move", .method = HTTP_GET, .handler = move_handler };
  httpd_uri_t capture = { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler };
  httpd_uri_t stream = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler };
  httpd_uri_t status = { .uri = "/status", .method = HTTP_GET, .handler = status_handler };
  httpd_uri_t kick = { .uri = "/kick", .method = HTTP_GET, .handler = kick_handler };
  httpd_uri_t turncap = { .uri = "/turncap", .method = HTTP_GET, .handler = turncap_handler };
  httpd_uri_t trim = { .uri = "/trim", .method = HTTP_GET, .handler = trim_handler };
  httpd_uri_t stopcm = { .uri = "/stopcm", .method = HTTP_GET, .handler = stopcm_handler };
  httpd_uri_t scanstart = { .uri = "/scan/start", .method = HTTP_GET, .handler = scan_start_handler };
  httpd_uri_t scan = { .uri = "/scan", .method = HTTP_GET, .handler = scan_handler };
  httpd_uri_t dist = { .uri = "/dist", .method = HTTP_GET, .handler = dist_handler };

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &move);
    httpd_register_uri_handler(server, &capture);
    httpd_register_uri_handler(server, &stream);
    httpd_register_uri_handler(server, &status);
    httpd_register_uri_handler(server, &kick);
    httpd_register_uri_handler(server, &turncap);
    httpd_register_uri_handler(server, &trim);
    httpd_register_uri_handler(server, &stopcm);
    httpd_register_uri_handler(server, &scanstart);
    httpd_register_uri_handler(server, &scan);
    httpd_register_uri_handler(server, &dist);
    Serial.println("HTTP: /move /kick /turncap /trim /stopcm /scan/start /scan /dist /capture /stream /status");
  } else {
    Serial.println("HTTP start failed");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  RoverSerial.begin(9600, SERIAL_8N1, ROVER_RX, ROVER_TX);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(ssid, password);
  Serial.print("WiFi");
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("WiFi failed, status=");
    Serial.println(WiFi.status());
    return;
  }
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  if (!initCamera()) {
    Serial.println("No camera - /move may still work");
  } else {
    Serial.println("Camera OK (QVGA q=20)");
  }
  startCameraServer();
}

void loop() {
  pollRoverSerial();
  delay(5);
}
