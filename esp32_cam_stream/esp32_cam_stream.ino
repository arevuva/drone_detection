/*
 * ESP32-CAM MJPEG streaming sketch compatible with esp32_stream_detect.py.
 * Flash to an AI-Thinker ESP32-CAM board, update Wi-Fi credentials below,
 * then open http://<board-ip>:81/stream to get the video feed.
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "esp_http_server.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#if CONFIG_IDF_TARGET_ESP32
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// #define CAMERA_MODEL_AI_THINKER
// Uncomment if you use ESP32-S3-EYE / ESP32-S3 CAM board with OV2640:
#define CAMERA_MODEL_ESP32S3_EYE

// Optional buzzer/LED pin to signal detection received from PC.
// Choose a free GPIO not used by camera; adjust to your board.
#ifndef BEEPER_PIN
#define BEEPER_PIN 33
#endif

// I2C pins for SSD1306 OLED (change to match your wiring).
#ifndef OLED_SDA
#define OLED_SDA 21  // SDA wire
#endif
#ifndef OLED_SCL
#define OLED_SCL 19  // SCL wire
#endif

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#elif defined(CAMERA_MODEL_ESP32S3_EYE)
// Common pin map for ESP32-S3-CAM / ESP32-S3-EYE with OV2640.
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 17
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 12
#define Y5_GPIO_NUM 10
#define Y4_GPIO_NUM 8
#define Y3_GPIO_NUM 9
#define Y2_GPIO_NUM 11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13
#else
#error "Select a supported CAMERA_MODEL_* define for your board."
#endif

const char *ssid = "BurningCocks";
const char *password = "karmanen";

// State received from host PC (updated via Serial).
volatile bool droneDetected = false;
volatile float droneProb = 0.0f;
unsigned long lastSignalMs = 0;
const unsigned long signalHoldMs = 800;  // keep beeper active shortly after last detection
unsigned long lastDisplayMs = 0;
const unsigned long displayIntervalMs = 1000;  // update screen at most once per second
int consecutiveDetections = 0;
const int requiredDetections = 3;  // number of consecutive DRONE messages required to confirm detection

void showStatus() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Drone:");
  display.setCursor(0, 24);
  if (droneDetected) {
    display.println("YES");
    display.setTextSize(1);
    display.setCursor(0, 48);
    display.print("Prob: ");
    display.print(droneProb * 100.0f, 1);
    display.println("%");
  } else {
    display.println("NO");
  }
  display.display();
}

// Streaming server definitions (borrowed from the official CameraWebServer example).
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return ESP_FAIL;
    }

    res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    if (res == ESP_OK) {
      char buf[64];
      size_t hlen = snprintf(buf, sizeof(buf), _STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    }
    esp_camera_fb_return(fb);

    if (res != ESP_OK) {
      break;
    }
  }
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;  // default camera stream port

  httpd_uri_t stream_uri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL};

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setup() {
  // On ESP32-S3 the brownout register differs; only disable for classic ESP32.
#if CONFIG_IDF_TARGET_ESP32
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();
  Serial.println("Booting ESP32-CAM");

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Waiting for data...");
    display.display();
  }

  pinMode(BEEPER_PIN, OUTPUT);
  digitalWrite(BEEPER_PIN, LOW);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // Lower resolution/quality improves stability and latency on Wi-Fi.
  config.frame_size = FRAMESIZE_VGA;  // 320x240 by default; try FRAMESIZE_VGA if link is stable
  config.jpeg_quality = 20;            // 10-20; higher number = more compression
  config.fb_count = 2;
#if defined(CAMERA_GRAB_LATEST)
  config.grab_mode = CAMERA_GRAB_LATEST;  // drop old frames instead of queueing (lower latency)
#endif

  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 3;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  WiFi.begin(ssid, password);
  Serial.printf("Connecting to Wi-Fi: %s\n", ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Wi-Fi connected, IP address: ");
  Serial.println(WiFi.localIP());

  startCameraServer();
  Serial.println("Camera stream ready");
  Serial.println("Open in browser or client: http://<above-ip>:81/stream");
}

void loop() {
  // Non-blocking read of serial telemetry from PC (esp32_stream_detect.py).
  while (Serial.available()) {
    static String line;
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) {
        line.trim();
        if (line.startsWith("DRONE")) {
          int idx = line.indexOf(' ');
          if (idx > 0) {
            droneProb = line.substring(idx + 1).toFloat();
          }
          // Require consecutive DRONE messages before confirming detection.
          consecutiveDetections = min(consecutiveDetections + 1, requiredDetections);
          if (consecutiveDetections >= requiredDetections) {
            droneDetected = true;
            lastSignalMs = millis();
          }
        } else if (line.startsWith("NONE")) {
          droneDetected = false;
          droneProb = 0.0f;
          consecutiveDetections = 0;
          lastSignalMs = millis();
        }
        line = "";
      }
    } else {
      line += c;
    }
  }

  // Drive buzzer/LED based on last detection.
  unsigned long now = millis();
  bool active = droneDetected && (now - lastSignalMs < signalHoldMs);
  digitalWrite(BEEPER_PIN, active ? HIGH : LOW);

  // Update OLED at most once per second.
  if (now - lastDisplayMs >= displayIntervalMs) {
    showStatus();
    lastDisplayMs = now;
  }

  // Small sleep to avoid busy loop; streaming handled by HTTP server task.
  delay(10);
}
