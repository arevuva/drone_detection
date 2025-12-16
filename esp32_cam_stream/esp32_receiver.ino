/*
   ESP32 DevKitC: индикатор дрона (PCD8544 Nokia 5110 + зумер) по Wi-Fi/UDP.

   ПК отправляет UDP-пакеты на порт telemetryPort:
     "DRONE 0.87\n"  -> включить зумер, показать YES и 87%
     "NONE\n"        -> выключить зумер, показать NO
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// ====== Wi-Fi ======
const char* ssid     = "BurningCocks";   // твоя сеть
const char* password = "karmanen";       // твой пароль

// ====== UDP ======
WiFiUDP telemetryUdp;
const uint16_t telemetryPort = 9000;     // этот порт должен использовать ПК

// ====== Зумер / светодиод ======
#ifndef BEEPER_PIN
#define BEEPER_PIN 25                    // можно поменять при необходимости
#endif

// ====== PCD8544 (Nokia 5110) по SPI ======
// Пины по твоей схеме:
// RST -> GPIO 2
// CE  -> GPIO 15
// DC  -> GPIO 4
// DIN -> GPIO 23
// CLK -> GPIO 18
#define LCD_PIN_SCLK 18
#define LCD_PIN_DIN  23
#define LCD_PIN_DC   4
#define LCD_PIN_CE   15
#define LCD_PIN_RST  2

Adafruit_PCD8544 display(LCD_PIN_SCLK, LCD_PIN_DIN, LCD_PIN_DC, LCD_PIN_CE, LCD_PIN_RST);
bool lcdPresent = false;

// ====== Состояние детекции ======
bool  droneDetected        = false;
float droneProb            = 0.0f;
unsigned long lastSignalMs = 0;
const unsigned long signalHoldMs = 800;      // держим зумер активным после последнего DRONE
unsigned long lastDisplayMs      = 0;
const unsigned long displayIntervalMs = 500; // обновление дисплея не чаще, чем раз в 0.5 сек

// ====== Отрисовка статуса на дисплее ======
void showStatus() {
  if (!lcdPresent) return;

  display.clearDisplay();
  display.setTextColor(BLACK);

  // 84x48, текст размером 1 (6x8px на символ)
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println("Drone:");

  display.setCursor(0, 12);
  if (droneDetected) {
    display.println("YES");
    display.setCursor(0, 24);
    display.print("Prob:");
    display.print(droneProb * 100.0f, 1);
    display.println("%");
  } else {
    display.println("NO");
  }

  display.display();
}

// ====== Обработка входящих UDP-пакетов ======
void handleTelemetry() {
  int packetSize = telemetryUdp.parsePacket();
  if (!packetSize) {
    return;
  }

  char buf[64] = {0};
  int len = telemetryUdp.read(buf, sizeof(buf) - 1);
  if (len <= 0) {
    return;
  }

  String line = String(buf);
  line.trim();

  Serial.print("UDP: ");
  Serial.println(line);

  if (line.startsWith("DRONE")) {
    int idx = line.indexOf(' ');
    if (idx > 0) {
      droneProb = line.substring(idx + 1).toFloat();
    }
    droneDetected = true;
    lastSignalMs = millis();
  } else if (line.startsWith("NONE")) {
    droneDetected = false;
    droneProb = 0.0f;
    lastSignalMs = millis();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("ESP32 DevKitC drone indicator (PCD8544)");

  pinMode(BEEPER_PIN, OUTPUT);
  digitalWrite(BEEPER_PIN, LOW);

  // --- Инициализация PCD8544 ---
  if (display.begin()) {
    lcdPresent = true;
    display.setContrast(60);      // при необходимости подстрой
    display.clearDisplay();
    display.setTextColor(BLACK);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Connecting");
    display.println("WiFi...");
    display.display();
    Serial.println("PCD8544 init OK");
  } else {
    Serial.println("PCD8544 init failed, continue without display");
  }

  // --- Wi-Fi ---
  Serial.printf("Connecting to Wi-Fi: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Wi-Fi connected, IP address: ");
  Serial.println(WiFi.localIP());

  if (lcdPresent) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi OK");
    display.setCursor(0, 12);
    display.print("IP:");
    display.println(WiFi.localIP());
    display.display();
  }

  // --- UDP ---
  telemetryUdp.begin(telemetryPort);
  Serial.printf("Listening for telemetry on UDP port %u\n", telemetryPort);
}

void loop() {
  handleTelemetry();

  unsigned long now = millis();
  bool active = droneDetected && (now - lastSignalMs < signalHoldMs);
  digitalWrite(BEEPER_PIN, active ? HIGH : LOW);

  if (lcdPresent && now - lastDisplayMs >= displayIntervalMs) {
    showStatus();
    lastDisplayMs = now;
  }

  delay(5); // небольшой yield
}
