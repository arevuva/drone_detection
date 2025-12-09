// --- Настройки OLED-дисплея ---

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -&Wire, -1);
    // Инициализация OLED-дисплея
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }
        display.clearDisplay();
    display.display();
    updateDisplay("System Disarmed", "Welcome!");