// Project J - OTA Server + Fleet Management (Team FleetForge)
// Step 3: basic program - sensor, OLED, RGB LED, buzzer, structured logs, version, boot count
// No WiFi yet. FreeRTOS tasks come in Step 4.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ─── FIRMWARE VERSION ────────────────────────────────────────────────────
#define FW_MAJOR 1
#define FW_MINOR 0
#define FW_PATCH 0
const uint32_t FW_VERSION = (FW_MAJOR << 16) | (FW_MINOR << 8) | FW_PATCH;

// ─── PINS ────────────────────────────────────────────────────────────────
#define DHT_PIN     4
#define BUZZER_PIN  25
#define LED_R       26
#define LED_G       27
#define LED_B       33
#define OLED_SDA    21
#define OLED_SCL    22

#define BUZZER_ON   LOW     // our buzzer module is active-low
#define BUZZER_OFF  HIGH

// ─── SETTINGS ────────────────────────────────────────────────────────────
#define TEMP_ALARM_C   30.0
#define READ_EVERY_MS  2000

// ─── STRUCTURED LOGGING ──────────────────────────────────────────────────
// Every line: [time ms] [LEVEL] message
#define LOG_INFO(fmt, ...)  Serial.printf("[%8lu] [INFO]  " fmt "\n", millis(), ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  Serial.printf("[%8lu] [WARN]  " fmt "\n", millis(), ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) Serial.printf("[%8lu] [ERROR] " fmt "\n", millis(), ##__VA_ARGS__)

// ─── BOOT COUNT (kept in RTC memory, survives deep sleep) ────────────────
RTC_DATA_ATTR uint32_t bootCount = 0;

DHT dht(DHT_PIN, DHT11);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;

void setColor(bool r, bool g, bool b) {
    digitalWrite(LED_R, r);
    digitalWrite(LED_G, g);
    digitalWrite(LED_B, b);
}

void beep(int ms) {
    digitalWrite(BUZZER_PIN, BUZZER_ON);
    delay(ms);
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
}

void showOnOled(float t, float h, bool alarm, bool sensorOk) {
    if (!oledOk) return;
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.printf("FleetForge v%d.%d.%d", FW_MAJOR, FW_MINOR, FW_PATCH);
    oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    oled.setTextSize(2);
    oled.setCursor(0, 16);
    if (sensorOk) {
        oled.printf("T:%.1fC\n", t);
        oled.printf("H:%.0f%%", h);
    } else {
        oled.print("SENSOR\nERROR");
    }

    oled.setTextSize(1);
    oled.setCursor(0, 56);
    oled.printf("Boot:%lu  %s", bootCount, alarm ? "ALARM!" : "OK");
    oled.display();
}

void setup() {
    Serial.begin(115200);
    delay(500);
    bootCount++;

    LOG_INFO("===== FleetForge Node starting =====");
    LOG_INFO("Firmware v%d.%d.%d (0x%06lX) | Boot #%lu",
             FW_MAJOR, FW_MINOR, FW_PATCH, FW_VERSION, bootCount);
    LOG_INFO("Chip: %s | IDF: %s", ESP.getChipModel(), esp_get_idf_version());

    // Outputs
    digitalWrite(BUZZER_PIN, BUZZER_OFF);   // silent before enabling output
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    setColor(0, 0, 1);                      // blue = starting up

    // OLED
    Wire.begin(OLED_SDA, OLED_SCL);
    if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        oledOk = true;
        oled.setTextColor(SSD1306_WHITE);
        LOG_INFO("OLED ready");
    } else {
        LOG_ERROR("OLED not found at 0x3C");
    }

    // Sensor
    dht.begin();
    LOG_INFO("DHT11 ready on GPIO %d | alarm above %.1f C", DHT_PIN, TEMP_ALARM_C);
}

void loop() {
    static uint32_t lastRead = 0;
    static bool lastAlarm = false;

    if (millis() - lastRead < READ_EVERY_MS) return;
    lastRead = millis();

    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (isnan(t) || isnan(h)) {
        LOG_ERROR("DHT11 read failed");
        setColor(0, 0, 1);                  // blue = sensor problem
        showOnOled(0, 0, false, false);
        return;
    }

    bool alarm = t > TEMP_ALARM_C;

    if (alarm) {
        setColor(1, 0, 0);                  // red
        beep(150);
        LOG_WARN("Temp %.1f C | Hum %.0f %% | ABOVE THRESHOLD", t, h);
    } else {
        setColor(0, 1, 0);                  // green
        LOG_INFO("Temp %.1f C | Hum %.0f %%", t, h);
    }

    // Log only when the alarm state changes
    if (alarm != lastAlarm) {
        if (alarm) LOG_WARN("ALARM started");
        else       LOG_INFO("Alarm cleared");
        lastAlarm = alarm;
    }

    showOnOled(t, h, alarm, true);
}
