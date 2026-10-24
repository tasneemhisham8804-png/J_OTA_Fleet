// Project J - OTA Server + Fleet Management (Team FleetForge)
// Step 4: FreeRTOS tasks - Sensor, Alarm and Display run in parallel
// No WiFi yet. Core 0 is kept free for the WiFi/MQTT task in Step 5.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <stdarg.h>

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

// ─── DATA SHARED BETWEEN TASKS ───────────────────────────────────────────
typedef struct {
    float    temperature;
    float    humidity;
    bool     valid;      // false if the DHT11 read failed
    bool     alarm;      // true if temperature > threshold
} SensorData_t;

QueueHandle_t alarmQueue;     // Sensor -> Alarm   (latest reading only)
QueueHandle_t displayQueue;   // Sensor -> Display (latest reading only)
SemaphoreHandle_t logMutex;   // only one task prints at a time

RTC_DATA_ATTR uint32_t bootCount = 0;

DHT dht(DHT_PIN, DHT11);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;

// ─── STRUCTURED LOGGING (thread-safe) ────────────────────────────────────
// Every line: [time ms] [LEVEL] [core] message
void logLine(const char* level, const char* fmt, ...) {
    char msg[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.printf("[%8lu] [%-5s] [C%d] %s\n", millis(), level, xPortGetCoreID(), msg);
        xSemaphoreGive(logMutex);
    }
}
#define LOG_INFO(...)  logLine("INFO",  __VA_ARGS__)
#define LOG_WARN(...)  logLine("WARN",  __VA_ARGS__)
#define LOG_ERROR(...) logLine("ERROR", __VA_ARGS__)

// ─── HELPERS ─────────────────────────────────────────────────────────────
void setColor(bool r, bool g, bool b) {
    digitalWrite(LED_R, r);
    digitalWrite(LED_G, g);
    digitalWrite(LED_B, b);
}

// ═════════════════════════════════════════════════════════════════════════
// TASK 1: SENSOR  (core 1, priority 3 - highest)
// Reads the DHT11 every 2 s and sends the result to the other tasks.
// Highest priority because the DHT11 timing must not be interrupted.
// ═════════════════════════════════════════════════════════════════════════
void sensorTask(void* param) {
    LOG_INFO("SensorTask started (priority %d)", uxTaskPriorityGet(NULL));
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {
        SensorData_t data;
        data.temperature = dht.readTemperature();
        data.humidity    = dht.readHumidity();
        data.valid       = !isnan(data.temperature) && !isnan(data.humidity);
        data.alarm       = data.valid && data.temperature > TEMP_ALARM_C;

        if (!data.valid) {
            LOG_ERROR("DHT11 read failed");
        } else if (data.alarm) {
            LOG_WARN("Temp %.1f C | Hum %.0f %% | ABOVE THRESHOLD", data.temperature, data.humidity);
        } else {
            LOG_INFO("Temp %.1f C | Hum %.0f %%", data.temperature, data.humidity);
        }

        // Overwrite = the mailbox always holds the newest reading
        xQueueOverwrite(alarmQueue, &data);
        xQueueOverwrite(displayQueue, &data);

        // Wait exactly 2 s from the last wake-up (fixed rhythm)
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(READ_EVERY_MS));
    }
}

// ═════════════════════════════════════════════════════════════════════════
// TASK 2: ALARM  (core 1, priority 2)
// Waits for new readings and controls the RGB LED and buzzer.
// ═════════════════════════════════════════════════════════════════════════
void alarmTask(void* param) {
    LOG_INFO("AlarmTask started (priority %d)", uxTaskPriorityGet(NULL));
    bool lastAlarm = false;
    SensorData_t data;

    while (true) {
        // Sleep here until SensorTask sends something (uses no CPU while waiting)
        if (xQueueReceive(alarmQueue, &data, portMAX_DELAY) != pdTRUE) continue;

        if (!data.valid) {
            setColor(0, 0, 1);                   // blue = sensor problem
        } else if (data.alarm) {
            setColor(1, 0, 0);                   // red = alarm
            digitalWrite(BUZZER_PIN, BUZZER_ON);
            vTaskDelay(pdMS_TO_TICKS(150));      // other tasks keep running during this
            digitalWrite(BUZZER_PIN, BUZZER_OFF);
        } else {
            setColor(0, 1, 0);                   // green = normal
        }

        if (data.valid && data.alarm != lastAlarm) {
            if (data.alarm) LOG_WARN("ALARM started");
            else            LOG_INFO("Alarm cleared");
            lastAlarm = data.alarm;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
// TASK 3: DISPLAY  (core 1, priority 1 - lowest)
// Waits for new readings and redraws the OLED.
// Lowest priority: a late screen update is not dangerous.
// ═════════════════════════════════════════════════════════════════════════
void displayTask(void* param) {
    LOG_INFO("DisplayTask started (priority %d)", uxTaskPriorityGet(NULL));
    SensorData_t data;

    while (true) {
        if (xQueueReceive(displayQueue, &data, portMAX_DELAY) != pdTRUE) continue;
        if (!oledOk) continue;

        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setCursor(0, 0);
        oled.printf("FleetForge v%d.%d.%d", FW_MAJOR, FW_MINOR, FW_PATCH);
        oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);

        oled.setTextSize(2);
        oled.setCursor(0, 16);
        if (data.valid) {
            oled.printf("T:%.1fC\n", data.temperature);
            oled.printf("H:%.0f%%", data.humidity);
        } else {
            oled.print("SENSOR\nERROR");
        }

        oled.setTextSize(1);
        oled.setCursor(0, 56);
        oled.printf("Boot:%lu  %s", bootCount, data.alarm ? "ALARM!" : "OK");
        oled.display();
    }
}

// ═════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    bootCount++;

    logMutex = xSemaphoreCreateMutex();   // must exist before the first log

    LOG_INFO("===== FleetForge Node starting =====");
    LOG_INFO("Firmware v%d.%d.%d (0x%06lX) | Boot #%lu",
             FW_MAJOR, FW_MINOR, FW_PATCH, FW_VERSION, bootCount);
    LOG_INFO("Chip: %s | IDF: %s", ESP.getChipModel(), esp_get_idf_version());

    // Outputs
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    setColor(0, 0, 1);                    // blue = starting up

    // OLED
    Wire.begin(OLED_SDA, OLED_SCL);
    if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        oledOk = true;
        oled.setTextColor(SSD1306_WHITE);
        LOG_INFO("OLED ready");
    } else {
        LOG_ERROR("OLED not found at 0x3C");
    }

    dht.begin();

    // Queues of length 1 = "mailbox" holding only the latest reading
    alarmQueue   = xQueueCreate(1, sizeof(SensorData_t));
    displayQueue = xQueueCreate(1, sizeof(SensorData_t));

    // Create the tasks:          function     name       stack  param priority handle core
    xTaskCreatePinnedToCore(sensorTask,  "SensorTask",  4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(alarmTask,   "AlarmTask",   4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 1, NULL, 1);

    LOG_INFO("All tasks created. Core 0 reserved for WiFi/MQTT (Step 5)");
}

void loop() {
    // Everything runs in tasks now, so the Arduino loop task deletes itself
    vTaskDelete(NULL);
}
