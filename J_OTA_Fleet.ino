// Project J - OTA Server + Fleet Management (Team FleetForge)
// Step 9: deep sleep - timer wake, OTA window after each wake
// Step 8: OTA - an accepted upgrade opens a 2-minute ArduinoOTA window
// Step 7: Fleet version check - accept upgrades, reject downgrades
// (Step 5 base: WiFiManager + secure MQTT (TLS) to EMQX Cloud)
// Core 1: Sensor, Alarm, Display tasks   |   Core 0: Network task (WiFi + MQTT)

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiManager.h>        // tzapu
#include <WiFiClientSecure.h>
#include <PubSubClient.h>       // Nick O'Leary
#include <ArduinoOTA.h>         // comes with the ESP32 board package
#include <stdarg.h>
#include "secrets.h"            

#define FW_MAJOR 1
#define FW_MINOR 1
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
#define BOOT_BTN    0       // hold BOOT while pressing EN to forget saved WiFi

#define BUZZER_ON   LOW     // our buzzer module is active-low
#define BUZZER_OFF  HIGH

// ─── SETTINGS ────────────────────────────────────────────────────────────
#define TEMP_ALARM_C     30.0
#define READ_EVERY_MS    2000
#define PUBLISH_EVERY_MS 10000          // send telemetry to the cloud every 10 s
#define NODE_ID          "node01"
#define OTA_WINDOW_MS    120000         // OTA window stays open 2 minutes
#define OTA_HOSTNAME     "fleetforge-" NODE_ID
#define SLEEP_SECONDS   60       // timer wake, allowed range 30..300
#define FLEET_WAIT_MS   4000     // after MQTT connects, wait for the retained fleet version
#define AWAKE_MAX_MS    120000   // failsafe: sleep anyway if MQTT never comes up
// #define DEMO_NO_SLEEP         // uncomment for the live demo: stay awake

// ─── MQTT TOPICS ─────────────────────────────────────────────────────────
// ois/fleetforge/node01/telemetry   temperature, humidity, boot count ...
// ois/fleetforge/node01/alert       sent when the alarm starts or stops
// ois/fleetforge/node01/status      "online" / "offline" (retained + Last Will)
// ois/fleetforge/node01/info        firmware, chip, IDF version (retained)
// ois/fleetforge/node01/ota         result of each fleet version check
// ois/fleetforge/fleet/firmware     <- we SUBSCRIBE: latest fleet version, e.g. "1.1.0" (retained)
#define TOPIC_BASE      "ois/fleetforge/" NODE_ID
#define TOPIC_TELEMETRY TOPIC_BASE "/telemetry"
#define TOPIC_ALERT     TOPIC_BASE "/alert"
#define TOPIC_STATUS    TOPIC_BASE "/status"
#define TOPIC_INFO      TOPIC_BASE "/info"
#define TOPIC_OTA       TOPIC_BASE "/ota"
#define TOPIC_FLEET     "ois/fleetforge/fleet/firmware"

// ─── EMQX CA CERTIFICATE (DigiCert Global Root G2, public, valid to 2038) ─
static const char EMQX_CA_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----
)EOF";

// ─── DATA SHARED BETWEEN TASKS ───────────────────────────────────────────
typedef struct {
    float temperature;
    float humidity;
    bool  valid;
    bool  alarm;
} SensorData_t;

QueueHandle_t alarmQueue;     // Sensor -> Alarm
QueueHandle_t displayQueue;   // Sensor -> Display
QueueHandle_t mqttQueue;      // Sensor -> Network
SemaphoreHandle_t logMutex;

volatile bool wifiOk = false; // written by Network task, read by Display task
volatile bool mqttOk = false;

// Fleet version state (written by Network task, read by Display/Alarm tasks)
volatile bool     upgradePending = false;  // a newer version was announced
volatile uint32_t targetVersion  = 0;      // that newer version, as a number

// OTA state (written by Network task, read by Display/Alarm tasks)
volatile bool     otaWindowOpen = false;   // waiting for an upload
volatile bool     otaRunning    = false;   // upload in progress
volatile uint8_t  otaPercent    = 0;
volatile uint32_t otaWindowLeftS = 0;
uint32_t          otaHandledFor  = 0;      // version we already opened a window for

RTC_DATA_ATTR uint32_t bootCount = 0;

DHT dht(DHT_PIN, DHT11);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;

WiFiClientSecure tlsClient;
PubSubClient mqtt(tlsClient);
char clientId[32];

// ─── STRUCTURED LOGGING (thread-safe) ────────────────────────────────────
void logLine(const char* level, const char* fmt, ...) {
    char msg[200];
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

void setColor(bool r, bool g, bool b) {
    digitalWrite(LED_R, r);
    digitalWrite(LED_G, g);
    digitalWrite(LED_B, b);
}

// ─── VERSION HELPERS ─────────────────────────────────────────────────────
// Version is stored as ONE number: (major << 16) | (minor << 8) | patch
// 1.0.0 -> 65536   1.1.0 -> 65792   1.10.0 -> 68096   2.0.0 -> 131072
// Comparing numbers is always correct. Comparing text is NOT:
// as text "1.10.0" < "1.9.0" (because '1' < '9'), but as numbers 68096 > 67840.

// Parse "1.2.3" or "v1.2.3" into a number. Returns false if the text is invalid.
bool parseVersion(const char* text, uint32_t* out) {
    if (text[0] == 'v' || text[0] == 'V') text++;
    unsigned int major, minor, patch;
    char extra;
    // exactly three numbers, nothing after them
    if (sscanf(text, "%u.%u.%u%c", &major, &minor, &patch, &extra) != 3) return false;
    if (major > 255 || minor > 255 || patch > 255) return false;
    *out = (major << 16) | (minor << 8) | patch;
    return true;
}

// Turn the number back into "1.2.3" text
void versionToText(uint32_t v, char* buf, size_t len) {
    snprintf(buf, len, "%lu.%lu.%lu", (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF);
}

// ═════════════════════════════════════════════════════════════════════════
// TASK 1: SENSOR  (core 1, priority 3)
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

        if (!data.valid)     LOG_ERROR("DHT11 read failed");
        else if (data.alarm) LOG_WARN("Temp %.1f C | Hum %.0f %% | ABOVE THRESHOLD", data.temperature, data.humidity);
        else                 LOG_INFO("Temp %.1f C | Hum %.0f %%", data.temperature, data.humidity);

        xQueueOverwrite(alarmQueue,   &data);
        xQueueOverwrite(displayQueue, &data);
        xQueueOverwrite(mqttQueue,    &data);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(READ_EVERY_MS));
    }
}

// ═════════════════════════════════════════════════════════════════════════
// TASK 2: ALARM  (core 1, priority 2)
// ═════════════════════════════════════════════════════════════════════════
void alarmTask(void* param) {
    LOG_INFO("AlarmTask started (priority %d)", uxTaskPriorityGet(NULL));
    SensorData_t data;

    while (true) {
        if (xQueueReceive(alarmQueue, &data, portMAX_DELAY) != pdTRUE) continue;

        if (!data.valid) {
            setColor(0, 0, 1);                        // blue  = sensor problem
        } else if (data.alarm) {
            setColor(1, 0, 0);                        // red   = alarm
            digitalWrite(BUZZER_PIN, BUZZER_ON);
            vTaskDelay(pdMS_TO_TICKS(150));
            digitalWrite(BUZZER_PIN, BUZZER_OFF);
        } else if (otaRunning) {
            setColor(0, 1, 1);                        // cyan  = installing update
        } else if (upgradePending) {
            setColor(1, 0, 1);                        // purple = update available
        } else {
            setColor(0, 1, 0);                        // green = normal
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
// TASK 3: DISPLAY  (core 1, priority 1)
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
        oled.printf("FleetForge v%d.%d.%d NEW", FW_MAJOR, FW_MINOR, FW_PATCH);
        oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);

        oled.setTextSize(2);
        oled.setCursor(0, 14);
        if (data.valid) {
            oled.printf("T:%.1fC\n", data.temperature);
            oled.printf("H:%.0f%%", data.humidity);
        } else {
            oled.print("SENSOR\nERROR");
        }

        oled.setTextSize(1);
        oled.setCursor(0, 47);
        oled.printf("WiFi:%s  MQTT:%s", wifiOk ? "OK" : "--", mqttOk ? "OK" : "--");
        oled.setCursor(0, 56);
        if (data.alarm) {
            oled.printf("Boot:%lu  ALARM!", bootCount);
        } else if (otaRunning) {
            oled.printf("OTA installing %u%%", otaPercent);
        } else if (otaWindowOpen) {
            oled.printf("OTA open %lus", otaWindowLeftS);
        } else if (upgradePending) {
            char t[12];
            versionToText(targetVersion, t, sizeof(t));
            oled.printf("Update v%s ready", t);
        } else {
            oled.printf("Boot:%lu  OK", bootCount);
        }
        oled.display();
    }
}

// ─── FLEET VERSION CHECK ─────────────────────────────────────────────────
// Publishes the result so the fleet manager can see what every node decided
void publishOtaResult(const char* result, const char* announced) {
    char cur[12];
    versionToText(FW_VERSION, cur, sizeof(cur));
    char msg[160];
    snprintf(msg, sizeof(msg),
             "{\"result\":\"%s\",\"current\":\"%s\",\"announced\":\"%s\",\"boot\":%lu}",
             result, cur, announced, bootCount);
    mqtt.publish(TOPIC_OTA, msg);
}

// Called by PubSubClient (inside mqtt.loop(), in the Network task)
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    if (strcmp(topic, TOPIC_FLEET) != 0) return;

    // Copy payload into a normal C string, trimming spaces/newlines
    char text[24];
    unsigned int n = 0;
    for (unsigned int i = 0; i < length && n < sizeof(text) - 1; i++) {
        if (payload[i] != ' ' && payload[i] != '\r' && payload[i] != '\n') text[n++] = (char)payload[i];
    }
    text[n] = '\0';
    if (n == 0) return;   // empty message (e.g. a retained message being cleared)

    uint32_t announced;
    if (!parseVersion(text, &announced)) {
        LOG_ERROR("Fleet version '%s' is not valid (expected e.g. 1.1.0)", text);
        publishOtaResult("invalid", text);
        return;
    }

    char cur[12];
    versionToText(FW_VERSION, cur, sizeof(cur));
    LOG_INFO("Fleet version received: %s (%lu) | mine: %s (%lu)",
             text, announced, cur, FW_VERSION);

    if (announced > FW_VERSION) {
        LOG_INFO("UPGRADE ACCEPTED: %s -> %s", cur, text);
        upgradePending = true;
        targetVersion  = announced;
        otaHandledFor  = 0;                   // publishing again re-opens the window
        publishOtaResult("upgrade_accepted", text);
        // The Network task sees upgradePending and opens the OTA window
    } else if (announced == FW_VERSION) {
        LOG_INFO("Already up to date (%s)", cur);
        upgradePending = false;
        publishOtaResult("up_to_date", text);
    } else {
        LOG_WARN("DOWNGRADE REJECTED: announced %s is older than %s", text, cur);
        upgradePending = false;
        publishOtaResult("downgrade_rejected", text);
    }
}

// ─── MQTT helpers (used only by the Network task) ────────────────────────
bool mqttConnect() {
    LOG_INFO("MQTT connecting to %s:%d as %s ...", MQTT_HOST, MQTT_PORT, clientId);

    // Last Will: if we vanish, the broker publishes "offline" for us (retained)
    bool ok = mqtt.connect(clientId, MQTT_USER, MQTT_PASS,
                           TOPIC_STATUS, 1, true, "offline");
    if (!ok) {
        LOG_ERROR("MQTT connect failed, state=%d", mqtt.state());
        return false;
    }

    // Birth message: we are online (retained, so new viewers see it at once)
    mqtt.publish(TOPIC_STATUS, "online", true);

    // Device info (retained)
    char info[200];
    snprintf(info, sizeof(info),
             "{\"fw\":\"%d.%d.%d\",\"fw_num\":%lu,\"chip\":\"%s\",\"idf\":\"%s\",\"boot\":%lu}",
             FW_MAJOR, FW_MINOR, FW_PATCH, FW_VERSION,
             ESP.getChipModel(), esp_get_idf_version(), bootCount);
    mqtt.publish(TOPIC_INFO, info, true);

    // Listen for the fleet version (retained, so we get it right after subscribing)
    if (mqtt.subscribe(TOPIC_FLEET, 1)) LOG_INFO("Subscribed to %s", TOPIC_FLEET);
    else                                LOG_ERROR("Subscribe failed: %s", TOPIC_FLEET);

    LOG_INFO("MQTT connected (TLS). Published status=online and device info");
    return true;
}

void publishTelemetry(const SensorData_t& d) {
    char payload[200];
    snprintf(payload, sizeof(payload),
             "{\"temp\":%.1f,\"hum\":%.0f,\"valid\":%s,\"alarm\":%s,\"boot\":%lu,\"uptime_s\":%lu,\"fw\":\"%d.%d.%d\"}",
             d.temperature, d.humidity,
             d.valid ? "true" : "false", d.alarm ? "true" : "false",
             bootCount, millis() / 1000, FW_MAJOR, FW_MINOR, FW_PATCH);

    if (mqtt.publish(TOPIC_TELEMETRY, payload)) LOG_INFO("Published %s", TOPIC_TELEMETRY);
    else                                       LOG_WARN("Publish failed: %s", TOPIC_TELEMETRY);
}

// ─── OTA (ArduinoOTA) ────────────────────────────────────────────────────
// Set up once. The window is opened with ArduinoOTA.begin() only after an
// upgrade is accepted, and closed again with ArduinoOTA.end() after 2 minutes.
void otaSetup() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);     // shows in Arduino IDE as a network port
    ArduinoOTA.setPassword(OTA_PASS);         // IDE asks for this before uploading

    ArduinoOTA.onStart([]() {
        otaRunning = true;
        otaPercent = 0;
        LOG_INFO("OTA upload STARTED");
        mqtt.publish(TOPIC_OTA, "{\"result\":\"ota_started\"}");
    });
    ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
        uint8_t p = (uint8_t)(done * 100 / total);
        if (p / 10 != otaPercent / 10) LOG_INFO("OTA progress %u%%", p);   // log every 10 %
        otaPercent = p;
    });
    ArduinoOTA.onEnd([]() {
        LOG_INFO("OTA upload FINISHED - rebooting into the new firmware");
        mqtt.publish(TOPIC_OTA, "{\"result\":\"ota_success_rebooting\"}");
        mqtt.disconnect();                    // clean goodbye before reboot
    });
    ArduinoOTA.onError([](ota_error_t err) {
        otaRunning = false;
        const char* why = err == OTA_AUTH_ERROR    ? "auth failed (wrong OTA password)" :
                          err == OTA_BEGIN_ERROR   ? "begin failed (sketch too big?)"    :
                          err == OTA_CONNECT_ERROR ? "connect failed"                    :
                          err == OTA_RECEIVE_ERROR ? "receive failed"                    :
                          err == OTA_END_ERROR     ? "end failed"                        : "unknown";
        LOG_ERROR("OTA error: %s", why);
        char msg[96];
        snprintf(msg, sizeof(msg), "{\"result\":\"ota_error\",\"reason\":\"%s\"}", why);
        mqtt.publish(TOPIC_OTA, msg);
    });
}

void goToSleep() {
    LOG_INFO("Going to deep sleep for %d s (boot #%lu)", SLEEP_SECONDS, bootCount);
    if (mqtt.connected()) {
        mqtt.publish(TOPIC_STATUS, "sleeping", true);   // clean exit, so the Last Will does not fire
        mqtt.loop();
        delay(100);
        mqtt.disconnect();
    }
    if (oledOk) { oled.clearDisplay(); oled.display(); }
    setColor(0, 0, 0);
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
    esp_deep_sleep_start();
}

// ═════════════════════════════════════════════════════════════════════════
// TASK 4: NETWORK  (core 0, priority 2)
// WiFi + MQTT live here so a slow network never blocks the core-1 tasks.
// ═════════════════════════════════════════════════════════════════════════
void networkTask(void* param) {
    LOG_INFO("NetworkTask started (priority %d)", uxTaskPriorityGet(NULL));

    // ---- WiFiManager: connect with saved WiFi, or open setup hotspot ----
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);           // hotspot stays open 3 minutes

    if (digitalRead(BOOT_BTN) == LOW) {       // BOOT held at start = forget WiFi
        LOG_WARN("BOOT button held: erasing saved WiFi");
        wm.resetSettings();
    }

    while (!wm.autoConnect("FleetForge-Setup")) {
        LOG_WARN("WiFi not configured. Join hotspot 'FleetForge-Setup' with your phone");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    wifiOk = true;
    LOG_INFO("WiFi connected: %s | IP %s | RSSI %d dBm",
             WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());

    // ---- MQTT over TLS ----
    tlsClient.setCACert(EMQX_CA_CERT);        // verify we really talk to EMQX
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(onMqttMessage);
    mqtt.setBufferSize(512);
    mqtt.setKeepAlive(30);

    otaSetup();
    uint32_t otaOpenedAt   = 0;
    uint32_t mqttUpAt = 0;     // when MQTT connected this wake
    bool     sentOnce = false; // telemetry already sent this wake

    uint32_t lastPublish = 0;
    uint32_t lastAttempt = 0;
    bool     lastAlarm   = false;
    SensorData_t latest  = {};
    bool     haveData    = false;

    while (true) {
        // WiFi watchdog
        wifiOk = (WiFi.status() == WL_CONNECTED);
        if (!wifiOk) {
            mqttOk = false;
            LOG_WARN("WiFi lost, reconnecting...");
            WiFi.reconnect();
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // MQTT (re)connect, at most once every 5 s
        if (!mqtt.connected()) {
            mqttOk = false;
            if (millis() - lastAttempt > 5000) {
                lastAttempt = millis();
                mqttOk = mqttConnect();
                if (mqttOk) mqttUpAt = millis();
            }
        } else {
            mqttOk = true;
            mqtt.loop();                      // keeps the connection alive
        }

        // Take the newest reading if there is one (don't wait)
        if (xQueueReceive(mqttQueue, &latest, 0) == pdTRUE) haveData = true;

        if (mqttOk && haveData) {
            // Alert immediately when the alarm state changes
            if (latest.valid && latest.alarm != lastAlarm) {
                char msg[96];
                snprintf(msg, sizeof(msg), "{\"alarm\":%s,\"temp\":%.1f,\"threshold\":%.1f}",
                         latest.alarm ? "true" : "false", latest.temperature, TEMP_ALARM_C);
                mqtt.publish(TOPIC_ALERT, msg);
                if (latest.alarm) LOG_WARN("ALERT published: alarm started");
                else              LOG_INFO("ALERT published: alarm cleared");
                lastAlarm = latest.alarm;
            }

            // Send right after wake, then every 10 s
            if (!sentOnce || millis() - lastPublish >= PUBLISH_EVERY_MS) {
                lastPublish = millis();
                publishTelemetry(latest);
                sentOnce = true;
            }
        }

        // ---- OTA window ----
        // Open it once for each newly accepted version
        if (upgradePending && !otaWindowOpen && targetVersion != otaHandledFor) {
            otaHandledFor = targetVersion;
            ArduinoOTA.begin();
            otaWindowOpen = true;
            otaOpenedAt   = millis();
            char t[12];
            versionToText(targetVersion, t, sizeof(t));
            LOG_INFO("OTA WINDOW OPEN for %d s | host %s.local | IP %s | waiting for v%s",
                     OTA_WINDOW_MS / 1000, OTA_HOSTNAME, WiFi.localIP().toString().c_str(), t);
            mqtt.publish(TOPIC_OTA, "{\"result\":\"ota_window_open\"}");
        }

        if (otaWindowOpen) {
            ArduinoOTA.handle();              // receives the upload (blocks while uploading)
            uint32_t elapsed = millis() - otaOpenedAt;
            otaWindowLeftS = elapsed < OTA_WINDOW_MS ? (OTA_WINDOW_MS - elapsed) / 1000 : 0;

            if (!otaRunning && elapsed >= OTA_WINDOW_MS) {
                ArduinoOTA.end();
                otaWindowOpen = false;
                LOG_WARN("OTA window closed: no upload received");
                mqtt.publish(TOPIC_OTA, "{\"result\":\"ota_window_closed\"}");
            }
        }

#ifndef DEMO_NO_SLEEP
        bool otaBusy = otaWindowOpen || otaRunning ||
                       (upgradePending && otaHandledFor != targetVersion);   // window about to open
        bool fleetChecked = mqttOk && (millis() - mqttUpAt > FLEET_WAIT_MS);

        static uint32_t lastDbg = 0;                                         // debug: why not sleeping?
        if (millis() - lastDbg > 5000) {
            lastDbg = millis();
            LOG_INFO("sleep check: sent=%d mqtt=%d fleetWait=%d busy=%d",
                     sentOnce, mqttOk, (int)(millis() - mqttUpAt > FLEET_WAIT_MS), otaBusy);
        }

        if (sentOnce && fleetChecked && !otaBusy) goToSleep();
        if (!mqttOk && !otaBusy && millis() > AWAKE_MAX_MS) goToSleep();     // failsafe
#endif

        vTaskDelay(pdMS_TO_TICKS(otaWindowOpen ? 10 : 50));
    }
}

// ═════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    bootCount++;

    logMutex = xSemaphoreCreateMutex();

    LOG_INFO("===== FleetForge Node starting =====");
    LOG_INFO("Firmware v%d.%d.%d (0x%06lX) | Boot #%lu",
             FW_MAJOR, FW_MINOR, FW_PATCH, FW_VERSION, bootCount);
    LOG_INFO("Chip: %s | IDF: %s", ESP.getChipModel(), esp_get_idf_version());

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(BOOT_BTN, INPUT_PULLUP);
    setColor(0, 0, 1);

    Wire.begin(OLED_SDA, OLED_SCL);
    if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        oledOk = true;
        oled.setTextColor(SSD1306_WHITE);
        LOG_INFO("OLED ready");
    } else {
        LOG_ERROR("OLED not found at 0x3C");
    }

    dht.begin();

    uint64_t mac = ESP.getEfuseMac();
    snprintf(clientId, sizeof(clientId), "fleetforge-%04X%08lX",
             (uint16_t)(mac >> 32), (uint32_t)mac);

    alarmQueue   = xQueueCreate(1, sizeof(SensorData_t));
    displayQueue = xQueueCreate(1, sizeof(SensorData_t));
    mqttQueue    = xQueueCreate(1, sizeof(SensorData_t));

    xTaskCreatePinnedToCore(sensorTask,  "SensorTask",   4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(alarmTask,   "AlarmTask",    4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(displayTask, "DisplayTask",  4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(networkTask, "NetworkTask", 10240, NULL, 2, NULL, 0);

    LOG_INFO("All 4 tasks created");
}

void loop() {
    vTaskDelete(NULL);
}