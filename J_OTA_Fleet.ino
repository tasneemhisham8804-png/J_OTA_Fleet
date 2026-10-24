// testtt - checks every part of the Project J wiring
// Active buzzer + RGB LED version
// Open Serial Monitor at 115200 baud, then press EN/RST on the ESP32.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

#define DHT_PIN     4
#define BUZZER_PIN  25
#define BUZZER_ON   LOW
#define BUZZER_OFF  HIGH
#define LED_R       26
#define LED_G       27
#define LED_B       33
#define OLED_SDA    21
#define OLED_SCL    22

#define TEMP_ALARM  30.0  // above this: red LED + beep

DHT dht(DHT_PIN, DHT11);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;

void setColor(bool r, bool g, bool b) {
    digitalWrite(LED_R, r);
    digitalWrite(LED_G, g);
    digitalWrite(LED_B, b);
}

void beep(int ms) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(ms);
    digitalWrite(BUZZER_PIN, LOW);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n===== HARDWARE TEST v2 =====");

    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // ---- TEST 1: RGB LED ----
    Serial.println("[TEST 1] RGB LED: red -> green -> blue -> purple -> off");
    setColor(1, 0, 0); Serial.println("   red");    delay(1000);
    setColor(0, 1, 0); Serial.println("   green");  delay(1000);
    setColor(0, 0, 1); Serial.println("   blue");   delay(1000);
    setColor(1, 0, 1); Serial.println("   purple"); delay(1000);
    setColor(0, 0, 0); Serial.println("   off");

    // ---- TEST 2: ACTIVE BUZZER ----
    Serial.println("[TEST 2] Buzzer should beep 3 times...");
    for (int i = 0; i < 3; i++) {
        beep(200);
        delay(300);
    }

    // ---- TEST 3: I2C SCAN (finds the OLED) ----
    Serial.println("[TEST 3] Scanning I2C bus...");
    Wire.begin(OLED_SDA, OLED_SCL);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("   Found device at 0x%02X%s\n", addr,
                          addr == 0x3C ? "  <-- this is the OLED" : "");
            found++;
        }
    }
    if (found == 0) Serial.println("   [ERROR] Nothing found - check OLED wires");

    // ---- TEST 4: OLED ----
    Serial.println("[TEST 4] Starting OLED...");
    if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        oledOk = true;
        oled.clearDisplay();
        oled.setTextColor(SSD1306_WHITE);
        oled.setTextSize(2);
        oled.setCursor(0, 0);
        oled.println("OLED OK!");
        oled.display();
        Serial.println("   [OK] OLED shows 'OLED OK!'");
    } else {
        Serial.println("   [ERROR] OLED did not start");
    }

    // ---- TEST 5: DHT11 + alarm (runs forever in loop) ----
    dht.begin();
    Serial.printf("[TEST 5] Reading DHT11 every 2 s. Alarm above %.0f C\n", TEMP_ALARM);
    Serial.println("   Tip: hold the DHT11 in your fingers to warm it up.");
}

void loop() {
    delay(2000);
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (isnan(t) || isnan(h)) {
        setColor(0, 0, 1);  // blue = sensor problem
        Serial.println("   [ERROR] DHT11 read failed - check DATA wire on GPIO 4");
        return;
    }

    bool alarm = t > TEMP_ALARM;
    if (alarm) {
        setColor(1, 0, 0);  // red
        beep(150);
        Serial.printf("   [WARN] Temp: %.1f C | Humidity: %.0f %% | ALARM!\n", t, h);
    } else {
        setColor(0, 1, 0);  // green
        Serial.printf("   [OK] Temp: %.1f C | Humidity: %.0f %%\n", t, h);
    }

    if (oledOk) {
        oled.clearDisplay();
        oled.setTextSize(2);
        oled.setCursor(0, 0);
        oled.printf("T: %.1fC\n", t);
        oled.printf("H: %.0f%%\n", h);
        oled.setTextSize(1);
        oled.setCursor(0, 50);
        oled.print(alarm ? "!! ALARM !!" : "Status: normal");
        oled.display();
    }
}
