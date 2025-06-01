#include <WiFi.h>
#include <U8g2lib.h>
#include "secrets.h"   // <-- contains WIFI_SSID and WIFI_PASSWORD

// ==== 1) OLED SETUP (SSD1306 via 4-wire SW-SPI) ====
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(
    U8G2_R0,
    /* clock=*/ 18,
    /* data=*/  23,
    /* cs=*/    5,
    /* dc=*/    4,
    /* reset=*/ 2
);

// ==== 2) HARDWARE PINS ====
static const int RED_LED_PIN = 15;
static const int BUZZER_PIN  = 21;

// ==== 3) Wi-Fi & TCP-SERVER CONFIG ====
static const uint16_t TCP_PORT = 5000;
WiFiServer tcpServer(TCP_PORT);

// ==== 4) STATE FOR NON-BLOCKING “DOOR ALARM” (physical unauthorized open) ====
bool doorAlarmActive   = false;
unsigned long alarmStart = 0;
unsigned long lastToggle = 0;
bool ledOn               = false;

// ==== 5) AUTHORIZED CARD DOUBLE BEEP STATE ====
bool authorizedBeep = false;
unsigned long beepStart = 0;
int beepState = 0; // 0=idle, 1=first beep, 2=gap, 3=second beep, 4=done

// Forward-declare handlers:
void triggerUnauthorizedCard();
void triggerAuthorizedCard();
void triggerPhysicalAlarm();

void setup() {
  Serial.begin(115200);

  u8g2.begin();

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  tcpServer.begin();
  tcpServer.setNoDelay(true);
  Serial.printf("TCP server is listening on port %u\n", TCP_PORT);
}

void loop() {
  // 1) Check for incoming TCP client (Pi)
  WiFiClient client = tcpServer.available();
  if (client) {
    client.setTimeout(0.2);
    Serial.println("Client connected");
    String line = client.readStringUntil('\n');
    line.trim();
    Serial.printf("Received: [%s]\n", line.c_str());

    if (line.equalsIgnoreCase("UNAUTHORIZED_CARD")) {
      triggerUnauthorizedCard();
    }
    else if (line.equalsIgnoreCase("AUTHORIZED_CARD")) {
      triggerAuthorizedCard();
    }
    else if (line.equalsIgnoreCase("PHYSICAL_ALARM")) {
      triggerPhysicalAlarm();
    }
    else {
      Serial.println("→ Unknown command (ignoring).");
    }
    client.stop();
    Serial.println("Client disconnected");
  }

  // 2) Handle non-blocking PHYSICAL ALARM (5s blinking)
  if (doorAlarmActive) {
    unsigned long now = millis();
    if (now - alarmStart >= 5000UL) {
      doorAlarmActive = false;
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      Serial.println("Physical alarm finished.");
    } else {
      if (now - lastToggle >= 500UL) {
        lastToggle = now;
        ledOn = !ledOn;
        digitalWrite(RED_LED_PIN, ledOn ? HIGH : LOW);
        digitalWrite(BUZZER_PIN, ledOn ? HIGH : LOW);
      }
    }
  }

  // 3) Handle non-blocking AUTHORIZED double beep
  if (authorizedBeep) {
    unsigned long now = millis();
    switch (beepState) {
      case 1:
        digitalWrite(BUZZER_PIN, HIGH);
        if (now - beepStart >= 100) {
          digitalWrite(BUZZER_PIN, LOW);
          beepStart = now;
          beepState = 2;
        }
        break;
      case 2:
        if (now - beepStart >= 80) {
          beepStart = now;
          beepState = 3;
          digitalWrite(BUZZER_PIN, HIGH);
        }
        break;
      case 3:
        if (now - beepStart >= 100) {
          digitalWrite(BUZZER_PIN, LOW);
          beepState = 4;
          authorizedBeep = false;
        }
        break;
      default:
        digitalWrite(BUZZER_PIN, LOW);
        authorizedBeep = false;
        beepState = 0;
        break;
    }
  }

  delay(2);
}

// ───────────────────────────────────────────────────────────────
// Called when UNAUTHORIZED_CARD command arrives:
//   • Beep once, flash red LED once, show "Unauthorized" (small)
// ───────────────────────────────────────────────────────────────
void triggerUnauthorizedCard() {
  Serial.println("** UNAUTHORIZED CARD **");

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB10_tr);
    u8g2.drawStr(10, 38, "Unauthorized");
  } while (u8g2.nextPage());

  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(120);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

// ───────────────────────────────────────────────────────────────
// Called when AUTHORIZED_CARD command arrives:
//   • Two fast beep-beeps, show "Alarm" on OLED
// ───────────────────────────────────────────────────────────────
void triggerAuthorizedCard() {
  Serial.println("** AUTHORIZED CARD **");

  // Start non-blocking double beep
  authorizedBeep = true;
  beepStart = millis();
  beepState = 1;

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(15, 40, "Welcome");
  } while (u8g2.nextPage());
}

// ───────────────────────────────────────────────────────────────
// Called when PHYSICAL_ALARM command arrives:
//   • Start a non-blocking 5 second blink/beep alarm, show "Alarm"
// ───────────────────────────────────────────────────────────────
void triggerPhysicalAlarm() {
  Serial.println("** PHYSICAL ALARM (door forced) **");

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(45, 40, "Alarm");
  } while (u8g2.nextPage());

  doorAlarmActive = true;
  alarmStart = millis();
  lastToggle = millis();
  ledOn = false;
  ledOn = true;
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
}