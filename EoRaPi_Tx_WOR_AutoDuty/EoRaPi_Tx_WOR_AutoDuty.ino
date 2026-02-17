#define EoRa_PI_V1
#include <Arduino.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include "boards.h"
#include "utilities.h"
#include "radio_eora.h"
#include "time.h"
#include "Ticker.h"

#include "index7.h"

// ---------------------------
// Network
// ---------------------------
const char *ssid = "R2D2";
const char *password = "Sky7388500";

// ---------------------------
// Timestamp helper
// ---------------------------
uint32_t getTimestampSafe() {
  time_t now;
  if (time(&now)) return (uint32_t)now;
  return (uint32_t)(millis() / 1000);
}

bool initNTP() {
  configTzTime("EST5EDT,M3.2.0/2,M11.1.0/2",
               "pool.ntp.org",
               "time.nist.gov");

  struct tm info;
  unsigned long start = millis();

  while (!getLocalTime(&info)) {
    if (millis() - start > 5000) return false;
    delay(200);
  }
  return true;
}

AsyncWebServer server(80);

String linkAddress = "192.168.12.27:80";

// ---------------------------
// Command constants
// ---------------------------
#define CMD_1 1
#define CMD_2 2

// ---------------------------
// Message struct (must match RX)
// ---------------------------
struct __attribute__((packed)) Message {
  uint8_t command;
  char timestr[48];
};

// ---------------------------
// Local timestamp (Indianapolis, DST auto)
// ---------------------------
String getLocalTimestamp() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "NTP not ready";
    }

    char buf[48];
    strftime(buf, sizeof(buf), "%A %Y-%m-%d %I:%M:%S %p", &timeinfo);
    return String(buf);
}


bool sendWORCommand(uint8_t cmd) {
    

    // ⭐ 1. Send WOR preamble burst
    Serial.println("TX: Sending WOR preamble burst...");


    int state = radio.transmit("WOR", 3);   // sends long preamble (configured in initRadio)
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("TX: preamble transmit failed: %d\n", state);
        return false;
    }

    delay(50);  // small gap

    Message msg;
    msg.command = cmd;

    String ts = getLocalTimestamp();
    ts.toCharArray(msg.timestr, sizeof(msg.timestr));

    Serial.printf("TX: Sending WOR command %u at %s\n", cmd, msg.timestr);

    // ⭐ 2. Send actual message
    state = radio.transmit((uint8_t*)&msg, sizeof(Message));
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("TX: message transmit failed: %d\n", state);
        return false;
    }

    // ⭐ 3. Wait for ACK
    radio.startReceive();
    uint32_t start = millis();

    while (millis() - start < 1500) {
        uint8_t buf[4];
        int state = radio.readData(buf, sizeof(buf));

        if (state == RADIOLIB_ERR_NONE) {
            if (buf[0] == 'A' && buf[1] == 'C' && buf[2] == 'K') {
                Serial.println("TX: ACK received");
                return true;
            }
        }
        delay(10);
    }

    Serial.println("TX: No ACK");
    return false;
}

// ---------------------------
// State flags
// ---------------------------
volatile bool countdownExpired = false;
volatile bool sendRequested = false;

bool cameraIsOn = false;
bool worBusy = false;

Ticker onceTick;

volatile bool ackReceived = false;

void IRAM_ATTR onDio1() {
  ackReceived = true;
}

// ---------------------------
// HTML processor
// ---------------------------
String processor7(const String &var) {
  if (var == F("LINK"))
    return linkAddress;
  return String();
}

// ---------------------------
// WiFi
// ---------------------------
void wifi_Start() {
  IPAddress local_IP(192, 168, 12, 27);
  IPAddress gateway(192, 168, 12, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(192, 168, 12, 1);

  WiFi.config(local_IP, gateway, subnet, dns);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// ---------------------------
// Countdown trigger
// ---------------------------
void IRAM_ATTR countdownTrigger() {
  countdownExpired = true;
}

// ===============================
// SETUP
// ===============================
void setup() {
  initRadio();

  Serial.begin(115200);
  while(!Serial){
    delay(10);
  }

  Serial.println("\nTx Booting");

  wifi_Start();

  bool ok = initNTP();
  if (!ok) Serial.println("NTP failed, using fallback");

  configTzTime(
    "EST5EDT,M3.2.0/2,M11.1.0/2",   // Indianapolis timezone with DST
    "pool.ntp.org",
    "time.nist.gov"
  );

  // Web route
  server.on("/relay", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, PSTR("text/html"), HTML7, processor7);
    sendRequested = true;
    onceTick.once(120, countdownTrigger);  // 2 minutes
  });

  server.begin();

  pinMode(RADIO_DIO1_PIN, INPUT);

  initRadio();   // ⭐ includes long preamble (512) for WOR wake

  Serial.println("Tx Ready");
}

// ===============================
// LOOP — example ON/OFF cycle
// ===============================
void loop() {

  if (worBusy) {
    delay(10);
    return;
  }

  // ON request
  if (sendRequested && !cameraIsOn) {
    sendRequested = false;
    worBusy = true;

    Serial.println("\nWEB REQUEST RECEIVED → Battery ON");
     sendWORCommand(CMD_1);

    cameraIsOn = true;
    worBusy = false;
  }

  // OFF request
  if (countdownExpired) {
    countdownExpired = false;

    Serial.println("\nCOUNTDOWN Timer EXPIRED --Battery OFF");
     sendWORCommand(CMD_2);

    cameraIsOn = false;
    worBusy = false;
  }

  delay(10);
}

