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

// === STRUCT FOR COMMAND PACKET ===
struct __attribute__((packed)) Message {
  uint8_t type;     // FIRST field
  uint8_t command;  // SECOND field
  char timestr[48]; // THIRD field
};

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

// ---------------------------
// Local timestamp (Indianapolis, DST auto)
// ---------------------------
String getLocalTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "NTP not ready";
  }

  char buf[49];
  strftime(buf, sizeof(buf), "%A %Y-%m-%d %I:%M:%S %p", &timeinfo);
  return String(buf);
}

AsyncWebServer server(80);

String linkAddress = "192.168.12.27:80";

// ---------------------------
// HTML processor
// ---------------------------
String processor7(const String &var) {
  if (var == F("LINK"))
    return linkAddress;
  return String();
}

// ---------------------------
// Command constants
// ---------------------------
#define CMD_1 1
#define CMD_2 2

bool sendWORCommand(uint8_t cmd) {
  // 1. WOR wake packet
  Message wor;
  wor.type = 0xAA;
  wor.command = 0;
  memset(wor.timestr, 0, sizeof(wor.timestr));
  Serial.println("TX: Sending WOR wake packet");

  Serial.printf("TX: wor.type=0x%02X wor.command=%u\n", wor.type, wor.command);
  radio.transmit((uint8_t*)&wor, sizeof(Message));
  
  delay(2000);  // let Rx wake and get ready

  // 2. Real command packet
  Message msg;
  msg.type = 0xBB;
  msg.command = cmd;
  String ts = getLocalTimestamp();
  ts.toCharArray(msg.timestr, sizeof(msg.timestr));

  Serial.printf("TX: Sending WOR command %u at %s\n", cmd, msg.timestr);

  Serial.printf("TX: msg.type=0x%02X msg.command=%u\n", msg.type, msg.command);
  radio.transmit((uint8_t*)&msg, sizeof(Message));  

  // 3. Listen for ACK
  radio.setDio1Action(onDio1);
  ackReceived = false;
  radio.startReceive();
  uint32_t start = millis();

  while (millis() - start < 8000) {
    if (ackReceived) {
      ackReceived = false;
      Message ack;
      int st = radio.readData((uint8_t*)&ack, sizeof(Message));
      if (st == RADIOLIB_ERR_NONE && ack.type == 0xFF) {
        Serial.println("TX: ACK received");
        return true;
      }
      radio.startReceive();
    }
    delay(10);
  }

  Serial.println("TX: No ACK");
  return false;
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

void IRAM_ATTR countdownTrigger() {
    countdownExpired = true;
    Serial.println("\nCOUNTDOWN Timer EXPIRED --Battery OFF");
    sendWORCommand(CMD_2);
    cameraIsOn = false;
}

// ===============================
// SETUP
// ===============================
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("\nTx Booting");

  wifi_Start();

  bool ok = initNTP();
  if (!ok) Serial.println("NTP failed, using fallback");

  configTzTime(
    "EST5EDT,M3.2.0/2,M11.1.0/2",
    "pool.ntp.org",
    "time.nist.gov");

  server.on("/relay", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send_P(200, PSTR("text/html"), HTML7, processor7);
      sendRequested = true;
      onceTick.once(120, countdownTrigger);
  });

  server.begin();

  pinMode(RADIO_DIO1_PIN, INPUT);

  initRadio();

  radio.setDio1Action(onDio1);

  Serial.println("Tx Ready");
}

void loop() {
  if (worBusy) {
    delay(10);
    return;
  }

  if (sendRequested) {
    Serial.printf("DEBUG: sendRequested=%d cameraIsOn=%d\n", sendRequested, cameraIsOn);
  }

  if (sendRequested && !cameraIsOn) {
    sendRequested = false;
    worBusy = true;
    Serial.println("\nWEB REQUEST RECEIVED → Battery ON");
    sendWORCommand(CMD_1);
    cameraIsOn = true;
    worBusy = false;
  }

  if (countdownExpired) {
    countdownExpired = false;
    worBusy = true;
    Serial.println("\nCOUNTDOWN Timer EXPIRED --Battery OFF");
    sendWORCommand(CMD_2);
    cameraIsOn = false;
    worBusy = false;
  }

  delay(10);
}
