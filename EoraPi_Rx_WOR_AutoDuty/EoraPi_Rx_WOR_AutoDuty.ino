/*
 * ============================================================
 *  Project:  EoRa-S3-900TB WOR AutoDuty Camera Control
 *  File:     EoraPi_Tx_WOR_AutoDuty.ino  /  EoraPi_Rx_WOR_AutoDuty.ino
 * ============================================================
 *
 *  Author:   William Lucid, AB9NQ (Tech500)
 *  Hardware: Ebyte EoRa-S3-900TB (SX1262, ESP32-S3)
 *  Library:  RadioLib v7.5.0 by Jan Gromes
 *
 *  Description:
 *    LoRa WOR (Wake-On-Radio) duty cycle based camera power
 *    control system. TX sends a wake packet followed by a
 *    command packet. RX wakes from deep sleep on WOR interrupt,
 *    receives the command, controls camera power via GPIO,
 *    and sends an ACK back to TX.
 *
 *  Features:
 *    - WOR duty cycle deep sleep for low power operation
 *    - Struct-based messaging with type identification
 *      (0xAA = wake, 0xBB = command, 0xFF = ACK)
 *    - Interrupt-driven ACK handshake via radio.setDio1Action()
 *    - NTP timestamp delivery with Indianapolis DST support
 *    - AsyncWebServer trigger for remote camera ON/OFF control
 *    - Ticker-based 120 second auto-off countdown
 *
 *  Development Assistance:
 *    Claude AI (Anthropic - claude.ai)
 *    Significant debugging and design assistance provided by
 *    Claude AI throughout development, including:
 *      - Interrupt handling via radio.setDio1Action()
 *      - WOR timing and deep sleep synchronization
 *      - Struct-based packet type identification
 *      - ACK handshake state machine design
 *      - ISR safety and watchdog crash resolution
 *
 *  References:
 *    RadioLib Examples: https://github.com/jgromes/RadioLib
 *    Ebyte EoRa-S3 Examples:
 *    https://github.com/Tech500/Ebyte-EoRa-S3-900TB-RadioLib-Examples
 *
 *  Date:     19 February 2026
 * ============================================================
 */


#define EoRa_PI_V1
#include <Arduino.h>
#include <RadioLib.h>
#include "boards.h"
#include "utilities.h"
#include "radio_eora.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"

volatile bool packetDone = false;

void packetHandler() {
  packetDone = true;
}

// ---------------------------
// Message struct (must match TX)
// ---------------------------
struct __attribute__((packed)) Message {
  uint8_t type;      // FIRST field
  uint8_t command;   // SECOND field
  char timestr[48];  // THIRD field
};

// ---------------------------
// Function prototypes
// ---------------------------
void enterDeepSleep();
void handleCommand(uint8_t cmd, const char* ts);
void sendAck();
void print_reset_reason(int reason);

// ---------------------------
// Struct-based ACK
// ---------------------------
void sendAck() {
  Message ack;
  ack.type = 0xFF;     // ACK marker
  ack.command = 0xFF;  // ACK marker
  memset(ack.timestr, 0, sizeof(ack.timestr));

  radio.transmit((uint8_t*)&ack, sizeof(Message));
  Serial.println("RX: ACK sent");
}

// ---------------------------
void handleCommand(uint8_t command, const char* timeStr) {
  Serial.printf("RX: Command %u at %s\n", command, timeStr);

  if (command == 1) {
    Serial.println("RX: Turning camera ON");
    digitalWrite(CAM_PWR_PIN, HIGH);
  } else if (command == 2) {
    Serial.println("RX: Turning camera OFF");
    digitalWrite(CAM_PWR_PIN, LOW);
  } else {
    Serial.println("RX: Unknown command");
  }
}

// ---------------------------
// Deep sleep entry
// ---------------------------
void enterDeepSleep() {
  Serial.println("=== PREPARING FOR DEEP SLEEP ===");
  Serial.printf("DIO1 pin state before sleep: %d\n", digitalRead(RADIO_DIO1_PIN));
  Serial.printf("Wake pin (GPIO16) state before sleep: %d\n", digitalRead(WAKE_PIN));

  initRadio();
  radio.startReceiveDutyCycleAuto();  // WOR mode

  Serial.println("Configuring RTC GPIO and deep sleep wake-up...");

  // Match 512 behavior: DIO1 idle LOW, wake on RISING
  rtc_gpio_pulldown_en(WAKE_PIN);                // bias LOW
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_16, 1);  // wake when pin goes HIGH

  digitalWrite(BOARD_LED, LED_OFF);
  Serial.println("✅ Going to deep sleep now...");
  Serial.flush();

  SPI.end();

  gpio_deep_sleep_hold_dis();
  gpio_hold_dis(GPIO_NUM_16);

  esp_deep_sleep_start();
}

// ---------------------------
void print_reset_reason(int reason) {
  switch (reason) {
    case 1: Serial.println("POWERON_RESET"); break;
    case 3: Serial.println("SW_RESET"); break;
    case 4: Serial.println("OWDT_RESET"); break;
    case 5: Serial.println("DEEPSLEEP_RESET"); break;
    case 9: Serial.println("RTCWDT_SYS_RESET"); break;
    case 15: Serial.println("RTCWDT_BROWN_OUT_RESET"); break;
    case 16: Serial.println("RTCWDT_RTC_RESET"); break;
    default: Serial.println("UNKNOWN_RESET"); break;
  }
}

// ---------------------------
// SETUP
// ---------------------------
void setup() {

  Serial.begin(115200);
  delay(300);

  gpio_deep_sleep_hold_dis();
  gpio_hold_dis(GPIO_NUM_16);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.printf("Wake cause: %d\n", cause);

  if (cause != ESP_SLEEP_WAKEUP_EXT0) {
    // POWERON path
    Serial.println("Non-deepsleep reset → entering deep sleep");
    enterDeepSleep();
  }

  bool gotPacket = false;

  // ===============================
  // EXT0 WAKE → RECEIVE PACKET
  // ===============================
  if (cause == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woke from LoRa packet");
    Serial.println("DEEPSLEEP_RESET: Woke from EXT0 on GPIO16");

    initRadio();
    radio.setDio1Action(packetHandler);  // override setFlag with packetHandler
    radio.startReceive();
    

    uint32_t start = millis();
    uint8_t rxBuf[sizeof(Message)];  // ← declare here
    Message msg;

   while (millis() - start < 5000) {
      if (packetDone) {
        packetDone = false;
        int state = radio.readData(rxBuf, sizeof(Message));
        Serial.printf("RX: readData state=%d\n", state);

        if (state == RADIOLIB_ERR_NONE) {
          memcpy(&msg, rxBuf, sizeof(Message));
          Serial.printf("RX: msg.command=%u msg.timestr=%s\n", msg.command, msg.timestr);

          if (msg.type == 0xAA) {
            Serial.println("RX: Wake packet, waiting for command...");
            radio.startReceive();
            continue;
          }
          if (msg.type == 0xBB) {
            sendAck();
            delay(500);
            handleCommand(msg.command, msg.timestr);
            gotPacket = true;
            break;            
          }
        }
      }
      delay(10);
    }

    if (!gotPacket) {
      Serial.println("No valid packet after wake");
    }

    enterDeepSleep();  // ← inside EXT0 block
    return;            // ← prevents fallthrough
  }                    // ← closes EXT0 if block

  // catches any other reset cause
  enterDeepSleep();
}

void loop() {}
