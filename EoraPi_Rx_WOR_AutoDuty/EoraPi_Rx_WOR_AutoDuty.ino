#define EoRa_PI_V1
#include <Arduino.h>
#include <RadioLib.h>
#include "boards.h"
#include "utilities.h"
#include "radio_eora.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"

// ---------------------------
// Message struct (must match TX)
// ---------------------------
struct __attribute__((packed)) Message {
  uint8_t command;
  char timestr[48];
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
  ack.command = 0xFF;   // ACK marker
  memset(ack.timestr, 0, sizeof(ack.timestr));

  radio.transmit((uint8_t*)&ack, sizeof(Message));
  Serial.println("RX: ACK sent");
}

// ---------------------------
void handleCommand(uint8_t cmd, const char* ts) {
  Serial.printf("RX: Command %u at %s\n", cmd, ts);

  if (cmd == 1) {
    Serial.println("RX: Turning camera ON");
    digitalWrite(CAM_PWR_PIN, HIGH);
  } else if (cmd == 2) {
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
    initBoard();
    delay(1500);

    Serial.begin(115200);
    delay(300);

    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(GPIO_NUM_16);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    Serial.printf("Wake cause: %d\n", cause);

    if (cause != ESP_SLEEP_WAKEUP_EXT0) {    
        // POWERON path
        Serial.println("Non-deepsleep reset → entering deep sleep");
        initRadio();
        radio.startReceiveDutyCycleAuto();
        enterDeepSleep();
    }

    // ===============================
    // EXT0 WAKE → RECEIVE PACKET
    // ===============================
    if (cause == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("Woke from LoRa packet");
        Serial.println("DEEPSLEEP_RESET: Woke from EXT0 on GPIO16");

        initRadio();
        radio.startReceive();

        uint32_t start = millis();
        bool gotPacket = false;

        while (millis() - start < 3000) {
            uint8_t rxBuf[sizeof(Message)];
            int state = radio.readData(rxBuf, sizeof(Message));

            if (state == RADIOLIB_ERR_NONE) {
                Message msg;
                memcpy(&msg, rxBuf, sizeof(Message));

                Serial.println("Packet received, sending ACK and handling command");

                sendAck();
                handleCommand(msg.command, msg.timestr);

                gotPacket = true;
                break;
            }
            delay(10);
        }

        if (!gotPacket) {
            Serial.println("No valid packet after wake");
        }

        enterDeepSleep();
        return;
    }

    // ===============================
    // POWERON / ANY OTHER RESET
    // ===============================
    Serial.println("POWERON → entering deep sleep immediately");
    enterDeepSleep();     // <-- MUST sleep here
    return;
}



void loop() {}
