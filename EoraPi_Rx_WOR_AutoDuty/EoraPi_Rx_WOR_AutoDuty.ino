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
void enterDeepSleep();
void handleCommand(uint8_t cmd, const char* ts);
void sendAck();
void print_reset_reason(int reason);

// ---------------------------
void sendAck() {
  const char ack[] = "ACK";
  radio.transmit((uint8_t*)ack, 3);
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

  // 1. Start WOR first
  radio.startReceiveDutyCycleAuto();

  // 2. Now read the pins
  Serial.printf("After WOR: DIO1=%d  WAKE_PIN=%d\n",
    digitalRead(RADIO_DIO1_PIN),
    digitalRead(WAKE_PIN));

  // 3. Wait until WAKE_PIN is HIGH (idle state)
  while (digitalRead(WAKE_PIN) == LOW) {
    delay(1);
  }

  // 4. Arm EXT0 for LOW wake
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_16, 0);

  digitalWrite(BOARD_LED, LED_OFF);
  Serial.println("Going to deep sleep...");
  Serial.flush();

  // 5. Sleep
  esp_deep_sleep_start();
}

// ---------------------------
void print_reset_reason(int reason) {
  switch (reason) {
    case 1:  Serial.println("POWERON_RESET"); break;
    case 3:  Serial.println("SW_RESET"); break;
    case 4:  Serial.println("OWDT_RESET"); break;
    case 5:  Serial.println("DEEPSLEEP_RESET"); break;
    case 9:  Serial.println("RTCWDT_SYS_RESET"); break;
    case 15: Serial.println("RTCWDT_BROWN_OUT_RESET"); break;
    case 16: Serial.println("RTCWDT_RTC_RESET"); break;
    default: Serial.println("UNKNOWN_RESET"); break;
  }
}

// ---------------------------
// SETUP
// ---------------------------
void setup() {
  initRadio();

  Serial.begin(115200);
  delay(200);
  Serial.println("\nRX boot");

  pinMode(WAKE_PIN, INPUT);
  pinMode(RADIO_DIO1_PIN, INPUT);
  pinMode(CAM_PWR_PIN, OUTPUT);
  digitalWrite(CAM_PWR_PIN, LOW);

   esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    Serial.printf("Wake cause: %d\n", cause);

    // SAFE BOOT: only EXT0 wake continues
    if (cause != ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("SAFE BOOT: Not an EXT0 wake. Staying awake.");
        return;   // Do NOT call initRadio()
    }

    Serial.printf("After WOR start: DIO1=%d WAKE_PIN=%d\n",
      digitalRead(RADIO_DIO1_PIN),
      digitalRead(WAKE_PIN));

  
  // ===============================
  // DEEP SLEEP WAKE → receive packet
  // ===============================
 if (cause == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woke from LoRa packet");

    Serial.println("DEEPSLEEP_RESET: Woke from EXT0 on GPIO16");

    radio.startReceive(); // full RX mode

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
  }

  // Any other reset → treat like POWERON
  Serial.println("Non-deepsleep reset → entering deep sleep");
  initRadio();
  radio.startReceiveDutyCycleAuto();
  enterDeepSleep();
}

void loop() {}

