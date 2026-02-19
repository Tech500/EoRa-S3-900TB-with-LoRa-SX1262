#define EoRa_PI_V1
#include <Arduino.h>
#include <RadioLib.h>
#include "radio_eora.h"
#include "utilities.h"

// Bring in ISR from .ino
extern void IRAM_ATTR onDio1();

// === CONFIGURATION ===
#define USING_SX1262_868M

#if defined(USING_SX1268_433M)
uint8_t txPower = 14;
float radioFreq = 433.0;
SX1268 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#elif defined(USING_SX1262_868M)
uint8_t txPower = 14;
float radioFreq = 915.0;
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif

#define BUFFER_SIZE 49  // Define the payload size here

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // check if the interrupt is enabled
  if (!enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag = true;
}

void initRadio(){
  initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);

    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);

    // initialize SX126x with default settings
    Serial.print(F("[SX126x] Initializing ... "));
    //int state = radio.begin(radioFreq);  //example
    int state = radio.begin(
    radioFreq,  // 915.0 MHz
    125.0,      // Bandwidth
    7,          // Spreading factor
    7,          // Coding rate
    RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
    14,   // 14 dBm for good balance
    512,   // Preamble length
    0.0,  // No TCXO
    true  // LDO mode ON
  );
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("success!"));
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true)
            ;
    }

    // set the function that will be called
    // when new packet is received
    radio.setDio1Action(setFlag);

    // start listening for LoRa packets
    Serial.print(F("[SX126x] Starting to listen ... "));
    // Set up the radio for duty cycle receiving
    state = radio.startReceiveDutyCycleAuto();
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("success!"));
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true)
            ;
    }

    // if needed, 'listen' mode can be disabled by calling
    // any of the following methods:
    //
    // radio.standby()
    // radio.sleep()
    // radio.transmit();
    // radio.receive();
    // radio.readData();
    // radio.scanChannel();
}