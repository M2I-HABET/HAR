/*
   HAR 5.0 Ground Station Receiver Code

   Updated for Adafruit Feather ESP32 with RFM95 compatibility
   Created by Nick Goeckner and Brandon Beaver
   Based on example code by Wes Furuya
   M2I HABET
   Date Created: July 13, 2023
   Last Updated: March 13, 2025
*/

#include <RadioLib.h>

#ifndef PIN_SPI_SS
  #ifdef SS
    #define PIN_SPI_SS SPI_CS
  #endif
  #ifdef SPI_CS0
    #define PIN_SPI_SS SS
  #endif
#endif

// SX1276 pin connections for SparkFun ESP32 MicroMod
int pin_cs = PIN_SPI_SS;
int pin_dio0 = D0;
int pin_tx_enable = PWM0;
int pin_rx_enable = G0;
int pin_nrst = G1;
int pin_dio1 = G2;

SX1276 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);

void setup() {
  Serial.begin(115200);

  // Initialize SX1276 with default settings
  pinMode(pin_cs, OUTPUT);
  digitalWrite(pin_cs, HIGH);
  
  Serial.print(F("Testing SPI communication... "));
  Serial.print(F("Initializing SPI... "));
  SPI.begin();
  Serial.println(F("done."));
  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin(915.0);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  Serial.print(F("Reading LoRa version... "));
  int16_t rssi = radio.getRSSI();  // Get the RSSI value of the last received packet
  Serial.print(F("LoRa RSSI: "));
  Serial.println(rssi);
  
  radio.setOutputPower(30);
  radio.setSpreadingFactor(7);
  radio.setBandwidth(125.0);
  radio.setCodingRate(5);
  radio.setRfSwitchPins(pin_rx_enable, pin_tx_enable);
}

void loop() {
  // Wait for incoming transmission from Adafruit Feather ESP32
  String str;
  int state = radio.receive(str);

  if (state == RADIOLIB_ERR_NONE) {
    // Packet received successfully
    Serial.print("Received: ");
    Serial.print(str);
    Serial.print(", RSSI: ");
    Serial.println(radio.getRSSI());
  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // Timeout occurred while waiting for a packet
    Serial.println(F("Timeout waiting for transmission"));
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    // Packet received, but is malformed
    Serial.println(F("CRC Error: Packet is corrupted"));
  } else {
    // Some other error occurred
    Serial.print(F("Receive failed, code "));
    Serial.println(state);
  }
}
