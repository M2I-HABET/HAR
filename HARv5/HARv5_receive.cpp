/*
   HAR 5.1 Ground Station Receiver Code


   Created by Nick Goeckner and Brandon Beaver
   Based on example code by Wes Furuya
   M2I HABET
   Date Created: July 13, 2023
   Last Updated: June 17, 2025

   
*/

#include <Arduino.h>
#include <RH_RF95.h>
#include <SPI.h>


// Pin definitions for SparkFun 1W LoRa Function Board (Slot A)
int pin_cs =        5;
int pin_dio0 =      D0;
int pin_tx_enable = PWM0;
int pin_rx_enable = G0;
int pin_nrst =      G1;
int pin_dio1 =      G2; 

#define RF95_FREQ    915.0 // Frequency (for US)

RH_RF95 rf95(pin_cs, pin_dio0);

void setup() {

  Serial.begin(115200);
    while (!Serial); // Wait for Serial Monitor (esp32 waits max 2 sec)

  Serial.println("Starting receiver...");

  pinMode(pin_tx_enable, OUTPUT);
  pinMode(pin_rx_enable, OUTPUT);
  pinMode(pin_nrst, OUTPUT);
  digitalWrite(pin_tx_enable, LOW);
  digitalWrite(pin_rx_enable, HIGH);  // Start in RX mode

  // Reset LoRa
  digitalWrite(pin_nrst, LOW);
  delay(10);
  digitalWrite(pin_nrst, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    delay(1000);
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    delay(1000);
    while (1);
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
  rf95.setTxPower(23, true);
  rf95.setSpreadingFactor(9); // SF7..12
  rf95.setSignalBandwidth(62500); // 62.5kHz
  rf95.setCodingRate4(5); // 4/5
  Serial.println("Setup complete.");
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      Serial.print("Received [");
      Serial.print(len);
      Serial.print(" bytes]: ");
      for (uint8_t i = 0; i < len; i++) {
        Serial.write(buf[i]);
      }
      Serial.println();
      Serial.print("RSSI: ");
      Serial.print(rf95.lastRssi());
      Serial.println(" dBm");
    } else {
      Serial.println("Receive failed");
    }
  }
  delay(1000);
}
