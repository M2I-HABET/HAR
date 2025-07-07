/*
   HAR v5.2 (MicroMod)

   Main program to be run on version 5.2 of HABET's High Altitude Reporter (HAR).
   Handles intake of GPS and sensor data and outputs over 915 MHz LoRa module to
   ground station, as well as storing to onboard SD card. Requires MicroMod GNSS Function Board (Function One), 
   1W LoRa MicroMod Function Board (Function Zero), and a MicroMod ESP32 Processor. Future updates will 
   include use of ESP32's built-in WiFi transceiver to communicate with other onboard devices.

   *IMPORTANT NOTE*: Do not plug in the GNSS Function Board on the Function One slot until the board is
   programmed. The ESP32 will not be able to talk to its attached flash chip which will result in a fatal
   error. This is because the Function One slot uses the GPIO pins 6-11, which the ESP32 requires to be
   open.

   Created by Nick Goeckner and Brandon Beaver
   M2I HABET
   Date Created: July 13, 2023
   Last Updated: July 7, 2025
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
  rf95.setCodingRate4(8); // 4/5
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
