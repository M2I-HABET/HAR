/*
   HAR 5.0 Ground Station Receiver Code


   Created by Nick Goeckner and Brandon Beavers
   Based on example code by Wes Furuya
   M2I HABET
   Date Created: July 13, 2023
   Last Updated: November 22, 2023

   
*/

#include <RadioLib.h> //Click here to get the library: http://librarymanager/All#RadioLib

// Redefine CS Pin Name
// SPI_CS0:     ESP32
// SS:          ESP32, nRF, RP2040
// SPI_CS:      Artemis
// PIN_SPI_SS:  STM32, SAMD51, nRF, RP2040

#ifndef PIN_SPI_SS
  // For Artemis
  #ifdef SS
    #define PIN_SPI_SS SPI_CS
  #endif
  // For ESP32
  #ifdef SPI_CS0
    #define PIN_SPI_SS SS
  #endif
#endif

// SX1276 pin connections:
//       | SLOT 0 | SLOT 1 |
//==========================
// cs    |   CS0  |   CS1  |
// dio0  |   D0   |   D1   |
// dio1  |   G2   |   G7   |
// dio2  |   G3   |   G8   |
// rst   |   G1   |   G6   |
// tx_en |  PWM0  |  PWM1  |
// rx_en |   G0   |   G5   |

    int pin_cs =        PIN_SPI_SS;
    int pin_dio0 =      D0;
    int pin_tx_enable = PWM0;
    int pin_rx_enable = G0;
    int pin_nrst =      G1;
    int pin_dio1 =      G2;
  
  SX1276 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);

void setup() {
  Serial.begin(115200);

  // initialize SX1276 with default settings
  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin(915.0);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  radio.setOutputPower(30);
  // some modules have an external RF switch
  // controlled via two pins (RX enable, TX enable)
  // to enable automatic control of the switch,
  // call the following method
  radio.setRfSwitchPins(pin_rx_enable, pin_tx_enable);
}
byte cmd1[] = {0x01};
byte cmd2[] = {0x02};
byte cmd3[] = {0x03};
byte cmd4[] = {0x04};
String var = "";
int transmissionState = RADIOLIB_ERR_NONE;
void loop() {
  // Transmitting:
  if(Serial.available() > 0){
    var = Serial.readString();
    var.trim();
    if(var == "cmd"){
      Serial.println("Select a command to send: ");
      Serial.println("1. Blinking Light");
      Serial.println("2. [CySat] Power Status Request");
      Serial.println("3. [CySat] Take Measurement Request");
      Serial.println("4. [CySat] SDR Values Request");
      delay(1000);
      int cmdchoice = Serial.parseInt();
      switch (cmdchoice) {
        case 1:
          Serial.print(F("[SX1276] Sending packet ... "));
          transmissionState = radio.transmit(cmd1, 1);
          if (transmissionState == RADIOLIB_ERR_NONE) {
            // packet was successfully sent
            Serial.println(F("transmission finished!"));
          } else {
            Serial.print(F("failed, code "));
            Serial.println(transmissionState);
          }
          var = "";
          delay(500);
          break;
        case 2:
          Serial.print(F("[SX1276] Sending packet ... "));
          transmissionState = radio.transmit(cmd2, 1);
          if (transmissionState == RADIOLIB_ERR_NONE) {
            // packet was successfully sent
            Serial.println(F("transmission finished!"));
          } else {
            Serial.print(F("failed, code "));
            Serial.println(transmissionState);
          }
          var = "";
          delay(500);
          break;
        case 3:
          Serial.print(F("[SX1276] Sending packet ... "));
          transmissionState = radio.transmit(cmd3, 1);
          if (transmissionState == RADIOLIB_ERR_NONE) {
            // packet was successfully sent
            Serial.println(F("transmission finished!"));
          } else {
            Serial.print(F("failed, code "));
            Serial.println(transmissionState);
          }
          var = "";
          delay(500);
          break;
        case 4:
          Serial.print(F("[SX1276] Sending packet ... "));
          transmissionState = radio.transmit(cmd4, 1);
          if (transmissionState == RADIOLIB_ERR_NONE) {
            // packet was successfully sent
            Serial.println(F("transmission finished!"));
          } else {
            Serial.print(F("failed, code "));
            Serial.println(transmissionState);
          }
          var = "";
          delay(500);
          break;
        default:
          Serial.println("Please choose a valid command.");
      }
    }
  }



  // Receiving:
  while (Serial.available() == 0){
    //Serial.print(F("[SX1276] Waiting for incoming transmission ... "));

    // you can receive data as an Arduino String
    // NOTE: receive() is a blocking method!
    //       See example ReceiveInterrupt for details
    //       on non-blocking reception method.
    String str;
    int state = radio.receive(str);

    // you can also receive data as byte array
    /*
    byte byteArr[8];
    int state = radio.receive(byteArr, 8);
    */

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      //Serial.println(F("success!"));

      // print the data of the packet
      //Serial.print(F("[SX1276] Data:\t\t\t"));
      Serial.println(str);

      // print the RSSI (Received Signal Strength Indicator)
      // of the last received packet
      //Serial.print(F("[SX1276] RSSI:\t\t\t"));
      //Serial.print(radio.getRSSI());
      //Serial.println(F(" dBm"));

      // print the SNR (Signal-to-Noise Ratio)
      // of the last received packet
      //Serial.print(F("[SX1276] SNR:\t\t\t"));
      //Serial.print(radio.getSNR());
      //Serial.println(F(" dB"));

      // print frequency error
      // of the last received packet
      //Serial.print(F("[SX1276] Frequency error:\t"));
      //Serial.print(radio.getFrequencyError());
      //Serial.println(F(" Hz"));

    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      // timeout occurred while waiting for a packet
      //Serial.println(F("timeout!"));

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      //Serial.println(F("CRC error!"));

    } else {
      // some other error occurred
      //Serial.print(F("failed, code "));
      //Serial.println(state);
    }
  }
}
