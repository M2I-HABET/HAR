/*
   HAR v5.0 (MicroMod)

   Created by Nick Goeckner and Brandon Beavers
   M2I HABET
   Date: July 2023
*/
#include <Arduino.h>
#include <RadioLib.h> //Click here to get the library:    https://jgromes.github.io/RadioLib/
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // Library found here: https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library 
#include <Wire.h>

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

SFE_UBLOX_GNSS GNSS;

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

//Initializes radio, serial, GPS, and I2C bus:
void setup() {
  //I2C:
  Wire.begin();
  //Serial:
  Serial.begin(115200);
  //GPS:
  GNSS.begin();
  GNSS.setI2COutput(COM_TYPE_UBX); //Outputting UBX only, no NMEA
  GNSS.setDynamicModel(DYN_MODEL_AIRBORNE1g); //Sets dynamic model to AIRBORNE1g. Other options: PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
  delay(500);
  //Radio: 
  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin(915.0); //-23dBm
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("init success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  

  // set output power to 10 dBm (accepted range is -3 - 17 dBm)
  // NOTE: 20 dBm value allows high power operation, but transmission
  //       duty cycle MUST NOT exceed 1%
  //  if (radio.setOutputPower(20) == ERR_INVALID_OUTPUT_POWER) {
  //    Serial.println(F("Selected output power is invalid for this module!"));
  //    while (true);
  //  }
  radio.setRfSwitchPins(pin_rx_enable, pin_tx_enable);
}

long GPSLat = 0;
long GPSLon = 0;
long GPSAlt = 0;

void loop() {
  GPSLat = GNSS.getLatitude(); //Divide Lat/Lon by 1000000 to get coords.
  GPSLon = GNSS.getLongitude();
  GPSAlt = GNSS.getAltitude(); //Measures in mm. Divide by 1000 for alt in m.
  Serial.print(F("[SX1276] Transmitting packet ... "));

  // you can transmit C-string or Arduino string up to
  // 256 characters long

  char output[100];
  sprintf(output, "$HAR: %d, %d, %d", GPSLat, GPSLon, GPSAlt);

  int state = radio.transmit(output);

  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F(" success!"));

    // print measured data rate
    Serial.print(F("[SX1276] Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));
    //print GPS data to serial(for debugging)
    Serial.print(F("[SX1276] Latitude:\t"));
    Serial.println(GNSS.getLatitude());
    Serial.print(F("[SX1276] Longitude:\t"));
    Serial.println(GNSS.getLongitude());
    Serial.print(F("[SX1276] Altitude:\t"));
    Serial.println(GNSS.getAltitude());
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));

  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occurred while transmitting packet
    Serial.println(F("timeout!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  // wait for a second before transmitting again
  delay(1000);
}
