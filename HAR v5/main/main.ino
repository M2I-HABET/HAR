/*
   HAR v5.0 (MicroMod)

   Main program to be run on version 5.0 of HABET's High Altitude Reporter (HAR).
   Handles intake of GPS and sensor data and outputs over 915 MHz LoRa module to
   ground station. Requires MicroMod GNSS Function Board (Function One), 1W LoRa MicroMod Function
   Board (Function Zero), and a MicroMod ESP32 Processor. Future updates will include data storage on
   onboard microSD card and use of ESP32's built-in WiFi transceiver to communicate with
   other onboard devices.

   *IMPORTANT NOTE*: Do not plug in the GNSS Function Board on the Function One slot until the board is
   programmed. The ESP32 will not be able to talk to its attached flash chip which will result in a fatal
   error. This is because the Function One slot uses the GPIO pins 6-11, which the ESP32 requires to be
   open.

   Created by Nick Goeckner and Brandon Beavers
   M2I HABET
   Date Created: July 13, 2023
   Last Updated: October 4, 2023
*/
#include <Arduino.h>
#include <RadioLib.h> //Click here to get the library:    https://jgromes.github.io/RadioLib/
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // Library found here: https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library 
#include <Zanshin_BME680.h>
#include <SparkFun_SDP3x_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_SDP3x
#include <ICM_20948.h>
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

SFE_UBLOX_GNSS GNSS;//
BME680_Class BME680;
ICM_20948_I2C icm20948;
SDP3X SDP31;
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

// Initializes radio, serial, GPS, and I2C bus:
void setup() {
  // Serial:
  Serial.begin(115200);
  // I2C:
  Wire.begin();
  // GPS:
  Serial.print(F("[NEO-M9N] Initializing..."));
  GNSS.begin();
  GNSS.setI2COutput(COM_TYPE_UBX); // Outputting UBX (U-blox binary protocol) only, no NMEA (National Marine Electronics Association)
  GNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g); // Sets dynamic model to AIRBORNE2g. Other options: PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE4g, WRIST, BIKE
  Serial.println(F("init success!"));
  delay(500);
  // ICM-20948:
  icm20948.begin(Wire,1);
  // SPD31 (Differential Pressure Sensor):
  SDP31.begin();
  // Radio: 
  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin(915.0); //-23dBm
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("init success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  radio.setOutputPower(30); // sets output power to 30 dBm - needs testing (may overheat!)
  // BME-680:
  while (!BME680.begin(I2C_STANDARD_MODE)) {  // Start BME680 using I2C, use first device found
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  // of loop until device is located
  Serial.print(F("- Setting 16x oversampling for all sensors\n"));
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680.setIIRFilter(IIR4);  // Use enumerated type values
  Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "�C" symbols
  BME680.setGas(320, 150);  // 320�c for 150 milliseconds

  radio.setRfSwitchPins(pin_rx_enable, pin_tx_enable);
  delay(100);
}
// initialize data variables here:
// Packet counter:
int counter = 0;
// GPS:
long GPSLat = 0;
long GPSLon = 0;
long GPSAlt = 0;
long GPSHour = 0;
long GPSMinute = 0;
long GPSSecond = 0;
// BME 680:
int temp = 0;
int pressure = 0;
int humidity = 0;
int gas = 0;
// SPD31:
float diffPressure = 0;
float diffTemp = 0;
void loop() {

  GPSLat = GNSS.getLatitude(); // divide Lat/Lon by 1000000 to get coords
  //int GPSLatInt = GPSLat/10000000.0;
  //float GPSLatFrac = (GPSLat/10000000.0) - GPSLatInt;
  //long GPSLatDec = trunc(GPSLatFrac*10000000.0);

  GPSLon = GNSS.getLongitude();
  //int GPSLonInt = GPSLon/10000000.0;
  //float GPSLonFrac = ((GPSLon/10000000.0) - GPSLonInt)*-1.0;
  //long GPSLonDec = trunc(GPSLonFrac*10000000.0);

  GPSAlt = GNSS.getAltitude(); // measures in mm. Divide by 1000 for alt in m
  //int GPSAltInt = GPSAlt/1000.0;
  //float GPSAltFrac = (GPSAlt/1000.0) - GPSAltInt;
  //long GPSAltDec = trunc(GPSAltFrac*1000.0);

  // grab time from GPS - NOTE 10/4/23 - Not currently in use as requires changes to ground station tracker
  GPSHour = GNSS.getHour();
  GPSMinute = GNSS.getMinute();
  GPSSecond = GNSS.getSecond();

  //get atmospheric data
  BME680.getSensorData(temp, humidity, pressure, gas);
  
  //get differential pressure - for pressure difference between balloon/atmosphere
  SDP31.triggeredMeasurement();
  delay(45);
  SDP31.readMeasurement(&diffPressure, &diffTemp);
  long diffPressureInt = diffPressure;
  // you can transmit C-string or Arduino string up to
  // 256 characters long

  char output[256];
  sprintf(output, "$$HAR, %d, %d, %d, %d, %d, %d, %d", GPSLat, GPSLon, GPSAlt, pressure, temp, humidity, diffPressureInt);
  Serial.println(output);
  int state = radio.transmit(output);

  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F(" success!"));

    // print measured data rate
    Serial.print(F("[SX1276] Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));
    // print GPS data to serial (for debugging)
    Serial.print(F("[NEO-M9N] Latitude:\t"));
    Serial.println(GNSS.getLatitude());
    Serial.print(F("[NEO-M9N] Longitude:\t"));
    Serial.println(GNSS.getLongitude());
    Serial.print(F("[NEO-M9N] Altitude:\t"));
    Serial.println(GNSS.getAltitude());
    // print battery voltage

    // print sensor data to serial

    Serial.print(F("[SX1276] Transmitting packet ... "));
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