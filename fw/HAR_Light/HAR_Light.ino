/*
   HAR v5.1 (RP2040 Light Version)

   Main program to be run on the RP2040 Feather version of HABET's High Altitude Reporter (HAR).
   Handles intake of GPS and sensor data and outputs over 915 MHz LoRa module to
   ground station, as well as storing to onboard SD card. Requires Sparkfun NEO-M9N GPS board. 
   
   Created by Matthew E. Nelson
   M2I HABET
   Date Created: Mar 4th, 2025
   Last Updated: March 4, 2025
*/
#include <Arduino.h>
//#include <RadioLib.h> //Click here to get the library:    https://jgromes.github.io/RadioLib/
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // Library found here: https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library 
#include <Zanshin_BME680.h>
#include <Wire.h>
#include <SPI.h>
//#include <SD.h>
#include <Adafruit_SleepyDog.h>
#include <RH_RF95.h>

// PIN_SPI_SS:  STM32, SAMD51, nRF, RP2040
/*
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
*/

SFE_UBLOX_GNSS GNSS;//
BME680_Class BME680;
///< Forward function declaration with default value for sea level
float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {
  /*!
  @brief     This converts a pressure measurement into a height in meters
  @details   The corrected sea-level pressure can be passed into the function if it is known,
             otherwise the standard atmospheric pressure of 1013.25hPa is used (see
             https://en.wikipedia.org/wiki/Atmospheric_pressure) for details.
  @param[in] press    Pressure reading from BME680
  @param[in] seaLevel Sea-Level pressure in millibars
  @return    floating point altitude in meters.
  */
  static float Altitude;
  Altitude =
      44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  // of method altitude()

#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17

// Change to 915.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int ledPin = 13;      // select the pin for the LED
int battPin = A0;

// Initializes radio, serial, GPS, and I2C bus:
void setup() {
  // Status light:
  pinMode(ledPin, OUTPUT);

  // Serial: REMOVE WHILE STATEMENT FOR DEPLOYMENT
  Serial.begin(115200);
  //while (!Serial) delay(1);
  delay(100);

  // I2C:
  Wire.begin();
// BME680
  while (!BME680.begin(I2C_STANDARD_MODE)) {  // Start BME680 using I2C, use first device found
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }
  Serial.print(F("- Setting 16x oversampling for all sensors\n"));
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680.setIIRFilter(IIR4);  // Use enumerated type values
  Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "�C" symbols
  BME680.setGas(320, 150);  // 320�c for 150 milliseconds

  // Watchdog:
  Serial.println("Setting up WatchDog...");
  int countdownMS = Watchdog.enable(10000);
  Serial.print("Enabled the watchdog with max countdown of ");
  Serial.print(countdownMS, DEC);
  Serial.println(" milliseconds!");
  // GPS:
  Serial.print(F("[NEO-M9N] Initializing..."));
  //if (GNSS.begin() == false){
  //  Serial.println(F("u-blox GNSS not detected. Freezing."));
  //  while (1);  
  //}
  GNSS.begin();
  GNSS.setI2COutput(COM_TYPE_UBX); // Outputting UBX (U-blox binary protocol) only, no NMEA (National Marine Electronics Association)
  GNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g); // Sets dynamic model to AIRBORNE2g. Other options: PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE4g, WRIST, BIKE
  Serial.println(F("init success!"));
  delay(500);
  //SD Card Initialization:
  //Serial.print("Initializing SD card...");
  //if (!SD.begin(16)) {
  //  Serial.println("initialization failed!");
  //  while (1);
  //}
  //Serial.println("init success!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

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
long GPSSpeed = 0;
long GPSHeading = 0;
int GPSPDOP = 0;
int batt = 0;
float volt = 0.0;
int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop() {
  static int32_t  temp, humidity, pressure, gas;  // BME readings
  static float    alt;                            // Temporary variable
  // Grab GPS
  if (GNSS.getPVT()){
    GPSLat = GNSS.getLatitude(); // divide Lat/Lon by 1000000 to get coords
    GPSLon = GNSS.getLongitude();
    GPSAlt = GNSS.getAltitude(); // measures in mm. Divide by 1000 for alt in m
    // grab time from GPS - NOTE 10/4/23 - Not currently in use as requires changes to ground station tracker
    GPSHour = GNSS.getHour();
    GPSMinute = GNSS.getMinute();
    GPSSecond = GNSS.getSecond();

    // grab heading, ground speed, and dilution of precision data
    GPSHeading = GNSS.getHeading(); //measurement in degrees * 10^-5
    GPSSpeed = GNSS.getGroundSpeed(); // measurement in mm/s
    GPSPDOP = GNSS.getPDOP(); 
  }

  // get atmospheric data
  BME680.getSensorData(temp, humidity, pressure, gas);
  alt = altitude(pressure); 
  // Get Battery Data

  batt = analogRead(battPin);
  volt = (analogRead(battPin) * (3.3 / 4096)) * 3;
  
  // you can transmit C-string or Arduino string up to
  // 256 characters long

  char output[90];
  //uint8_t data[] = " ";
  sprintf(output, "$HAR, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %.2f", packetnum++,GPSLat, GPSLon, GPSAlt, GPSHeading, GPSSpeed, GPSPDOP, pressure, temp, humidity, gas,volt);
  //File file = SD.open("/HARdata.csv", FILE_APPEND);
  //file.print("$$HAR,");
  //file.print(GPSHour);
  //file.print(":");
  //file.print(GPSMinute);
  //file.print(":");
  //file.print(GPSSecond);
  //file.print(",");
  //file.print(GPSLat);
  //file.print(",");
  //file.print(GPSLon);
  //file.print(",");
  //file.print(GPSAlt);
  //file.print(",");
  //file.print(GPSHeading);
  //file.print(",");
  //file.print(GPSSpeed);
  //file.print(",");
  //file.print(GPSPDOP);
  //file.print(",");
  //file.print(pressure);
  //file.print(",");
  //file.print(temp);
  //file.print(",");
  //file.println(humidity);
  //file.close();
  Serial.println(output);
  //Serial.println(GPSPDOP);
  //int state = radio.transmit(output);
  digitalWrite(ledPin, HIGH);
  rf95.send((uint8_t *)output, sizeof(output));
  //Serial.print(F("[SX1276] Transmitting packet ... "));
  // clears RAM allocated to PVT processing - needs testing, might require re-initializing GPS every loop
  //GNSS.end();
  // reset watchdog timer
  Watchdog.reset();
  digitalWrite(ledPin, LOW);
  // wait for a second before transmitting again
  delay(1000);
}