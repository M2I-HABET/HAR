/*
   HAR v5.1 (MicroMod)

   Main program to be run on version 5.0 of HABET's High Altitude Reporter (HAR).
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
   Last Updated: June 17, 2025
*/
#include <Arduino.h>
#include <RH_RF95.h> //RadioHead library
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // Library found here: https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library 
#include <Zanshin_BME680.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_SleepyDog.h>

// Redefine CS Pin Name
// SPI_CS0:     ESP32
// SS:          ESP32, nRF, RP2040
// SPI_CS:      Artemis
// PIN_SPI_SS:  STM32, SAMD51, nRF, RP2040

#define RF95_FREQ      915.0

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

int pin_cs =        5;
int pin_dio0 =      D0;
int pin_tx_enable = PWM0;
int pin_rx_enable = G0;
int pin_nrst =      G1;
int pin_dio1 =      G2;
int ledPin =        2;
int battPin =       39;
const int DynModpin =2;
int i;

  
RH_RF95 rf95(pin_cs, pin_dio0);

// Initializes radio, serial, GPS, and I2C bus:
void setup() {
  // Status light:
  pinMode(ledPin, OUTPUT);
  pinMode(DynModpin, OUTPUT);
  // Serial:
  Serial.begin(115200);
  // I2C:
  Wire.begin();
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
  Serial.print("Dynamic model set: ");
  Serial.println(GNSS.getDynamicModel()); // Prints the dynamic model set to the serial monitor
  bool modelSet = GNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g);
  if (modelSet) {
    // Blink 6 times for AIRBORNE2g
    for (int i = 0; i < 6; i++) {
      digitalWrite(DynModpin, HIGH);
      delay(200);
      digitalWrite(DynModpin, LOW);
      delay(200);
    }
  } else {
    // Optional: indicate failure (e.g., 3 long blinks)
    for (int i = 0; i < 3; i++) {
      digitalWrite(DynModpin, HIGH);
      delay(600);
      digitalWrite(DynModpin, LOW);
      delay(600);
    }
  }

  Serial.println(F("init success!"));
  delay(500);
  //SD Card Initialization:
  Serial.print("Initializing SD card...");
  if (!SD.begin(16)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("init success!");
  // Radio: 
  Serial.print(F("[SX1276] Initializing ... "));
    // LoRa Pin setup
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

  Serial.print(F("[SX1276] Initializing ... "));
  if (!rf95.init()) {
    Serial.println("init failed!");
    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  rf95.setTxPower(23, false);  // PA_BOOST = false = true for SX1276, range 5-23 dBm
  rf95.setSpreadingFactor(9); // SF7..12
  rf95.setSignalBandwidth(62500); // 62.5kHz
  rf95.setCodingRate4(5); // 4/5
  Serial.println("init success!");

  // BME-680:
  while (!BME680.begin(I2C_STANDARD_MODE)) {                // Start BME680 using I2C, use first device found
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
long GPSSpeed = 0;
long GPSHeading = 0;
int GPSPDOP = 0;
int GPSFixType = 0;
int GPSCheckStatus = 0; // Check status of GPS module, 0 = no fix, 1 = fix, 2 = RTK fix, 3 = DGPS fix, 4 = PPP fix, 5 = SBAS fix
// BME 680:
int32_t temp = 0;
int32_t pressure = 0;
int32_t humidity = 0;
int32_t gas = 0;
//power reading
int batt = 0;
float volt = 0.0;


int var = 0;
byte cmd2[] = {0xFF,0x28,0x01,0x00, 0x00}; // replace end value w/ checksum
byte cmd3[] = {0xFF,0x28,0x19,0x01,0x2A,0x30, 0x00}; // replace end value w/ checksum
byte cmd4[] = {0xFF,0x28,0x17,0x00, 0x00}; // replace end value w/ checksum
byte byteArr[1];


void loop() {
 // Receive
  uint8_t buf[64];
  uint8_t len = sizeof(buf);
  int var = 0;

  if (rf95.available()) {
    if (rf95.recv(buf, &len)) {
      var = buf[0];
      counter++;

      switch (var) {
        case 1:
          digitalWrite(ledPin, HIGH); delay(500);
          digitalWrite(ledPin, LOW);  delay(500);
          break;
        case 2:
          Serial.write(cmd2, 5);
          break;
        case 3:
          Serial.write(cmd3, 6);
          break;
        case 4:
          Serial.write(cmd4, 5);
          break;
        default:
          Serial.println("ERROR: Command not recognized");
          break;
      }
    }
  }

  // Transmit:
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
    GPSFixType = GNSS.getFixType(); // Get fix type (0-5, 0 = no fix, 5 = RTK fix)
    GPSCheckStatus = GNSS.checkUblox(); // Check status of GPS module, 0 = no fix, 1 = fix, 2 = RTK fix, 3 = DGPS fix, 4 = PPP fix, 5 = SBAS fix
  }

  // get atmospheric data
  BME680.getSensorData(temp, humidity, pressure, gas);

  // Get Battery Data

  batt = analogRead(battPin);
  volt = (analogRead(battPin) * (3.3 / 4096)) * 3;
  
  // you can transmit C-string or Arduino string up to
  // 256 characters long

  char output[256];
  sprintf(output, "$$HAR, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %.2f", GPSLat, GPSLon, GPSAlt, GPSHeading, GPSSpeed, GPSPDOP, pressure, temp, humidity, GPSFixType, GPSCheckStatus, counter,volt);
  File file = SD.open("/HARdata.csv", FILE_APPEND);
  file.print("$$HAR,");
  file.print(GPSHour);
  file.print(":");
  file.print(GPSMinute);
  file.print(":");
  file.print(GPSSecond);
  file.print(",");
  file.print(GPSLat);
  file.print(",");
  file.print(GPSLon);
  file.print(",");
  file.print(GPSAlt);
  file.print(",");
  file.print(GPSHeading);
  file.print(",");
  file.print(GPSSpeed);
  file.print(",");
  file.print(GPSPDOP);
  file.print(",");
  file.print(pressure);
  file.print(",");
  file.print(temp);
  file.print(",");
  file.println(humidity);
  file.print(",");
  file.print(GPSFixType);
  file.print(",");
  file.print(GPSCheckStatus);
  file.close();

 Serial.println(output);

  // Transmit
  digitalWrite(pin_rx_enable, LOW);
  digitalWrite(pin_tx_enable, HIGH);
  rf95.send((uint8_t *)output, strlen(output));
  rf95.waitPacketSent();
  digitalWrite(pin_tx_enable, LOW);
  digitalWrite(pin_rx_enable, HIGH);

  Watchdog.reset();
  delay(1000);
}
