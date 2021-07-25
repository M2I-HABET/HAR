/*
 * High Altitude Reporter (HAR)
 * Startup Routine
 * Matthew E. Nelson
 */

/*!
  @brief     Hardware Functions
  @details   C code used to setup devices and sensors on the Clue Board and to access
             these various sensors. All startup functions have a debug option. If set 
             to false, only error messages will be printed. If true, then additional 
             messages will be sent to the serial port.

             Startup timeline
             When calling the routines, they should be called in this order.
             init_i2C->init_arcada->init_gps->init_lora->additional sensors
             The reason for calling I2C and Arcada first is that these functions
             are used later on such as indicated an error. In addition, most sensors
             use I2C.

             Failure Modes
             Certain failures will cause the bootup sequence to stop. These include
             LoRa failure, GPS failure, FLASH memory failure and Arcada failure. 
             When this occurs, the Neopixel will indicate the error based on below.

             Neopixel Error modes
             The following relate to certain failures during bootup. Since Arcada
             is used to control the Neopixel, an Arcada failure results in the red
             LED on D13 to flash. If an error occurs, turn on debugging and check
             the serial output. In many cases, a failure is probably a hardware failure
             and may indicate a bad Clue board.

             Arcada Failure - D13 LED Flashes
             FLASH Failure - Flash Red
             LoRa Failure - Flash Green
             GPS Failure - Flash Blue
             Any other sensor failure - Flash Yellow

  */

/* Includes
============================
*/
#include <Arduino.h>
#include <Adafruit_Arcada.h>
#include <Adafruit_SPIFlash.h>
#include <Wire.h> //Needed for I2C to GNSS GPS
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_SHT31.h>
//#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <RH_RF95.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <time.h>
#include "hardware.hpp"

// Instantiates functions for hardware
Adafruit_Arcada arcad;
Adafruit_LSM6DS33 lsm;
Adafruit_LIS3MDL lis;
Adafruit_SHT31 sht;
//Adafruit_APDS9960 apds9960;
Adafruit_BMP280 bmp;
extern Adafruit_FlashTransport_QSPI flashTransport;
extern Adafruit_SPIFlash Arcada_QSPI_Flash;
SFE_UBLOX_GNSS GNSS;

//Setup LoRa radio
RH_RF95 lora(RFM95_CS, RFM95_INT);
// RHMesh manager(rf95,CLIENT_ADDRESS);

// file system object from SdFat
FatFileSystem fs;

struct gps_data {
    long GPSLat;
    long GPSLon;
    long GPSAlt;
    long altitudeMSL;
    byte SIV;
    int Year;
    int Month;
    int Day;
    int Hour;
    int Minute;
    int Second;
    byte fixType;
    int RTK;
};

/*!
  @brief     Initialize I2C Bus
  @details   Initialize the I2C bus, there is no indication if this works or not
  @param[in] debug    If true, output to the serial port
  @return    None
  */
void init_i2c (bool debug){
    Wire.begin();
    if (debug){
        Serial.println("Initializing I2C Bus....OK");
    }
}
/*!
  @brief     Initialize Arcada
  @details   Initialize the Arcada library and setup the display. Arcada allows us to
             tap into several functions useful for built in display on the Clue board
             and for talking to the Neopixel that is onboard
  @param[in] debug    If true, output to the serial port
  @return    None
  */
void init_arcada (bool debug){
    pinMode(WHITE_LED, OUTPUT);
    digitalWrite(WHITE_LED, LOW);
    if (!arcad.arcadaBegin()) {
        Serial.println("Failed to start Arcada!");
        while (1);
    }
    if (debug){
        Serial.println("Booting up Arcada...OK");
    }
    arcad.displayBegin();

    for (int i=0; i<250; i+=10) {
        arcad.setBacklight(i);
        delay(1);
    }

    arcad.display->setCursor(0, 0);
    arcad.display->setTextWrap(true);
    arcad.display->setTextSize(2);
    if (debug) {
        Serial.println("Setting up Display...OK");
    }

}

/*!
  @brief     Initialize QSPI Flash memory
  @details   Initialize the onboard QSPI Flash memory on the Clue board
             This will not make a filesystem, this needs to be done first
             To make a filesystem, simply flash a CircuitPython UF2 file
             and then reload the HAROS. If this fails, it is usually due to
             the filesystem not being created. 
  @param[in] debug    If true, output to the serial port
  @return    None
*/
void init_flash(bool debug){

    /********** Setup QSPI Flash Memory */
    // Initialize flash library and check its chip ID.
    if (!Arcada_QSPI_Flash.begin()) {
        Serial.println("Error, failed to initialize flash chip!");
    while(1);
    }
    if (debug) {
        Serial.print("Setting up Filesystem...OK");
        Serial.println("Flash chip JEDEC ID: 0x"); Serial.println(Arcada_QSPI_Flash.getJEDECID(), HEX);
        
    }

    // First call begin to mount the filesystem.  Check that it returns true
    // to make sure the filesystem was mounted.
    if (!fs.begin(&Arcada_QSPI_Flash)) {
        Serial.println("Error, failed to mount newly formatted filesystem!");
        Serial.println("Was the flash chip formatted with the fatfs_format example?");
        while(1);
    }
    if (debug) {
        Serial.println("Mounting Filesystem...OK");
    }
}

/*!
  @brief     Initialize LoRa module
  @details   Initialize the onboard QSPI Flash memory on the Clue board
             This will not make a filesystem, this needs to be done first
             To make a filesystem, simply flash a CircuitPython UF2 file
             and then reload the HAROS. If this fails, it is usually due to
             the filesystem not being created. 

             Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

             The default transmitter power is 13dBm, using PA_BOOST.
             If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
             you can set transmitter powers from 5 to 23 dBm:
  @param[in] debug    If true, output to the serial port
  @return
*/

void init_lora(bool debug) {
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    Serial.println("Feather LoRa TX Test!");

    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!lora.init()) {
        Serial.println("LoRa radio init failed");
        Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
        while (1);
    }

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!lora.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }
    lora.setTxPower(RF95_PWR, false);
    if (debug) {
        Serial.println("LoRa radio init...OK");
        Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
        Serial.print("Set TX power to: "); Serial.println(RF95_PWR);
    }

}

/*!
  @brief     Initialize GPS module
  @details   Initialize the Sparkfun NEO u-blox GPS module
             This module is connected via the QWIIC connector
             on the Clue board that provides I2C and power.
  @param[in] debug    If true, output to the serial port
  @return
*/
void init_gps(bool debug) {
    Serial.print("Initializing GPS Sensor....");

    if (GNSS.begin() == false)
    {
        Serial.println("FAILED");
        Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
        
        while (1);
    }
    else{
        GNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
        GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
        if (debug) {
            Serial.println("Initializing GPS Sensor....OK");
            Serial.println("Setting GPS to UBX Mode....OK");
            Serial.println("Saving config to GPS Module....OK");
        }
    }
}

void init_lsm6ds33 (bool debug) {
    if (!lsm.begin_I2C()) {
        Serial.println("No LSM6DS33 found!");
    }
    else {
        if (debug) {
        Serial.println("Init LSM6DS33....OK");
        }
    }
}

void init_lis3mdl (bool debug) {
    if (!lis.begin_I2C()) {
        Serial.println("No LIS3MDL found!");
    } 
    else {
        if (debug) {
            Serial.println("Init LIS3MDL....OK");
        }
    }
}

void init_sht30 (bool debug) {
    if (!sht.begin(0x44)) {
        Serial.println("No SHT30 found");
    } 
    if (debug) {
        Serial.println("Init SHT30....OK");
    }
}

void init_bmp280 (bool debug) {
    /********** Check BMP280 */
    if (!bmp.begin()) {
        Serial.println("No BMP280 found");
    } 
    if (debug) {
        Serial.println("Init BMP280...OK");
    }
    
}

gps_data read_gps(void) {

    gps_data GPS;
    GPS.GPSLat = GNSS.getLatitude();
    GPS.GPSLon = GNSS.getLongitude();
    GPS.GPSAlt = GNSS.getAltitude();
    GPS.altitudeMSL = GNSS.getAltitudeMSL();
    
    GPS.SIV = GNSS.getSIV();
    GPS.Year = GNSS.getYear();
    GPS.Month = GNSS.getMonth();
    GPS.Day = GNSS.getDay();
    GPS.Hour = GNSS.getHour();
    GPS.Minute = GNSS.getMinute();
    GPS.Second = GNSS.getSecond();
    GPS.fixType = GNSS.getFixType();
    GPS.RTK = GNSS.getCarrierSolutionType();

    return GPS;

}
