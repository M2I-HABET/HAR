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
//#include <Adafruit_Arcada.h>
//#include <Adafruit_SPIFlash.h>
#include <Wire.h> //Needed for I2C to GNSS GPS
//#include <Adafruit_GFX.h>    // Core graphics library
//#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_SHT31.h>
//#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <RH_RF95.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
//#include <time.h>
#include "hardware.h"

// Instantiates functions for hardware

Adafruit_LSM6DS33 lsm;
Adafruit_LIS3MDL lis;
Adafruit_SHT31 sht30;
//Adafruit_APDS9960 apds9960;
Adafruit_BMP280 bmp280;

SFE_UBLOX_GNSS GNSS;

//Setup LoRa radio
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// RHMesh manager(rf95,CLIENT_ADDRESS);

/* GPS Structure
* GPS struct that contains the following info:
* Latitude, Longitude, Altitude, MSL Altitude,
* SIV, Data/Time, Fix Type and RTK value.
*/



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

    //Serial.println("Feather LoRa TX Test!");

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

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }
    rf95.setTxPower(RF95_PWR, false);
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

  if (GNSS.begin() == false)
  {
    Serial.println("FAILED");
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  else
  {
    GNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    // If we are going to change the dynamic platform model, let's do it here.
    // Possible values are:
    // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
    // For High Altitude Balloon applications, we need this set to AIRBORNE, either 1g or 2g should be fine.

    if (GNSS.setDynamicModel(DYN_MODEL_AIRBORNE1g) == false) // Set the dynamic model to PORTABLE
    {
      Serial.println(F("*** Warning: setDynamicModel failed ***"));
    }
    else
    {
      Serial.println(F("Dynamic platform model changed successfully!"));
    }
    // Let's read the new dynamic model to see if it worked
    uint8_t newDynamicModel = GNSS.getDynamicModel();
    if (newDynamicModel == DYN_MODEL_UNKNOWN)
    {
      Serial.println(F("*** Warning: getDynamicModel failed ***"));
    }
    else
    {
      Serial.print(F("The new dynamic model is: "));
      Serial.println(newDynamicModel);
    }

    //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_NAVCONF); //Uncomment this line to save only the NAV settings to flash and BBR
    GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    Serial.println("OK");
    if (debug) {
      Serial.println("Initializing GPS Sensor....OK");
      Serial.println("Setting GPS to Aircraft Mode....OK");
      Serial.println("Saving config to GPS Module....OK");
    }
  }
}

/*!
  @brief     Initialize LSM6D33 module
  @details   Initialize the LSM6D33 Accel/Gyro Sensor.
             This module is hardwired on the Clue board
  @param[in] debug    If true, output to the serial port
  @return
*/
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

/*!
  @brief     Initialize LIS3MDL module
  @details   Initialize the LIS3MDL Magnetometer.
             This module is hardwired on the Clue board
  @param[in] debug    If true, output to the serial port
  @return
*/
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

/*!
  @brief     Initialize SHT30 module
  @details   Initialize the SHT30 temperature and humidity sensor.
             This module is hardwired on the Clue board
  @param[in] debug    If true, output to the serial port
  @return
*/
void init_sht30 (bool debug) {
    if (!sht30.begin(0x44)) {
        Serial.println("No SHT30 found");
    } 
    if (debug) {
        Serial.println("Init SHT30....OK");
    }
}

/*!
  @brief     Initialize BMP280 module
  @details   Initialize the BMP280 temperature and pressure sensor.
             This module is hardwired on the Clue board
  @param[in] debug    If true, output to the serial port
  @return
*/
void init_bmp280 (bool debug) {
    /********** Check BMP280 */
    if (!bmp280.begin()) {
        Serial.println("No BMP280 found");
    } 
    if (debug) {
        Serial.println("Init BMP280...OK");
    }
    
}

/*!
  @brief     Read data from GPS sensor
  @details   Read data from the Sparkfun GPS sensor.
             Currently this is setup to read Latitude,
             Longitude, Altitude (including MSL), date
             and time info and satellite information.
  @param[in] void Nothing is passed in
  @return struct gps_data - Returns a struct with GPS data
*/
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

/*!
  @brief     Initialize LIS3MDL module
  @details   Initialize the LIS3MDL Magnetometer.
             This module is hardwired on the Clue board
  @param[in] debug    If true, output to the serial port
  @return
*/
environ_data read_environ(void) {

    envrion_data env;
    env.temp = bmp280.readTemperature();
    env.humidity = sht30.readHumidity();
    env.pres = bmp280.readPressure()/100;

    return env;
}

void send_packet(char *data) {
    //int packetnum = 0;

    // Testing of sending a packet
    //Serial.println("Transmitting..."); // Send a message to rf95_server
  
    //char radiopacket[20] = "Hello World #      ";
    //Intellisense doesn't seem to see ITOA, but this does compile
    //itoa(packetnum++, data+13, 10);
    //Serial.print("Sending "); Serial.println(data);
    //radiopacket[19] = 0;
    
    Serial.println("Sending...");
    delay(10);
    rf95.send((uint8_t *)data, 50);

    delay(10);
    rf95.waitPacketSent();
    Serial.println("Packet Sent!");
    // Now wait for a reply
    //uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    //uint8_t len = sizeof(buf);
    /*
    if (rf95.waitAvailableTimeout(1000))
      { 
      // Should be a reply message for us now   
        if (rf95.recv(buf, &len))
        {
          Serial.print("Got reply: ");
          Serial.println((char*)buf);
          Serial.print("RSSI: ");
          Serial.println(rf95.lastRssi(), DEC);    
        }
      else
      {
        Serial.println("Receive failed");
      }
    }
    else
    {
      Serial.println("No reply, is there a listener around?");
    }
    */

}

/*!
@TODO move receive part into this function, use interrrupts
*/
char rx_packet(void) {

}
