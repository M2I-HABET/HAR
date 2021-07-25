/*
 * High Altitude Reporter (HAR)
 * HW Version - 0.1
 * FW Version - 0.1
 * Matthew E. Nelson
 */

/*
 * Some code based on the following Libraries
 * - Sparkfun GNSS Library
 * - Adafruit Arcada
 * 
 */

/********HARDWARE REQUIREMENTS*****************
 * - Adafruit Clue Board
 * - Sparkfun Neo GPS Unit 
 * - Adafruit LoRa board
 * 
 * Hardware hookup is via QWICC Connector
 * (CLUE <-> [QWIIC] <----> [QWIIC] <-> Sparkfun GPS BOB
 *   ^
 *   |Micro::Bit breakout board
 *   LoRa Breakout Board
 * 
 * NOTE - Clue board requires 3-6 VDC (different from Micro:bit)
 * Recommend using 3 AA or Boost board
 * 
 * *********************************************/
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

/**************************************************************************************************
** Declare global variables and instantiate classes                                              **
**************************************************************************************************/
//set to true to see all serial outputs, otherwise only errors are sent out
#define DEBUG true

// Instantiates functions for hardware
Adafruit_Arcada arcada;
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_LIS3MDL lis3mdl;
Adafruit_SHT31 sht30;
//Adafruit_APDS9960 apds9960;
Adafruit_BMP280 bmp280;
extern Adafruit_FlashTransport_QSPI flashTransport;
extern Adafruit_SPIFlash Arcada_QSPI_Flash;
SFE_UBLOX_GNSS myGNSS;

//Setup LoRa radio
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// RHMesh manager(rf95,CLIENT_ADDRESS);

// file system object from SdFat
FatFileSystem fatfs;

// Check the timer callback, this function is called every millisecond!
volatile uint16_t milliseconds = 0;
void timercallback() {
  analogWrite(LED_BUILTIN, milliseconds);  // pulse the LED
  if (milliseconds == 0) {
    milliseconds = 255;
  } else {
    milliseconds--;
  }
}

unsigned long myTime;
unsigned long lastTime = 0; //Simple local timer.
unsigned long lastTime2 = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("High Altitude Reporter OS (HAROS)");
  Serial.println("============================================");
  Serial.println(" HW Rev. 0.1 | FW Rev. 0.2");
  Serial.println("============================================");
  delay(3000);

  init_i2c(DEBUG);
  init_arcada(DEBUG);
  arcada.display->setTextColor(ARCADA_GREEN);
  arcada.display->println("HAROS Booting up...");
  arcada.display->println("FW Rev: 0.2");
  init_flash(DEBUG);
  arcada.display->println("Memory...OK");
  init_gps(DEBUG);
  arcada.display->println("GPS...ONLINE");
  init_bmp280(DEBUG);
  init_lis3mdl(DEBUG);
  init_lsm6ds33(DEBUG);
  init_sht30(DEBUG);
  arcada.display->println("Sensors...ONLINE");
  init_lora(DEBUG);
  arcada.display->println("LoRa...ONLINE");

  Serial.println("Setup Process Comlplete.");
  arcada.display->setTextColor(ARCADA_GREEN);
  arcada.display->println("Bootup Complete!");
  delay(5000);
  arcada.display->fillScreen(ARCADA_BLACK);
}

/*=====================================================
* Main Loop
=========================================================*/

int16_t packetnum = 0;  // packet counter, we increment per transmission

void loop() {
  float temp, pres, humidity;

/*!
  @brief    5 Second Routine
  @details  At 5 second intervals, collect data, send to Serial and 
            send data to Flash memory on the Clue Board. This will append
            to the file in case of power failure. File format is as follows
            Temp,Pressure,Humidity,Lat,Lon,Altitude,FixType
  @return   void
*/
  
  if (millis() - lastTime > 5000)
  {
    // Serial.println("In the 1 sec function");
    lastTime = millis(); //Update the timer
    temp = bmp280.readTemperature();
    pres = bmp280.readPressure()/100;
    humidity = sht30.readHumidity();
    

  // Open the datalogging file for writing.  The FILE_WRITE mode will open
  // the file for appending, i.e. it will add new data to the end of the file.
  File dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
  
  // Check that the file opened successfully and write a line to it.
  if (dataFile) {
    dataFile.print(temp,2);
    dataFile.print(",");
    dataFile.print(pres, 2);
    dataFile.print(",");
    dataFile.print(humidity, 2);
    dataFile.print(",");
    dataFile.println();
    // Finally close the file when done writing.  This is smart to do to make
    // sure all the data is written to the file.
    dataFile.close();
    
    Serial.println("Wrote new measurement to data file!");
  }
  else {
    Serial.println("Failed to open data file for writing!");
  }

  /*
   * Update the Arcada Display
   */

    arcada.display->fillScreen(ARCADA_BLACK);
    arcada.display->setTextColor(ARCADA_WHITE, ARCADA_BLACK);
    arcada.display->setCursor(0, 0);
    
    arcada.display->print("Temp: ");
    arcada.display->print(temp);
    arcada.display->print(" C");
    arcada.display->println("         ");
    
    arcada.display->print("Baro: ");
    arcada.display->print(pres);
    arcada.display->print(" hPa");
    arcada.display->println("         ");
    
    arcada.display->print("Humid: ");
    arcada.display->print(humidity);
    arcada.display->print(" %");
    arcada.display->println("         ");
  

    /*
     * Print to Serial Output
    TODO: Add under DEBUG statement
    TODO: Move this to a function to generate the string
  */
    Serial.print(F("$HAR,"));
    Serial.print(temp);
    Serial.print(",");
    Serial.print(pres);
    Serial.print(",");
    Serial.println(humidity);
    Serial.print(",");
    //TODO Add checksum
  
 }

 /*!
  @brief    1 sec interval
  @details  Take the data collected and transmit via LoRa radio
            Data is sent as:
            BERT,Lat,Lon,Altitude,FixType,Temp,Pressure,Humidity,SatQuality
  @return   void
*/

  if (millis() - lastTime2 > 1000)
  {
    temp = bmp280.readTemperature();
    pres = bmp280.readPressure()/100;
    humidity = sht30.readHumidity();
    

    // Testing of sending a packet
    Serial.println("Transmitting..."); // Send a message to rf95_server
  
    char radiopacket[20] = "Hello World #      ";
    //Intellisense doesn't seem to see ITOA, but this does compile
    itoa(packetnum++, radiopacket+13, 10);
    Serial.print("Sending "); Serial.println(radiopacket);
    radiopacket[19] = 0;
    
    Serial.println("Sending...");
    delay(10);
    rf95.send((uint8_t *)radiopacket, 20);

    Serial.println("Waiting for packet to complete..."); 
    delay(10);
    rf95.waitPacketSent();
    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    Serial.println("Waiting for reply...");
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
  }
}