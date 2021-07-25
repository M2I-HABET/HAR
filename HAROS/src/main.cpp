/*
 * High Altitude Reporter (HAR)
 * HW Version - 0.1
 * FW Version - 0.3
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
//#include <Wire.h> //Needed for I2C to GNSS GPS
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <Adafruit_Sensor.h>

//#include <RH_RF95.h>

#include <time.h>
#include "hardware.h"

/**************************************************************************************************
** Declare global variables and instantiate classes                                              **
**************************************************************************************************/
//set to true to see all serial outputs, otherwise only errors are sent out
#define DEBUG true

// Instantiates functions for hardware
Adafruit_Arcada arcada;

extern Adafruit_FlashTransport_QSPI flashTransport;
extern Adafruit_SPIFlash Arcada_QSPI_Flash;

// file system object from SdFat
FatFileSystem fs;
//Setup LoRa radio
//RH_RF95 rf95(RFM95_CS, RFM95_INT);
// RHMesh manager(rf95,CLIENT_ADDRESS);

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
  delay(50);
  Serial.println("--------------------------------------------");
  delay(5000);
  Serial.println("High Altitude Reporter OS (HAROS)");
  Serial.println("============================================");
  Serial.println(" HW Rev. 0.1 | FW Rev. 0.3");
  Serial.println("============================================");

  init_i2c(DEBUG);
  pinMode(WHITE_LED, OUTPUT);
  digitalWrite(WHITE_LED, LOW);
  if (!arcada.arcadaBegin()) {
      Serial.println("Failed to start Arcada!");
      while (1);
  }
  arcada.displayBegin();

  for (int i=0; i<250; i+=10) {
      arcada.setBacklight(i);
      delay(1);
  }
  arcada.display->setCursor(0, 0);
  arcada.display->setTextWrap(true);
  arcada.display->setTextSize(2);
  arcada.display->setTextColor(ARCADA_GREEN);
  arcada.display->println("HAROS Booting up...");
  arcada.display->println("FW Rev: 0.3");
  /********** Check QSPI manually */
  if (!Arcada_QSPI_Flash.begin()){
    Serial.println("Could not find flash on QSPI bus!");
    arcada.display->setTextColor(ARCADA_RED);
    arcada.display->println("QSPI Flash FAIL");
  } else {
    uint32_t jedec;
    jedec = Arcada_QSPI_Flash.getJEDECID();
    Serial.print("JEDEC ID: 0x"); Serial.println(jedec, HEX);
    arcada.display->setTextColor(ARCADA_GREEN);
    arcada.display->print("QSPI JEDEC: 0x"); arcada.display->println(jedec, HEX);
  }
  
   /********** Check filesystem next */
  if (!arcada.filesysBegin()) {
    Serial.println("Failed to load filesys");
    arcada.display->setTextColor(ARCADA_YELLOW);
    arcada.display->println("Filesystem not found");
  } else {
    Serial.println("Filesys OK!");
    arcada.display->setTextColor(ARCADA_GREEN);
    arcada.display->println("Filesystem OK");
  }

  arcada.display->setTextColor(ARCADA_WHITE);
  arcada.display->println("Sensors Found: ");
  // Initialize flash library and check its chip ID.
  /*
  if (!Arcada_QSPI_Flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    //while(1);
  }
  Serial.print("Setting up Filesystem...OK");
  Serial.println("Flash chip JEDEC ID: 0x"); Serial.println(Arcada_QSPI_Flash.getJEDECID(), HEX);

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fs.begin(&Arcada_QSPI_Flash)) {
    Serial.println("Error, failed to mount newly formatted filesystem!");
    Serial.println("Was the flash chip formatted with the fatfs_format example?");
    while(1);
  }
  
  Serial.println("Mounting Filesystem...OK");
  arcada.display->println("Memory...OK");
  */
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

  // Open the datalogging file for writing.  The FILE_WRITE mode will open
  // the file for appending, i.e. it will add new data to the end of the file.

  /*
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
*/
  /*
   * Update the Arcada Display
   */

    arcada.display->fillScreen(ARCADA_BLACK);
    arcada.display->setTextColor(ARCADA_WHITE, ARCADA_BLACK);
    arcada.display->setCursor(0, 0);
    
    arcada.display->print("Temp: ");
    //arcada.display->print(temp);
    arcada.display->print(" C");
    arcada.display->println("         ");
    
    arcada.display->print("Baro: ");
    //arcada.display->print(pres);
    arcada.display->print(" hPa");
    arcada.display->println("         ");
    
    arcada.display->print("Humid: ");
    //arcada.display->print(humidity);
    arcada.display->print(" %");
    arcada.display->println("         ");
  

    /*
     * Print to Serial Output
    TODO: Add under DEBUG statement
    TODO: Move this to a function to generate the string
  */
    Serial.print(F("$HAR,"));
    //Serial.print(temp);
    Serial.print(",");
    //Serial.print(pres);
    Serial.print(",");
    //Serial.println(humidity);
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
    /*
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
    */
  }
}