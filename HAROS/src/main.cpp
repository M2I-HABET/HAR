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
#include <time.h>
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

/**************************************************************************************************
** Declare global variables and instantiate classes                                              **
**************************************************************************************************/

// Define the two white LEDs on the front of the Clue Board
#define WHITE_LED 43
// Defines for LoRa module
#define RFM95_CS 16
#define RFM95_RST 2
#define RFM95_INT 8
#define RF95_FREQ 434.0

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

#define DIAGNOSTICS false // Change this to see diagnostics


uint32_t buttons, last_buttons;

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

// Forward function declaration with default value for sea level
//float altitude(const int32_t press, const float seaLevel = 1013.25);
//float altitude(const int32_t press, const float seaLevel) {
  /*!
  @brief     This converts a pressure measurement into a height in meters
  @details   The corrected sea-level pressure can be passed into the function if it is known,
             otherwise the standard atmospheric pressure of 1013.25hPa is used (see
             https://en.wikipedia.org/wiki/Atmospheric_pressure) for details.
  @param[in] press    Pressure reading from BME680
  @param[in] seaLevel Sea-Level pressure in millibars
  @return    floating point altitude in meters.
  */
//static float Altitude;
// Altitude = 44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
//  return (Altitude);
// }  

/*
typedef struct gpsdata {
    long latitude;
    long longitude;
    long altitude;
    long altitudeMSL;
    byte SIV;
    byte fixType;
    byte RTK;
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    
}
*/

// file system object from SdFat
FatFileSystem fatfs;

// Configuration for the datalogging file:
#define FILE_NAME      "FDR.csv"

unsigned long myTime;
unsigned long lastTime = 0; //Simple local timer.
unsigned long lastTime2 = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("High Altitude Reporter OS (HAROS)");
  Serial.println("============================================");
  Serial.println(" HW Rev. 0.1 | FW Rev. 0.1");
  Serial.println("============================================");
  delay(3000);
  pinMode(WHITE_LED, OUTPUT);
  digitalWrite(WHITE_LED, LOW);
  
  Wire.begin();
  Serial.println("Initializing I2C Bus....OK");
  
  Serial.print("Booting up Arcada...");
  if (!arcada.arcadaBegin()) {
    Serial.println("Failed to start Arcada!");
    while (1);
  }
  Serial.println("OK");
  arcada.displayBegin();
  Serial.print("Setting up Display...");

  for (int i=0; i<250; i+=10) {
    arcada.setBacklight(i);
    delay(1);
  }

  arcada.display->setCursor(0, 0);
  arcada.display->setTextWrap(true);
  arcada.display->setTextSize(2);
  Serial.println("OK");

  /********** Setup QSPI Flash Memory */
  Serial.print("Setting up Filesystem...");

  // Initialize flash library and check its chip ID.
  if (!Arcada_QSPI_Flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while(1);
  }
  Serial.println("Flash chip JEDEC ID: 0x"); Serial.println(Arcada_QSPI_Flash.getJEDECID(), HEX);
  Serial.print("Mounting Filesystem...");

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&Arcada_QSPI_Flash)) {
    Serial.println("Error, failed to mount newly formatted filesystem!");
    Serial.println("Was the flash chip formatted with the fatfs_format example?");
    while(1);
  }
  Serial.println("Mounted filesystem!");

  arcada.display->setTextColor(ARCADA_WHITE);
  arcada.display->println("Getting a clue...");
  arcada.display->setTextColor(ARCADA_WHITE);
  arcada.display->println("Sensors Found: ");

  /******** Initialize LoRa Radio*************************
   * dfd
   * dfd
   * */

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

/********** Check LSM6DS33 */
  Serial.print("Checking LSM6DS33...");
  if (!lsm6ds33.begin_I2C()) {
    Serial.println("No LSM6DS33 found");
    arcada.display->setTextColor(ARCADA_RED);
  } else {
    Serial.println("**LSM6DS33 OK!");
    arcada.display->setTextColor(ARCADA_GREEN);
  }
  arcada.display->println("LSM6DS33 ");

  /********** Check LIS3MDL */
  Serial.print("Checking LIS3MDL...");
  if (!lis3mdl.begin_I2C()) {
    Serial.println("No LIS3MDL found");
    arcada.display->setTextColor(ARCADA_RED);
  } else {
    Serial.println("**LIS3MDL OK!");
    arcada.display->setTextColor(ARCADA_GREEN);
  }
  arcada.display->print("LIS3MDL ");

  /********** Check SHT3x */
  Serial.print("Checking SHT30...");
  if (!sht30.begin(0x44)) {
    Serial.println("No SHT30 found");
    arcada.display->setTextColor(ARCADA_RED);
  } else {
    Serial.println("**SHT30 OK!");
    arcada.display->setTextColor(ARCADA_GREEN);
  }
  arcada.display->print("SHT30 ");

  /********** Check BMP280 */
  Serial.print("Checking BPM280...");
  if (!bmp280.begin()) {
    Serial.println("No BMP280 found");
    arcada.display->setTextColor(ARCADA_RED);
  } else {
    Serial.println("**BMP280 OK!");
    arcada.display->setTextColor(ARCADA_GREEN);
  }
  arcada.display->println("BMP280");

  buttons = last_buttons = 0;
  arcada.timerCallback(1000, timercallback);
  arcada.display->setTextWrap(false);

  Serial.print("Initializing GPS Sensor....");

  if (myGNSS.begin() == false)
  {
    Serial.println("FAILED");
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    arcada.display->setTextColor(ARCADA_RED);
    while (1);
  }
  else{
    arcada.display->setTextColor(ARCADA_GREEN);
    arcada.display->println("NEO-N9M GPS");
    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    Serial.println("OK");
  }


  Serial.println("Setup Process Comlplete...Booting HAROS");
  arcada.display->setTextColor(ARCADA_GREEN);
  arcada.display->println("Bootup Complete!");
  delay(5000);
  arcada.display->fillScreen(ARCADA_BLACK);

}

/*=====================================================
* Main Loop
=========================================================*/

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop() {
  float temp, pres, humidity;
  long GPSLat, GPSLon,GPSAlt;
  long altitudeMSL;
  byte SIV;
  int Year, Month, Day, Hour, Minute, Second;
  byte fixType;
  int signalQuality = -1;
  int err;
  char IMEI[16];
  int RTK;

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
    GPSLat = myGNSS.getLatitude();
    GPSLon = myGNSS.getLongitude();
    GPSAlt = myGNSS.getAltitude();
    altitudeMSL = myGNSS.getAltitudeMSL();
    
    SIV = myGNSS.getSIV();
    Year = myGNSS.getYear();
    Month = myGNSS.getMonth();
    Day = myGNSS.getDay();
    Hour = myGNSS.getHour();
    Minute = myGNSS.getMinute();
    Second = myGNSS.getSecond();
    fixType = myGNSS.getFixType();

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
    dataFile.print(GPSLat);
    dataFile.print(",");
    dataFile.print(GPSLon);
    dataFile.print(",");
    dataFile.print(altitudeMSL);
    dataFile.print(",");
    dataFile.print(fixType);
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
  
    arcada.display->print("lat: ");
    arcada.display->print(GPSLat);
    arcada.display->println("         ");
  
    arcada.display->print("lon: ");
    arcada.display->print(GPSLon);
    arcada.display->println("         ");
  
    arcada.display->print("alt: ");
    arcada.display->print(GPSAlt);
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
    Serial.print(GPSLat);
    Serial.print(",");
    Serial.print(GPSLon);
    //Serial.print(" (degrees * 10^-7)");
    
    Serial.print(",");
    Serial.print(GPSAlt);
    Serial.print(",");
    Serial.print(altitudeMSL);
  
    Serial.print(",");
    Serial.print(SIV);
  
    Serial.print(",");
    Serial.println(fixType);
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
    GPSLat = myGNSS.getLatitude();
    GPSLon = myGNSS.getLongitude();
    GPSAlt = myGNSS.getAltitude();
    altitudeMSL = myGNSS.getAltitudeMSL();
    
    SIV = myGNSS.getSIV();
    Year = myGNSS.getYear();
    Month = myGNSS.getMonth();
    Day = myGNSS.getDay();
    Hour = myGNSS.getHour();
    Minute = myGNSS.getMinute();
    Second = myGNSS.getSecond();
    fixType = myGNSS.getFixType();

    RTK = myGNSS.getCarrierSolutionType();

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