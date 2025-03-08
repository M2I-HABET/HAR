/* ======================================
HABET LoRa Ground Statsion
Version: 1.1

LoRa Ground Station based on the RFM95 chip sets
Should be compatible with most Adafruit RFM95 boards
and feather wings. 

Notes: This uses a basic TX/RX example and no verification or addressing
is used. 

Based on Example code provided by Adafruit:
https://learn.adafruit.com/feather-rp2040-rfm95/using-the-rfm-9x-radio

Currently tested and verified working on:
  * Adafruit RP2040 RFM

Current HAR Hardware tested and verified:
  * HAR_Light (RP2040 RFM95 based board)
==========================================*/


#include <SPI.h>
#include <RH_RF95.h>

// First 3 here are boards w/radio BUILT-IN. Boards using FeatherWing follow.
#if defined (__AVR_ATmega32U4__)  // Feather 32u4 w/Radio
  #define RFM95_CS    8
  #define RFM95_INT   7
  #define RFM95_RST   4

#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
  #define RFM95_CS    8
  #define RFM95_INT   3
  #define RFM95_RST   4

#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_RFM)  // Feather RP2040 w/Radio
  #define RFM95_CS   16
  #define RFM95_INT  21
  #define RFM95_RST  17

#elif defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM95_CS    4  //
  #define RFM95_INT   3  //
  #define RFM95_RST   2  // "A"

#elif defined(ESP8266)  // ESP8266 feather w/wing
  #define RFM95_CS    2  // "E"
  #define RFM95_INT  15  // "B"
  #define RFM95_RST  16  // "D"

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
  #define RFM95_CS   10  // "B"
  #define RFM95_INT   9  // "A"
  #define RFM95_RST  11  // "C"

#elif defined(ESP32)  // ESP32 feather w/wing
  #define RFM95_CS   33  // "B"
  #define RFM95_INT  27  // "A"
  #define RFM95_RST  13

#elif defined(ARDUINO_NRF52832_FEATHER)  // nRF52832 feather w/wing
  #define RFM95_CS   11  // "B"
  #define RFM95_INT  31  // "C"
  #define RFM95_RST   7  // "A"

#endif

// For Region 2, must be set between 902 - 928 MHz
// Default is 915 MHz, if interference is detected,
// try moving to a different freq. in that band.
#define RF95_FREQ 915.0
// The default transmitter power is 13dBm, using PA_BOOST.
// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
// you can set transmitter powers from 5 to 23 dBm:
#define RF95_PWR 23

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// ========= Setup Hardware =========================
void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("HABET RFM95 LoRa Ground Station");
  Serial.println("Ver: 1.0");
  Serial.println("Booting up board...");

  // manual reset
  Serial.println("Resetting LoRa radio...");
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  // Defaults after init are 915.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  Serial.println("Initializing LoRa radio...");
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

  rf95.setTxPower(RF95_PWR, false);
  Serial.print("Power set to "); Serial.println(RF95_PWR);
}
// =========== Begin Main Loop ===========================
void loop() {
  // We wait until we get a message
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len)) {
      // Ensure the received buffer is null-terminated
      buf[len] = '\0';
      
      digitalWrite(PIN_LED, HIGH);
      Serial.print("$RSSI,");
      Serial.println(rf95.lastRssi(), DEC);

      // Tokenize the CSV string from the buffer.
      char *ident   = strtok((char*)buf, ",");
      char *num     = strtok(NULL, ",");
      char *lat     = strtok(NULL, ",");
      char *lon     = strtok(NULL, ",");
      char *alt     = strtok(NULL, ",");
      char *pAlt    = strtok(NULL, ",");
      char *heading = strtok(NULL, ",");
      char *speed   = strtok(NULL, ",");
      char *pdop    = strtok(NULL, ",");
      char *pressure= strtok(NULL, ",");
      char *temp    = strtok(NULL, ",");
      char *humidity = strtok(NULL, ",");
      char *gas     = strtok(NULL, ",");
      char *batt    = strtok(NULL, ",");

      // Perform conversions:
      // For example, dividing by 100 to get the correct value.
      float pressureVal = pressure ? atof(pressure) / 100.0 : 0.0;
      float tempVal     = temp     ? atof(temp)     / 100.0 : 0.0;
      float humidVal    = humidity ? atof(humidity)  / 1000.0 : 0.0;
      float LatVal      = lat      ? atof(lat) / 10000000.0 : 0.0;
      float LonVal      = lon     ? atof(lon)     / 10000000.0 : 0.0;
      float AltVal      = alt  ? atof(alt)  / 1000.0 : 0.0;
      float headVal     = heading  ? atof(heading)  / 100000.0 : 0.0;

      // Print header (if desired; you might consider printing this once in setup())
      Serial.println("IDENT Packet#   LAT        LON         Alt       PAlt     Heading    Speed     PDOP     Pressure  TempC     Humidity  Gas       Batt");

      // Prepare a formatted output row with the converted values.
      // Adjust field widths as needed.
      char output[150];
      sprintf(output, "%-6s %-8s %-9.7f %-9.7f %-9.3f %-9s %-9.3f %-9s %-9s %-9.2f %-9.2f %-9.2f %-9s %-9s",
              ident   ? ident   : "",
              num     ? num     : "",
              LatVal,
              LonVal,
              AltVal,
              pAlt    ? pAlt    : "",
              headVal,
              speed   ? speed   : "",
              pdop    ? pdop    : "",
              pressureVal,
              tempVal,
              humidVal,
              gas     ? gas     : "",
              batt    ? batt    : "");
      Serial.println(output);

      digitalWrite(PIN_LED, LOW);
    }
  }
}
