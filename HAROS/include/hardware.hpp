/*
 * High Altitude Reporter (HAR)
 * Startup Routine
 * Matthew E. Nelson
 */

/*!
  @brief     Hardware routine
  @details   Consolidates much of the code used to initialize and talk
             to the various hardware we have. This includes talking to 
             anything on the Clue or connected to the clue.
  
  @return    Depends
  */

#ifndef HARDWARE_HPP
#define HARDWARE_HPP

// Defines
// Configuration for the datalogging file:
#define FILE_NAME      "FDR.csv"

#define RFM95_CS 16
#define RFM95_RST 2
#define RFM95_INT 8
#define RF95_FREQ 434.0
#define RF95_PWR 23

// Define the two white LEDs on the front of the Clue Board
#define WHITE_LED 43

#define DIAGNOSTICS false // Change this to see diagnostics

typedef struct gps_data gps_data;

// uint32_t buttons, last_buttons;

/* Functions declarations
============================
*/

 void init_arcada (bool debug);

 void init_i2c (bool debug);

 void init_flash(bool debug);

 void init_lora(bool debug);

 void init_gps(bool debug);

 void init_lsm6ds33 (bool debug);

 void init_lis3mdl (bool debug);

 void init_sht30 (bool debug);

 void init_bmp280 (bool debug);

 gps_data read_gps(void);

#endif /* STARTUP_HPP_ */