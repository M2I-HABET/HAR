/*
 * High Altitude Reporter (HAR)
 * Startup Routine
 * Matthew E. Nelson
 */

/*!
  @brief     Startup Routine
  @details   C code used to setup devices and sensors on the Clue Board. All 
             functions have a debug option. If set to false, only error messages
             will be printed. If true, then additional messages will be sent to
             the serial port.

             Startup timeline
             When calling the routines, they should be called in this order.
             init_i2C->init_arcada

             Failure Modes
             Certain failures will cause the bootup sequence to stop. These include
             LoRa failure, GPS failure, FLASH memory failure and Arcada failure.

             Neopixel Error modes
             The following relate to certain failures during bootup. Since Arcada
             is used to control the Neopixel, an Arcada failure results in the red
             LED on D13 to flash. If an error occurs, turn on debugging and check
             the serial output. In many cases, a failure is probably a hardware failure
             and may indicate a bad Clue board.

             Arcada Failure - D13 LED Flashes
             FLASH Failure - Solid Blue
             LoRa Failure - Solid Red

  */