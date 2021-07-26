/*
 * High Altitude Reporter (HAR)
 * Startup Routine
 * Matthew E. Nelson
 */

/*!
  @brief     Data Builder
  @details   These functions are used to build up the data strings
             used for either transmission or storage of data. All 
             transmitted strings need to formatted in the following 
             way:

             $IDENT,!TYPE,TIMESTAMP,[DATA],#CHECKSUM

             IDENT
             IDENT is the identification of the system that is sending the data. 
             We typically fly with at least two systems, usually HAR and BERT.
             This helps to identify which string came from what system.

             TYPE
             Type is 

  */

 /* Includes
============================
*/
#include <Arduino.h>

//#include <time.h>
#include "hardware.h"

char *gps_string (char *ident) {

  char typepkt[5] = "GPS";
  

}