/*
 * High Altitude Reporter (HAR)
 * Display helper
 * Matthew E. Nelson
 */

/*!
  @brief     Display helper
  @details   Functions to assist in drawing or sending text to the 
             display on the CLUE board.
  @param[in] gps    If true, output Latitude, Longitude, and Altitude
  @param[in] envir If true, output temp, humidity and pressure
  @param[in] accel If true, output X,Y,Z acceleartion
  @param[in] gyro  If true, output X,Y,Z rate gyro
  @param[in] mag   If true, output X,Y,Z magnetometer
  @return    Formated data string
  */