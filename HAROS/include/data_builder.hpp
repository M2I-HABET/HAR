/*
 * High Altitude Reporter (HAR)
 * Data Builder helper
 * Matthew E. Nelson
 */

/*!
  @brief     Data builder helper
  @details   Based on what is requested, this function will build a data string
             that can be used both for storing data to the internal FLASH or for
             transmission on the network. It will read in the requested sensor data
             and then build the string per our data format. It will also calculate a
             checksum that can be used for data verification
  @param[in] gps    If true, output Latitude, Longitude, and Altitude
  @param[in] envir If true, output temp, humidity and pressure
  @param[in] accel If true, output X,Y,Z acceleartion
  @param[in] gyro  If true, output X,Y,Z rate gyro
  @param[in] mag   If true, output X,Y,Z magnetometer
  @return    Formated data string
  */

 // Prototyping of function

 char *data_builder (bool gps, bool envir, bool accel, bool gyro);
