# High Altitude Reporter (HAR)

HAR is named after the Norse god HÁR, the Norse god of knowledge whose name means "high." It seemed fitting then that we name our main onboard computer HAR.  HAR is used to gather data, store data, transmit data, and perform other housekeeping needs to keep the spacecraft functional through all stages of flight.

# Electrical Specification

HAR is the primary avionics system for all HABET missions. It provides a processing system, sensors, and communication functions to support all spacecraft missions.

## Electrical Overview

For all HABET missions, a few key things are needed to carry out a successful mission. To be successful, we need to have a system that allows us to track the spacecraft, communicate with the spacecraft, process sensor data, and manage power.

Generally speaking, Power and some sort of data management is usually the bare minimum needed for most payloads. However, many payloads either benefit or require the additional items listed. Therefore, Athena was designed with all of the items listed. As of the date of this document, the following features are currently provided.

| Feature | Specification |
| --- | --- |
| Power |  |
| LoRa Data link | 115,200 bps |
| Data Storage | Up to 2GB per payload |
| Data downlink | 1 packet every 4 seconds or higher. 1 packet must be under 256 bytes |
| Data uplink | 1 packet every 10 seconds or higher, 1 packet must be under 256 bytes |
| GPS Data | NMEA of the GPGGA string is available at 9600 bps |
| Video monitoring | Video stored onboard at 720p |
