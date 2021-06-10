# LoRa-APRS-Sender für Dragino LoRa/GPS shield
Send APRS mic-e encoded GPS position via LoRa to a LoRa APRS Gateway using Dragino LoRa/GPS shield and Arduino Mega.
The weather version sends in addition temperature, relative humidity and barometric pressure measured by a BME280 (I2C).

Please change this line to reflect your callsign:

`String Tcall="DO7DH-12";   //your Call Sign`

## Installation
Installation procedure is described here:

https://stefan.schultheis.at/2017/aprs-dragino-lora-gps-shield-70-cm-433-mhz/

## Copyright
Changes by Dennis Heitmann (DO7DH), 2021.

This code is a fork of the work of Stefan Schultheis for use with Dragino LoRa/GPS board.

It is a fork of https://github.com/IoT4pi/LoRa-APRS-Sender,
based on work of http://www.kh-gps.de/lora.htm
and
Copyright of the author Stuart Robinson - 02/07/2015 15:00

These programs may be used free of charge for personal, recreational and educational purposes only.
This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.
