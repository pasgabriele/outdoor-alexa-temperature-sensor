# outdoor-alexa-temperature-sensor

This device is an outdoor battery powered temperature sensor compatible with Amazon Alexa.

To permit Amazon Alexa integration is used the Sinric Pro service (https://sinric.pro).

## Features

* Battery powered
* Compatible with Amazon Alexa
* Configurable Deep Sleep time to save battery life

## Requirements

* Sinric Pro account

## Hardware

The hardware composition is the following:
* NodeMCU ESP32
* Dallas DS18B20 Temperature sensor
* 18650 3.7V Li-Ion battery

### Wiring

ESP32     | DS18B20
--------- | -----
3V3       | +
GND       | -
D5 (GPIO5)| DATA
 
## Software
 
The sketch is derived from the examples stored in [Sinric Pro official GitHub page](https://github.com/sinricpro/esp8266-esp32-sdk "Sinric Pro official GitHub page"). It's edited to permit the Deep Sleep mode so that the Outdoor Alexa Temperature Sensor can be powered by a 18650 battery and keeps the battery alive as long as possible.


