/*
 * Example for how to use SinricPro Temperaturesensor device:
 * - setup a temperature sensor device
 * - send temperature event to SinricPro server when temperature has changed
 *
 * DHT Sensor is connected to GPIO5 on ESP32 devices
 *
 * DHT Library used in this example: https://github.com/markruys/arduino-DHT
 *
 * If you encounter any issues:
 * - check the readme.md at https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md
 * - ensure all dependent libraries are installed
 *   - see https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md#arduinoide
 *   - see https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md#dependencies
 * - open serial monitor and check whats happening
 * - check full user documentation at https://sinricpro.github.io/esp8266-esp32-sdk
 * - visit https://github.com/sinricpro/esp8266-esp32-sdk/issues and check for existing issues or open a new one
 */

// Uncomment the following line to enable serial debug output
//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
       #define DEBUG_ESP_PORT Serial
       #define NODEBUG_WEBSOCKETS
       #define NDEBUG
#endif

#include <Arduino.h>
#include <WiFi.h>
#include "SinricPro.h"
#include "SinricProTemperaturesensor.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define WIFI_SSID	  "SSID"
#define WIFI_PASS	  "WIFI PASSWORD"
#define APP_KEY		  "SINRIC APP KEY"	  // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET	  "SINRIC APP SECRET"  // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define TEMP_SENSOR_ID	  "SINRIC SENSOR ID"	  // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define BAUD_RATE	  9600			  // Change baudrate to your need (used for serial monitor)
#define LED		  2			  // Buildin ESP32/ESP8266 LED
#define OPERATIVE_TIME	  30000			  // Max device operative time (in millis) before deep sleep
#define uS_TO_S_FACTOR	  1000000		  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP     1800			  // Deep sleep time (in seconds) (900 = 15 minutes)
#define ONE_WIRE_BUS      5

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

bool deviceIsOn;				  // Temeprature sensor on/off state
float temperature;				  // actual temperature
float lastTemperature;				  // last known temperature (for compare)
unsigned long startup;				  // startup time (for compare)

//RTC variables. These will be preserved during the deep sleep.
RTC_DATA_ATTR int bootCount = 0;		  // counter to monitor deep sleep cycles

//function that prints the reason by which ESP32 has been awaken from sleep
void print_wakeup_reason(){
	esp_sleep_wakeup_cause_t wakeup_reason;
	wakeup_reason = esp_sleep_get_wakeup_cause();
	switch(wakeup_reason)
	{
		case ESP_SLEEP_WAKEUP_EXT0  : Serial.println("INFO: Wakeup caused by external signal using RTC_IO"); break;
		case ESP_SLEEP_WAKEUP_EXT1  : Serial.println("INFO: Wakeup caused by external signal using RTC_CNTL"); break;
		case ESP_SLEEP_WAKEUP_TIMER  : Serial.println("INFO: Wakeup caused by timer"); break;
		case ESP_SLEEP_WAKEUP_TOUCHPAD  : Serial.println("INFO: Wakeup caused by touchpad"); break;
		case ESP_SLEEP_WAKEUP_ULP  : Serial.println("INFO: Wakeup caused by ULP program"); break;
		default : Serial.println("INFO: Wakeup was not caused by deep sleep"); break;
	}
}

/* bool onPowerState(String deviceId, bool &state)
 *
 * Callback for setPowerState request
 * parameters
 *  String deviceId (r)
 *    contains deviceId (useful if this callback used by multiple devices)
 *  bool &state (r/w)
 *    contains the requested state (true:on / false:off)
 *    must return the new state
 *
 * return
 *  true if request should be marked as handled correctly / false if not
 */
bool onPowerState(const String &deviceId, bool &state) {
  Serial.printf("Temperaturesensor turned %s (via SinricPro) \r\n", state?"on":"off");
  deviceIsOn = state;				  // turn on/off temperature sensor
  return true;					  // request handled properly
}

/* handleTemperatatureSensor()
 * - Checks if Temperaturesensor is turned on
 * - Get actual temperature and humidity and check if these values are valid
 * - Compares actual temperature and humidity to last known temperature and humidity
 * - Send event to SinricPro Server if temperature or humidity changed
 */
void handleTemperaturesensor() {
  if (deviceIsOn == false) return;		  // device is off...do nothing

  sensors.requestTemperatures(); // Send the command to get temperatures
  temperature = sensors.getTempCByIndex(0);

  if (temperature == DEVICE_DISCONNECTED_C) {	  // reading failed...
    Serial.printf("Dallas DS18B20 reading failed!\r\n");	  // print error message
    return;					  // try again next time
  }

  if (temperature == lastTemperature) return; // if no values changed do nothing

  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];	  // get temperaturesensor device
  bool success = mySensor.sendTemperatureEvent(temperature);	  // send event
  if (success) {							  // if event was sent successfuly, print temperature to serial
    Serial.printf("Temperature: %2.1f Celsius\r\n", temperature);
  } else {								  // if sending event failed, print error message
    Serial.printf("Something went wrong...could not send Event to server!\r\n");
  }

  lastTemperature = temperature;		  // save actual temperature for next compare
}


// setup function for WiFi connection
void setupWiFi() {
  Serial.printf("\r\n[Wifi]: Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(250);
  }
  IPAddress localIP = WiFi.localIP();
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %d.%d.%d.%d\r\n", localIP[0], localIP[1], localIP[2], localIP[3]);
}

// setup function for SinricPro
void setupSinricPro() {
  // add device to SinricPro
  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  mySensor.onPowerState(onPowerState);

  // setup SinricPro
  SinricPro.onConnected([](){ Serial.printf("Connected to SinricPro\r\n"); });
  SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n"); });
  SinricPro.begin(APP_KEY, APP_SECRET);
  SinricPro.restoreDeviceStates(true);		  // get latest known deviceState from server (is device turned on?)
}

// main setup function
void setup() {
  //turn on ESP buildin LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  Serial.begin(BAUD_RATE); Serial.printf("\r\n\r\n");
  sensors.begin();

  //increment boot number and print it every reboot
  ++bootCount;
  Serial.println("INFO: Boot number: " + String(bootCount));

  print_wakeup_reason();			  // print the wakeup reason for ESP32

  setupWiFi();
  setupSinricPro();

  startup = millis();				  // get startup time
}

void loop() {
  SinricPro.handle();
  handleTemperaturesensor();

  unsigned long actualMillis = millis();	  // get actual time (to compare with startup time)

  if (actualMillis - startup > OPERATIVE_TIME){	  // compare starup time and actual time to set device in deep sleep mode
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

    //turn off ESP32 buildin LED
    digitalWrite(LED, LOW);

    //go to deep sleep now
    Serial.println("Going to sleep now");
    delay(1000);
    Serial.flush();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }
}
