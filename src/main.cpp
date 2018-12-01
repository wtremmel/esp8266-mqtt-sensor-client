// No more ifdefs


#include <Arduino.h>
#include <ArduinoLog.h>

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include <PubSubClient.h>
#include <FS.h>
#include <ArduinoJson.h>


// Sensor Libraries
#include <Wire.h>
#include "Adafruit_Si7021.h"
#include "Adafruit_BME280.h"
#include "Adafruit_TSL2561_U.h"
#include <Adafruit_NeoPixel.h>

// Global defines
#define NEOPIXEL 14 //D5
#define NROFLEDS 10


// Global Objects
Adafruit_Si7021 si7021;
Adafruit_BME280 bme280;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);
Adafruit_NeoPixel led = Adafruit_NeoPixel(NROFLEDS, NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Strings for dynamic config
String Smyname, Spass, Sssid, Smqttserver, Ssite, Slocation, Smqttuser, Smqttpass;
unsigned int Imqttport;


// Flags for sensors found
bool si7021_found = false;
bool bme280_found = false;
bool tsl2561_found= false;
bool voltage_found= true;
bool rtc_init_done = false;
bool rtc_alarm_raised = false;


// LED routines
void setled(byte r, byte g, byte b) {
  led.setPixelColor(0, r, g, b);
  led.show();
}

void setled(byte n, byte r, byte g, byte b) {
  led.setPixelColor(n, r, g, b);
  led.show();
}

void setled(byte n, byte r, byte g, byte b, byte show) {
  led.setPixelColor(n, r, g, b);
  if (show) {
    led.show();
  }
}

void setled(byte show) {
  if (!show) {
    int i;
    for (i = 0; i < NROFLEDS; i++) {
      setled(i, 0, 0, 0, 0);
    }
  }
  led.show();
}

// Debug functions
void log_config () {

  Log.verbose("Smyname = %s",Smyname.c_str());
  Log.verbose("Ssite = %s",Ssite.c_str());
  Log.verbose("Slocation = %s",Slocation.c_str());
  Log.verbose("Sssid = %s",Sssid.c_str());
  Log.verbose("Spass = %s",Spass.c_str());
  Log.verbose("Smqttuser = %s",Smqttuser.c_str());
  Log.verbose("Smqttpass = %s",Smqttpass.c_str());
  Log.verbose("Imqttport = %d",Imqttport);

}

// Logging helper routines
void printTimestamp(Print* _logOutput) {
  char c[12];
  sprintf(c, "%10lu ", millis());
  _logOutput->print(c);
}

void printNewline(Print* _logOutput) {
  _logOutput->print('\n');
}


// Setup routines
//
// Scan for sensors
//
void setup_i2c() {
  byte error, address;

// 0x29 TSL45315 (Light)
// 0x38 VEML6070 (Light)
// 0x39 TSL2561
// 0x40 SI7021
// 0x48 4*AD converter
// 0x4a GY49 or MAX44009 Light Sensor
// 0x50 PCF8583P
// 0x57 ATMEL732
// 0x68 DS3231 Clock
// 0x76 BME280
// 0x77 BME680 (also BMP180)


  Log.notice("Scanning i2c bus");
  Wire.begin();
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Log.trace("I2C device found at address 0x%2x",address);

      if (address == 0x39) {
        tsl2561 = Adafruit_TSL2561_Unified(address);
        tsl2561_found = tsl2561.begin();
        Log.notice("TSL2561 found? %T",tsl2561_found);
        if (tsl2561_found) {
          // init the sensor
          tsl2561.enableAutoRange(true);
          tsl2561.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
        }
      }
      if (address == 0x40) {
        // SI7021
        si7021 = Adafruit_Si7021();
        si7021_found = si7021.begin();
        Log.notice("Si7021 found? %T",si7021_found);
      }

      if (address == 0x76 || address == 0x77) {
        // BME280
        bme280_found = bme280.begin(address);
        Log.notice("BME280 found? %T at %x",bme280_found,address);
      }
    }
  }
}

void setup_serial() {
  Serial.begin(115200);
}

void setup_logging() {
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.setPrefix(printTimestamp);
  Log.setSuffix(printNewline);
  Log.verbose("Logging has started");
}

// we assume there is always a LED connected
void setup_led() {
  led.begin();
  led.show();
}

// read the config file and parse its data
void setup_readconfig() {
  SPIFFS.begin();
  File f = SPIFFS.open("/config.json","r");
  if (!f) {
    Log.error("Cannot open config file");
    return;
  }
  StaticJsonBuffer<512> jsonBuffer;

 // Parse the root object
 JsonObject &root = jsonBuffer.parseObject(f);

 if (!root.success())
   Log.error("Failed to read file");

 // Copy values from the JsonObject to the Config
   Smyname = root["myname"].as<String>();
   Spass = root["network"]["pass"].as<String>();
   Sssid = root["network"]["ssid"].as<String>();
   Smqttserver = root["mqtt"]["server"].as<String>();
   Ssite = root["location"]["site"].as<String>();
   Slocation = root["location"]["room"].as<String>();
   Smqttuser = root["mqtt"]["user"].as<String>();
   Smqttpass = root["mqtt"]["pass"].as<String>();
   Imqttport = root["mqtt"]["port"];


  f.close();
  SPIFFS.end();
}

void setup_wifi() {
  WiFi.persistent(false);
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(Smyname);
  WiFi.begin(Sssid.c_str(), Spass.c_str());

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    retries++;
    Log.error("Wifi.status() = %d",WiFi.status());
  }
}

void setup_mqtt() {

}

void setup() {
  setup_serial();
  setup_led();
  setled(255,0,0);
  setup_logging();
  setup_readconfig();
  log_config();
  setup_i2c();
  setup_wifi();
  setled(255, 128, 0);
  setup_mqtt();
  setled(0, 255, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
}
