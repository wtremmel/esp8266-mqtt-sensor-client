// No more ifdefs


#include <Arduino.h>
#include <ArduinoLog.h>

// Sensor Libraries
#include "Adafruit_Si7021.h"
#include "Adafruit_BME280.h"
#include "Adafruit_TSL2561_U.h"

// Global Objects
Adafruit_Si7021 si7021;
Adafruit_BME280 bme280;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);

// Flags for sensors found
bool si7021_found = false;
bool bme280_found = false;
bool tsl2561_found= false;
bool voltage_found= true;
bool rtc_init_done = false;
bool rtc_alarm_raised = false;


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

void setup() {
  setup_serial();
  setup_logging();
  setup_i2c();
}

void loop() {
  // put your main code here, to run repeatedly:
}
