// No more ifdefs

// List of devices:
// #define ESP1 // Test Arbeitszimmer
// #define ESP2 // Kueche
// #define ESP3 // Wohnzimmer
// #define ESP4 // Garten
// #define ESP6 // Test mit Lithium-Akku
// #define ESP7  // Hausanschlussraum
// #define ESP8 // Fernsehzimmer
// #define ESP9 // Heizraum
// #define ESP10 // Flur 1.OG
// #define ESP11 // Lolin32 Lite
// #define ESP12 // Schlafzimmer
// #define ESP13 // Lichterkette / Uhr
// #define ESP14 // Display
// #define ESP15 // Keller
// #define ESP16 // Flur EG
// #define ESP17 // DECIX Office
// #define ESP18 // Flur 2.OG
// #define ESP19 // Loggia
// #define ESP31 // Test
// #define ESP30 // DingDong

const char compile_date[] = __DATE__ " " __TIME__;

#include <Arduino.h>
#include <ArduinoLog.h>


#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <AddrList.h>
#include <FS.h>
#define I2CSDA 4  //D2 gruen
#define I2CSCL 5  //D1 gelb
ADC_MODE(ADC_VCC);

#elif defined (ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <SPIFFS.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
typedef enum {
    APP_GAP_STATE_IDLE = 0,
    APP_GAP_STATE_DEVICE_DISCOVERING,
    APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE,
    APP_GAP_STATE_SERVICE_DISCOVERING,
    APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE,
} app_gap_state_t;

typedef struct {
    bool dev_found;
    uint8_t bdname_len;
    uint8_t eir_len;
    uint8_t rssi;
    uint32_t cod;
    uint8_t eir[ESP_BT_GAP_EIR_DATA_LEN];
    uint8_t bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
    esp_bd_addr_t bda;
    app_gap_state_t state;
} app_gap_cb_t;

static app_gap_cb_t m_dev_info;

#else

#pragma message ("unknow build enviroment")
#error
#endif

// Board dependencies
#if defined(BOARD_HELTEC)
#define I2CSDA 4  //D2 gruen
#define I2CSCL 15  //D1 gelb
#endif

#include <PubSubClient.h>
#include <ArduinoJson.h>

// #define BME680Bosch true

// Sensor Libraries
#include <Wire.h>
#include "Adafruit_Si7021.h"
#include "Adafruit_BME280.h"
#if defined(BME680Bosch)
#include "bsec.h"
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_28d/bsec_iaq.txt"
};

#else
#include "Adafruit_BME680.h"
#endif
#include "Adafruit_TSL2561_U.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_VEML6070.h"
#include "Adafruit_ADS1015.h"
#include "Adafruit_VL53L0X.h"
#include "Adafruit_TCS34725.h"
#include "Adafruit_CCS811.h"
#include <U8x8lib.h>
#include <SparkFun_AS3935.h>
#include <Max44009.h>

// Global defines
#define NEOPIXEL 14 //D5

#define AS3935_ADDR 0x03

// Global Objects
uint8_t nrofleds = 1;
neoPixelType ledtype = NEO_GRB + NEO_KHZ800;
bool has_white = false;

Adafruit_Si7021 si7021;
Adafruit_BME280 bme280;
#if defined(BME680Bosch)
Bsec iaqSensor;
void checkIaqSensorStatus(void);
#else
Adafruit_BME680 bme680;
#endif
Adafruit_CCS811 ccs;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);
Adafruit_VEML6070 veml = Adafruit_VEML6070();
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_ADS1115 ads1115;
SparkFun_AS3935 as3935(AS3935_ADDR);
Adafruit_NeoPixel led = Adafruit_NeoPixel(nrofleds, NEOPIXEL, ledtype);
Adafruit_TCS34725 tcs = Adafruit_TCS34725();
Max44009 gy49(0x4a);
WiFiClient espClient;
PubSubClient client;
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
int pirState = LOW;
uint8_t pirInput = 12;
uint8_t as3935_input = 13; // D7 hardware int pin for AS3935
uint8_t as3935_tunecap = 96;
bool as3935_inconfig = false;


unsigned transmission_delay = 60; // seconds
uint32_t led_current_color;

// Strings for dynamic config
String Smyname, Spass, Sssid, Smqttserver, Ssite, Sroom, Smqttuser, Smqttpass;
unsigned int Imqttport;
bool Bflipped;
bool Bindoor = true;
bool Bco2alert = false; // CO2-Ampel


// Flags for sensors found
bool si7021_found = false;
bool bme280_found = false;
bool bme680_found = false;
bool tsl2561_found= false;
bool voltage_found= true;
bool ccs811_found = false;
bool u8x8_found = false;
bool veml_found = false;
bool lox_found = false;
bool tcs_found = false;
bool ads1115_found = false;
bool rtc_init_done = false;
bool rtc_alarm_raised = false;
bool light_on = true;
bool pir_found = true;
bool as3935_found = false;
bool gy49_found = false;

bool color_watch = false;

// Floats for calibration
float bme280TempAdjust = 0.0;
float bme680TempAdjust = 0.0;
float si7021TempAdjust = 0.0;

// Flags for display
#define DISPLAY_OFF 0
#define DISPLAY_TEMPERATURE 1
#define DISPLAY_HUMIDITY 2
#define DISPLAY_AIRPRESSURE 3
#define DISPLAY_LUX 4
#define DISPLAY_STRING 5
#define DISPLAY_DISTANCE 6
#define DISPLAY_TEMPHUM 7
#define DISPLAY_CO2 8


unsigned int display_what = DISPLAY_TEMPHUM;
unsigned int display_brightness = 1;
bool display_refresh = true;

// Timer variables
unsigned long now;
unsigned long last_transmission = 0;
unsigned long last_display = 0;

// forward declarations
boolean setup_wifi();
void loop_publish_tcs34725();
void publish_status();
void publish_as3935();
void publish_bme280();
void publish_bme680();
void publish_si7021();


// LED routines

uint32_t toColor(byte r, byte g, byte b) {
  byte w = 0;
  if (has_white && r==g && r==b) {
    if (r<255) {
      w = r;
      r = 0; g = 0; b = 0;
    } else {
      w = 255;
    }
  }
  return b + (g<<8) + (r<<16) + (w<<24);
}

uint32_t toColor(byte r, byte g, byte b, byte w) {
  return  b + (g<<8) + (r<<16) + (w<<24);
}

void setallleds(byte r, byte g, byte b) {
  led.fill(toColor(r,g,b),0,nrofleds);
  if (light_on) {
    led.show();
  }
}

void setled(byte r, byte g, byte b) {
  if (has_white && r==g && r==b) {
    if (r<255)
      led.setPixelColor(0, 0, 0, 0, r);
    else
      led.setPixelColor(0, 255, 255, 255, 255);
  } else {
    led.setPixelColor(0, r, g, b);
  }
  led_current_color = led.Color(r,g,b);
  if (light_on) {
    led.show();
  }
}

void setled(byte n, byte r, byte g, byte b) {
  if (has_white && r==g && r==b) {
    if (r < 255)
      led.setPixelColor(n, 0, 0, 0, r);
    else
      led.setPixelColor(n, 255, 255, 255, 255);
  } else {
    led.setPixelColor(n, r, g, b);
  }
  if (n == 0)
    led_current_color = led.Color(r,g,b);
  if (light_on) {
    led.show();
  }
}

void setled(byte n, byte r, byte g, byte b, byte show) {
  led.setPixelColor(n, r, g, b);
  if (light_on && show) {
    led.show();
  }
}

void setled(byte show) {
  if (!show) {
    int i;
    for (i = 0; i < nrofleds; i++) {
      setled(i, 0, 0, 0, 0);
    }
  }
  led.show();
}

// Debug functions
void log_config () {

  Log.verbose("Smyname = %s",Smyname.c_str());
  Log.verbose("Ssite = %s",Ssite.c_str());
  Log.verbose("Sroom = %s",Sroom.c_str());
  Log.verbose("Sssid = %s",Sssid.c_str());
  Log.verbose("Spass = %s",Spass.c_str());
  Log.verbose("Smqttuser = %s",Smqttuser.c_str());
  Log.verbose("Smqttpass = %s",Smqttpass.c_str());
  Log.verbose("Imqttport = %d",Imqttport);
  Log.verbose(F("Bflipped = %t"),Bflipped);
  Log.verbose(F("Bindoor = %t"),Bindoor);
  Log.verbose(F("Display = %d"),display_what);
  Log.verbose(F("Brightness = %d"),display_brightness);
  if (pir_found)
    Log.verbose(F("Motion detector = %d"),pirInput);
  Log.verbose(F("AS3935 int pin = %d"),as3935_input);
  Log.verbose(F("AS3935 tune cap = %d"),as3935_tunecap);
  Log.verbose(F("nrofleds = %d"),nrofleds);
  Log.verbose(F("ledtype = %x"),ledtype);
  Log.verbose(F("CO2alert = %t"),Bco2alert);

}

void write_config () {
  StaticJsonDocument<512> doc;
  doc["myname"] = Smyname;
  JsonObject display = doc.createNestedObject("display");
  display["flipped"] = Bflipped;
  display["brightness"] = display_brightness;
  display["content"] =display_what;
  JsonObject network = doc.createNestedObject("network");
  network["pass"] = Spass;
  network["ssid"] = Sssid;
  JsonObject mqtt = doc.createNestedObject("mqtt");
  mqtt["server"] = Smqttserver;
  mqtt["user"] = Smqttuser;
  mqtt["pass"] = Smqttpass;
  mqtt["port"] = Imqttport;
  JsonObject location = doc.createNestedObject("location");
  location["site"] = Ssite;
  location["room"] = Sroom;
  location["indoor"] = Bindoor;

  JsonObject sensors = doc.createNestedObject("sensors");

  if (bme280_found) {
    JsonObject bme280 = sensors.createNestedObject("bme280");
    bme280["tempadjust"] = bme280TempAdjust;
  }

  if (bme680_found) {
    JsonObject bme680 = sensors.createNestedObject("bme680");
    bme680["tempadjust"] = bme680TempAdjust;
  }

  if (si7021_found) {
    JsonObject si7021 = sensors.createNestedObject("si7021");
    si7021["tempadjust"] = si7021TempAdjust;
  }

  if (pir_found) {
    JsonObject pir;
    pir = sensors.createNestedObject("pir");
    pir["pin"] = pirInput;
  }
  doc["as3935_input"] = as3935_input;
  doc["as3935_tunecap"] = as3935_tunecap;

  JsonObject leds;
  leds = doc.createNestedObject("led");
  leds["count"] = nrofleds;
  leds["type"] = ledtype;

  doc["co2alert"] = Bco2alert;


  Log.notice(F("Writing new config file"));
  serializeJsonPretty(doc,Serial);

  SPIFFS.begin();
  File f = SPIFFS.open("/config.json","w");
  if (!f) {
    Log.error(F("Open of config file for writing failed"));
  } else {
    if (serializeJson(doc,f)) {
      Log.notice(F("Written new config. Now reboot"));
    } else {
      Log.error(F("Writing object into file failed"));
    }
    f.close();
  }
SPIFFS.end();
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

// MQTT main callback routines
void mqtt_callback(char* topic, byte* payload, unsigned int length)  {

  String in[10];
  unsigned int wordcounter = 0;

  for (unsigned int i = 0; i < length; i++) {
    if ((char)payload[i] == ' ' && wordcounter < 9) {
      wordcounter++;
    } else {
      in[wordcounter] += String((char)payload[i]);
    }
  }
  Log.verbose("Message arrived[%s]: %d Words",topic,wordcounter);
  for (unsigned int i=0; i <= wordcounter; i++)
    Log.verbose("Word[%d] = %s",i,in[i].c_str());

  if (in[0] == "reboot") {
    ESP.restart();
  }

  if (in[0] == "status") {
    publish_status();
  }

  if (in[0] == "led") {
    if (wordcounter == 3) {
      // led r g b
      setled(in[1].toInt(),in[2].toInt(),in[3].toInt());
    } else if (wordcounter == 4) {
      setled(in[1].toInt(),in[2].toInt(),in[3].toInt(),in[4].toInt());
    }
  }

  if (in[0] == "ledb") {
    // led in binary code
    int position = 5;
    int thisled = 0;
    while ((position < length-3) && (thisled < nrofleds)) {
      // 3 bytes per led: red green blue
      setled(thisled,
        payload[position],
        payload[position+1],
        payload[position+2]);
      position +=3;
      thisled++;
    }
  }

  if (in[0] == "leds") {
    // apply to all leds
    if (wordcounter == 3) {
      setallleds(in[1].toInt(),in[2].toInt(),in[3].toInt());
    }
  }

  if (in[0] == "sensor") {
    // sensor name command
    if (wordcounter >= 1) {
      if (tcs_found && in[1] == "color") {
        if (wordcounter == 1) {
          loop_publish_tcs34725();
        }
        if (wordcounter == 2) {
          if (in[2] == "enable")
            color_watch = 1;
          if (in[2] == "disable")
            color_watch = 0;
        }
        if (wordcounter == 3) {
          if (in[2] == "gain") {
            tcs.setGain((tcs34725Gain_t) atoi(in[3].c_str()));
          }
          if (in[2] == "time") {
            tcs.setIntegrationTime((tcs34725IntegrationTime_t) atoi(in[3].c_str()));
            tcs.begin();
          }
          if (in[2] == "interrupt") {
            tcs.setInterrupt(in[3] == "on");

          }
        }
      }
      if (as3935_found && in[1] == "as3935") {
        if (wordcounter == 1) {
          publish_as3935();
        }
        if (wordcounter == 3) {
          if (in[2] == "tunecap") {
            as3935.tuneCap((uint8_t) atoi(in[3].c_str()));
            delay(50);
            as3935_tunecap = as3935.readTuneCap();
            Log.verbose(F("as3935 tuning capacitor now %d"),
              as3935_tunecap);
          }
        }
      }
      if (bme280_found && in[1] == "bme280") {
        if (wordcounter == 1) {
          publish_bme280();
        }
        if (wordcounter == 3 && in[2] == "tempadjust") {
          bme280.setTemperatureCompensation(atof(in[3].c_str()));
          bme280TempAdjust = bme280.getTemperatureCompensation();
          publish_bme280();
        }
      }

      if (bme680_found && in[1] == "bme680") {
        if (wordcounter == 1) {
          publish_bme680();
        }
        if (wordcounter == 3 && in[2] == "tempadjust") {
          bme680TempAdjust = atof(in[3].c_str());
          #if defined (BME680Bosch)
          iaqSensor.setTemperatureOffset(-1.0 * bme680TempAdjust);
          #endif
          publish_bme680();
        }
      }


      if (si7021_found && in[1] == "si7021") {
        if (wordcounter == 1) {
          publish_si7021();
        }
        if (wordcounter == 3 && in[2] == "tempadjust") {
          si7021TempAdjust = atof(in[3].c_str());
          publish_si7021();
        }
      }
    }
  }

  if (in[0] == F("config")) {
    if (wordcounter == 0) {
      log_config();
    }
    if (wordcounter == 1) {
      if (in[1] == F("write")) {
        log_config();
        write_config();
      }
    }
    if (wordcounter == 2) {
      if (in[1] == F("room")) {
        Sroom = in[2];
      }
      if (in[1] == F("site")) {
        Ssite = in[2];
      }
      if (in[1] == F("myname")) {
        Smyname = in[2];
      }
      if (in[1] == F("mqttuser")) {
        Smqttuser = in[2];
      }
      if (in[1] == F("mqttpass")) {
        Smqttpass = in[2];
      }
    }
  }

  if (u8x8_found && (in[0] == "display") && (wordcounter >= 1)) {
    last_display = 0;
    display_refresh = true;
    if (in[1] == "humidity") {
      display_what = DISPLAY_HUMIDITY;
    } else if (in[1] == "airpressure") {
      display_what = DISPLAY_AIRPRESSURE;
    } else if (in[1] == F("temperature")) {
      display_what = DISPLAY_TEMPERATURE;
    } else if (in[1] == F("distance")) {
      display_what = DISPLAY_DISTANCE;
    } else if (in[1] == F("temphum")) {
      display_what = DISPLAY_TEMPHUM;
    } else if (in[1] == F("co2")) {
      display_what = DISPLAY_CO2;
    } else if (in[1] == "off") {
      u8x8.clearDisplay();
      display_what = DISPLAY_OFF;
    } else if (in[1] == F("flip")) {
      u8x8.clearDisplay();
      u8x8.setFlipMode(wordcounter > 1);
      Bflipped = (wordcounter > 1);
    } else if (in[1] == F("brightness") && wordcounter >= 2) {
      display_brightness = in[2].toInt();
      Log.verbose(F("Setting brightness to %d"),display_brightness);
      u8x8.setContrast(display_brightness);
    } else { // String
      // large or small?
      // small, if a line is longer than 8 chars or if there are more than 3 lines
      boolean large = true;
      int start;
      for (unsigned int i=1; i<=wordcounter;i++) {
        if (in[i].length() > 8)
          large = false;
      }
      if (wordcounter > 3)
        large = false;

      if (wordcounter == 1)
        start = 3;
      else
        start = 0;

      display_what = DISPLAY_STRING;
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
      u8x8.clearDisplay();

      for (unsigned int i=1; i <= wordcounter; i++) {
        if (large) {
          u8x8.draw2x2UTF8(0, start, in[i].c_str());
          start += 3;
        } else {
          u8x8.draw1x2UTF8(0, start, in[i].c_str());
          start += 2;
        }
      }
    }
  }
}

boolean mqtt_reconnect() {
  // Loop until we're reconnected
  char mytopic[50];
  snprintf(mytopic, 50, "/%s/%s/status", Ssite.c_str(), Sroom.c_str());

  if (WiFi.status() != WL_CONNECTED) {
    if (!setup_wifi())
      return false;
  }

  Log.verbose("Attempting MQTT connection...%d...",client.state());

  // Attempt to connect
  if (client.connect(Smyname.c_str(),Smqttuser.c_str(),Smqttpass.c_str(),mytopic,0,0,"stopped")) {
    Log.verbose("MQTT connected");

    client.publish(mytopic, "started");
    delay(10);
    // ... and resubscribe to my name
    client.subscribe(Smyname.c_str());
    // and to my location
    String mylocation = "/" + Ssite + "/cmd";
    client.subscribe(mylocation.c_str());
    String myroom = "/" + Ssite + "/" + Sroom + "/cmd";
    client.subscribe(myroom.c_str());
    delay(10);
  } else {
    Log.error("MQTT connect failed, rc=%d",client.state());
  }
  return client.connected();
}

unsigned long lastReconnectAttempt = 0;
void mqtt_publish(char *topic, char *msg) {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (mqtt_reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  client.loop();


  char mytopic[50];
  snprintf(mytopic, 50, "/%s/%s/%s", Ssite.c_str(), Sroom.c_str(),topic);
  Log.verbose("MQTT Publish message [%s]:%s",mytopic,msg);

  client.publish(mytopic, msg);
}

void mqtt_publish(const __FlashStringHelper *topic, const __FlashStringHelper *msg) {
  char a[64],b[64];
  snprintf(a,63,"%s",topic);
  snprintf(b,63,"%s",msg);
  mqtt_publish(a,b);
}

void mqtt_publish(const __FlashStringHelper *topic, char *msg) {
  char a[64];
  snprintf(a,63,"%s",topic);
  mqtt_publish(a,msg);
}

void mqtt_publish(const __FlashStringHelper *topic, const char *msg) {
  char buf[64];
  snprintf(buf,63,"%s",msg);
  mqtt_publish(topic, buf);
}


void mqtt_publish(char *topic, int i) {
  char buf[15];
  snprintf(buf,14,"%d",i);
  mqtt_publish(topic, buf);
}

void mqtt_publish(char *topic, uint32_t i) {
  char buf[32];
  snprintf(buf,31,"%lu",i);
  mqtt_publish(topic,buf);
}

void mqtt_publish(const __FlashStringHelper *topic, uint32_t i) {
  char buf[32];
  snprintf(buf,31,"%lu",i);
  mqtt_publish(topic,buf);
}

void mqtt_publish(char *topic, float value) {
  char buf[15];
  snprintf(buf,14,"%.3f",value);
  mqtt_publish(topic, buf);
}

void mqtt_publish(const __FlashStringHelper *topic, float value) {
  char buf[15];
  snprintf(buf,14,"%.3f",value);
  mqtt_publish(topic, buf);
}



void mqtt_influx(char *field, char *value) {
  char out[128];

  sprintf(out,"sensor,site=%s,room=%s %s=%s",
    Ssite.c_str(), Sroom.c_str(),field,value);
  mqtt_publish("influx",out);
}

void mqtt_influx(char *field, int i) {
  char buf[15];
  snprintf(buf,14,"%di",i);
  mqtt_influx(field,buf);
}

void mqtt_influx(char *field, float f) {
  char buf[15];
  snprintf(buf,14,"%.3f",f);
  mqtt_influx(field,buf);
}

void mqtt_influx(char *field, uint32_t i) {
  char buf[32];
  snprintf(buf,31,"%lu",i);
  mqtt_influx(field,buf);
}

void mqtt_influx(char *field, bool b) {
  if (b)
    mqtt_influx(field,"true");
  else
    mqtt_influx(field,"false");
}

void mqtt_influx(const __FlashStringHelper *topic, const __FlashStringHelper *msg) {
  char a[32],b[32];
  snprintf(a,31,"%s",topic);
  snprintf(b,31,"%s",msg);
  mqtt_influx(a,b);
}

void mqtt_influx(const __FlashStringHelper *topic, char *msg) {
  char a[32];
  snprintf(a,31,"%s",topic);
  mqtt_influx(a,msg);
}

void mqtt_influx(const __FlashStringHelper *topic, const char *msg) {
  char buf[32];
  snprintf(buf,31,"%s",msg);
  mqtt_influx(topic, buf);
}

void mqtt_influx(const __FlashStringHelper *topic, uint32_t i) {
  char buf[32];
  snprintf(buf,31,"%lu",i);
  mqtt_influx(topic,buf);
}

void mqtt_influx(const __FlashStringHelper *topic, float f) {
  char buf[32];
  snprintf(buf,31,"%.3f",f);
  mqtt_influx(topic,buf);
}


// Setup routines
//
// Scan for sensors
//
void setup_i2c() {
  byte error, address;

// 0x03 AS3935 Lightning Detector
// 0x29 TSL45315 (Light)
// 0x29  Color
// 0x38 VEML6070 (Light)
// 0x39 TSL2561
// 0x3c Display
// 0x40 SI7021
// 0x48 4*AD converter ADS1115
// 0x4a GY49 or MAX44009 Light Sensor
// 0x50 PCF8583P
// 0x57 ATMEL732
// 0x5a CCS811 Gas Sensor
// 0x68 DS3231 Clock
// 0x76 BME280
// 0x77 BME680 (also BMP180)


  Log.notice("Scanning i2c bus");
  Wire.begin(I2CSDA, I2CSCL);
  Wire.setClock(10000);

  // Try for DISPLAY
#if defined(BOARD_HELTEC)
  pinMode(16,OUTPUT);
  digitalWrite(16,LOW);
  delay(100);
  digitalWrite(16,HIGH);
#endif

  for(address = 1; address < 127; address++ ) {
    Log.verbose(F("Trying %d"),address);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    // force ok for Lightning sensor
    if (address == AS3935_ADDR && error != 0 && as3935_inconfig) {
      // force as3935 found
      Log.trace("AS3935 forced");
      error = 0;
    }

    if (error == 0) {
      Log.trace("I2C device found at address 0x%x",address);

      if (address == AS3935_ADDR) {
        as3935_found = as3935.begin();
        Log.notice(F("AS3935 found? %T"),as3935_found);
        if (as3935_found) {
          // initialize it
          // Are we indoor or outdoor?
          if (Bindoor) {
            as3935.setIndoorOutdoor(INDOOR);
          } else {
            as3935.setIndoorOutdoor(OUTDOOR);
          }
          // set default values (tune code later)
          as3935.maskDisturber(false);
          as3935.setNoiseLevel(1);
          as3935.watchdogThreshold(1);
          as3935.spikeRejection(2);
          as3935.lightningThreshold(5);
          as3935.tuneCap(as3935_tunecap);
          as3935.clearStatistics(true);
        }
      }

      if (address == 0x29) {
        tcs_found = tcs.begin();
        Log.notice(F("TCS found? %T"),tcs_found);

        if (!tcs_found) {
          lox_found = lox.begin();
          Log.notice(F("LOX found? %T"),lox_found);
        }
      }

      if (address == 0x39) {
        tsl2561 = Adafruit_TSL2561_Unified(address);
        tsl2561_found = tsl2561.begin();
        Log.notice("TSL2561 found? %T",tsl2561_found);
        if (tsl2561_found) {
          // init the sensor
          tsl2561.enableAutoRange(true);
          // tsl2561.setGain(TSL2561_GAIN_1X);
          tsl2561.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
        }
      }

      if (address == 0x38) {
        // VEML6070
        veml.begin(VEML6070_1_T);
        if (veml.readUV() != -1) {
          veml_found = true;
        }
        Log.notice(F("VEML6070 found? %T"), veml_found);
      }

      if (address == 0x3c) {
        u8x8_found = u8x8.begin();
        Log.notice("U8xu found? %T",u8x8_found);
        if (u8x8_found) {
          u8x8.clear();
          u8x8.setFont(u8x8_font_chroma48medium8_r);
          u8x8.setFlipMode(Bflipped);
        }
      }
      if (address == 0x40) {
        // SI7021
        si7021 = Adafruit_Si7021();
        si7021_found = si7021.begin();
        Log.notice("Si7021 found? %T",si7021_found);
      }

      if (address == 0x4a) {
        // GY-49
        float l = gy49.getLux();
        if (! gy49.getError()) {
          gy49_found = true;
          Log.notice(F("GY49 found at 0x%x"),address);
        }
      }

      if ((address >= 0x48) && (address <= 0x4b)) {
        ads1115 = Adafruit_ADS1115(address);
        ads1115.begin();
        ads1115_found = true;
        Log.notice(F("ADS1115 found at 0x%x"),address);
      }

      if (address == 0x5a) {
        // CCS811 Gas Sensor
        if (ccs.begin()){
          ccs811_found = true;
          Log.notice(F("CCS811 found at 0x%x"),address);
          // driver mode 60 seconds is enough
          if (Bco2alert)
            ccs.setDriveMode(CCS811_DRIVE_MODE_1SEC);
          else
            ccs.setDriveMode(CCS811_DRIVE_MODE_60SEC);
        }
      }
      if (address == 0x76 || address == 0x77) {
        // BME280 or BME680
        #if defined(BME680Bosch)
        iaqSensor.begin(address,Wire);
        checkIaqSensorStatus();
        if (bme680_found) {
          Log.notice("BME680 found using Bosch? %T at 0x%x",bme680_found,address);
          bsec_virtual_sensor_t sensorList[12] = {
            BSEC_OUTPUT_IAQ,
            BSEC_OUTPUT_CO2_EQUIVALENT,
            BSEC_OUTPUT_STABILIZATION_STATUS,
            BSEC_OUTPUT_RUN_IN_STATUS,
            BSEC_OUTPUT_RAW_TEMPERATURE,
            BSEC_OUTPUT_RAW_PRESSURE,
            BSEC_OUTPUT_RAW_HUMIDITY,
            BSEC_OUTPUT_RAW_GAS,
            BSEC_OUTPUT_COMPENSATED_GAS,
            BSEC_OUTPUT_GAS_PERCENTAGE,
            BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
            BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY
          };
          iaqSensor.updateSubscription(sensorList, 12, BSEC_SAMPLE_RATE_LP);
          iaqSensor.setTemperatureOffset(-1.0 * bme680TempAdjust);
        #else // Adafruit
        bme680_found = bme680.begin(address);
        if (bme680_found) {
          bme680.setTemperatureOversampling(BME680_OS_8X);
          bme680.setHumidityOversampling(BME680_OS_2X);
          bme680.setPressureOversampling(BME680_OS_4X);
          bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
          bme680.setGasHeater(320, 150); // 320*C for 150 ms
          Log.notice("BME680 found using Adafruit? %T at 0x%x",bme680_found,address);
        #endif
        } else {
          bme280_found = bme280.begin(address);
          Log.notice("BME280 found? %T at 0x%x",bme280_found,address);
          bme280.setTemperatureCompensation(bme280TempAdjust);
          Log.notice("BME280 adjustment %F",bme280TempAdjust);
        }
      }
    }
  }
  Log.notice("End scanning i2c bus");
}

void setup_serial() {
  delay(1000);
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
  has_white = (ledtype >> 6) != ((ledtype >> 4) & 0x03);
}

// setup PIR motion detector
void setup_pir() {
  pinMode(pirInput, INPUT);
  pirState = digitalRead(pirInput);
}

// read the config file and parse its data
void setup_readconfig() {
  SPIFFS.begin();
  File f = SPIFFS.open("/config.json","r");
  if (!f) {
    Log.error("Cannot open config file");
    return;
  }
  StaticJsonDocument<1024> jsonBuffer;

 // Parse the root object
 // JsonObject root = jsonBuffer.parseObject(f);

 auto error = deserializeJson(jsonBuffer,f);
 if (error) {
   Log.error(F("Failed to read file %s"),error.c_str());
 } else {
   serializeJsonPretty(jsonBuffer,Serial);
 }

 JsonObject root = jsonBuffer.as<JsonObject>();

 // Copy values from the JsonObject to the Config
   Smyname = root["myname"].as<String>();
   Bflipped = root["display"]["flipped"];
   display_what = root["display"]["content"];
   display_brightness=root["display"]["brightness"];
   Spass = root["network"]["pass"].as<String>();
   Sssid = root["network"]["ssid"].as<String>();
   Smqttserver = root["mqtt"]["server"].as<String>();
   Ssite = root["location"]["site"].as<String>();
   Sroom = root["location"]["room"].as<String>();
   Bindoor = root["location"]["indoor"];
   Smqttuser = root["mqtt"]["user"].as<String>();
   Smqttpass = root["mqtt"]["pass"].as<String>();
   Imqttport = root["mqtt"]["port"];

   bme280TempAdjust = root["sensors"]["bme280"]["tempadjust"];
   bme680TempAdjust = root["sensors"]["bme680"]["tempadjust"];
   si7021TempAdjust = root["sensors"]["si7021"]["tempadjust"];

   if (as3935_input = root["as3935_input"]) {
     as3935_inconfig = true;
   }
   if (nrofleds = root["led"]["count"]) {
     led.updateLength(nrofleds);
   }
   if (ledtype = root["led"]["type"]) {
     led.updateType(ledtype);
     has_white = (ledtype >> 6) != ((ledtype >> 4) & 0x03);
   }

   Bco2alert = root["co2alert"];

   as3935_tunecap = root["as3935_tunecap"];
   if (pirInput = root["sensors"]["pir"]["pin"]) {
     pir_found = true;
   } else {
     pir_found = false;
   }

  f.close();
  SPIFFS.end();
}

boolean setup_wifi() {
  WiFi.persistent(false);
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_STA);
#if defined(ARDUINO_ARCH_ESP8266)
  WiFi.hostname(Smyname);
#endif
  WiFi.begin(Sssid.c_str(), Spass.c_str());

  int retries = 0;
  bool ipv6found = false;
  bool ipv4found = false;

  #if LWIP_IPV6 && defined(ARDUINO_ARCH_ESP8266)
  for (bool configured = false; !configured;) {
    for (auto addr : addrList) {
      if (addr.isV6() && !addr.isLocal()) {
        ipv6found = true;
      }
      if (addr.isV4()) {
        ipv4found = true;
      }
      delay(500);
      configured = (ipv4found && ipv6found);
    }
  }
  #else
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    retries++;
    Log.error(F("Wifi.status() = %d"),WiFi.status());
    if (retries > 20) {
      Log.error(F("Cannot connect to %s"),Sssid.c_str());;
      return ipv6found;
    }
  }
  String myIP = String(WiFi.localIP().toString());
  String myMask = String(WiFi.subnetMask().toString());
  Log.verbose("Wifi connected as %s/%s",myIP.c_str(),myMask.c_str());
#endif
  return true;
}

void setup_mqtt() {
  client.setClient(espClient);
  client.setServer(Smqttserver.c_str(), Imqttport);
  client.setCallback(mqtt_callback);

}


// ESP32 Specific Setup routines
#if defined(ARDUINO_ARCH_ESP32)

char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

void update_device_info(esp_bt_gap_cb_param_t *param)
{
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    esp_bt_gap_dev_prop_t *p;

    Log.verbose("Device found: %s", bda2str(param->disc_res.bda, bda_str, 18));
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            Log.verbose("--Class of Device: 0x%x", cod);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            Log.verbose("--RSSI: %d", rssi);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
        default:
            break;
        }
    }

    /* search for device with MAJOR service class as "rendering" in COD */
    app_gap_cb_t *p_dev = &m_dev_info;
    if (p_dev->dev_found && 0 != memcmp(param->disc_res.bda, p_dev->bda, ESP_BD_ADDR_LEN)) {
        return;
    }

    if (!esp_bt_gap_is_valid_cod(cod) ||
            !(esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_PHONE)) {
        return;
    }

    memcpy(p_dev->bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
    p_dev->dev_found = true;
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            p_dev->cod = *(uint32_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            p_dev->rssi = *(int8_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME: {
            uint8_t len = (p->len > ESP_BT_GAP_MAX_BDNAME_LEN) ? ESP_BT_GAP_MAX_BDNAME_LEN :
                          (uint8_t)p->len;
            memcpy(p_dev->bdname, (uint8_t *)(p->val), len);
            p_dev->bdname[len] = '\0';
            p_dev->bdname_len = len;
            break;
        }
        case ESP_BT_GAP_DEV_PROP_EIR: {
            memcpy(p_dev->eir, (uint8_t *)(p->val), p->len);
            p_dev->eir_len = p->len;
            break;
        }
        default:
            break;
        }
    }

    if (p_dev->eir && p_dev->bdname_len == 0) {
        get_name_from_eir(p_dev->eir, p_dev->bdname, &p_dev->bdname_len);
        Log.verbose("Found a target device, address %s, name %s", bda_str, (char *)(p_dev->bdname));
        // p_dev->state = APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE;
        // Log.verbose("Cancel device discovery ...");
        // esp_bt_gap_cancel_discovery();
    }
}


void callback_esp32_gap(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  char bda_str[18];
  char uuid_str[37];

  switch (event) {
    case 1:
      break;
    case ESP_BT_GAP_DISC_RES_EVT:
      sprintf(bda_str,"%02x:%02x:%02x:%02x:%02x:%02x",
        param->disc_res.bda[0],
        param->disc_res.bda[1],
        param->disc_res.bda[2],
        param->disc_res.bda[3],
        param->disc_res.bda[4],
        param->disc_res.bda[5]);
      Log.notice(F("BT Device found %s"),bda_str);
      update_device_info(param);

      break;
    default:
      Log.verbose(F("BT event %d"),event);
      break;
  }
}



void setup_esp32_bluetooth() {
  if (!btStart()) {
    Log.error(F("Failed to initialize bluetooth"));
    return;
  }

  if (esp_bluedroid_init() != ESP_OK) {
    Log.error(F("Failed to initialize bluedroid"));
    return;
  }

  if (esp_bluedroid_enable() != ESP_OK) {
    Log.error(F("Failed to enable bluedroid"));
    return;
  }

  esp_bt_dev_set_device_name(Smyname.c_str());
  esp_bt_gap_register_callback(callback_esp32_gap);
}

void setup_esp32() {
  setup_esp32_bluetooth();
}
#endif

void setup() {
  setup_led();
  setled(255,0,0);
  delay(5000);
  setup_serial();
  setup_logging();
  setup_readconfig();
  log_config();
  pinMode(as3935_input, INPUT);
  setup_i2c();
  if (pir_found)
    setup_pir();
#ifdef ARDUINO_ARCH_ESP32
  setup_esp32();
#endif
  setled(255, 128, 0);
  if (setup_wifi()) {
    setup_mqtt();
    setled(0, 255, 0);
    delay(1000);
    setled(0,0,0);
  } else {
    setled(2,1,0);
  }
  publish_status();
  if (as3935_found)
    pinMode(as3935_input, INPUT);
}

void publish_as3935() {
  String value;
  const __FlashStringHelper *s = F("status/sensors/as3935");
  if (as3935.readMaskDisturber()) {
    mqtt_publish(s,F("disturbers masked"));
  } else {
    mqtt_publish(s,F("disturbers NOT masked"));
  }
  if (as3935.readIndoorOutdoor() == INDOOR) {
    mqtt_publish(s,F("INDOOR"));
  } else {
    mqtt_publish(s,F("OUTDOOR"));
  }
  char buf[32];

  snprintf(buf,31,"%s %d",F("Watchdog Threshold"),
      (int)(as3935.readWatchdogThreshold()));
  mqtt_publish(s,buf);

  snprintf(buf,31,"%s %d",F("Noise Level"),
      (int)(as3935.readNoiseLevel()));
  mqtt_publish(s,buf);

  snprintf(buf,31,"%s %d",F("Spike Rejection"),
      (int)(as3935.readSpikeRejection()));
  mqtt_publish(s,buf);

  snprintf(buf,31,"%s %d",F("Lightning numbers threshold"),
      (int)(as3935.readLightningThreshold()));
  mqtt_publish(s,buf);

  snprintf(buf,31,"%s %d",F("Tuning capacitor"),
      (int)(as3935.readTuneCap()));
  mqtt_publish(s,buf);


}

void publish_status() {
  const __FlashStringHelper *status = F("status/sensors"),
   *net = F("status/network");
  String ipv4, ipv6, linkLocal;


  if (si7021_found) mqtt_publish(status,F("si7021_found"));
  if (bme280_found) mqtt_publish(status,F("bme280_found"));
  if (bme680_found) mqtt_publish(status,F("bme680_found"));
  if (tsl2561_found) mqtt_publish(status,F("tsl2561_found"));
  if (voltage_found) mqtt_publish(status,F("voltage_found"));
  if (u8x8_found) mqtt_publish(status,F("u8x8_found"));
  if (veml_found) mqtt_publish(status,F("veml_found"));
  if (lox_found) mqtt_publish(status,F("lox_found"));
  if (tcs_found) mqtt_publish(status,F("tcs_found"));
  if (ads1115_found) mqtt_publish(status,F("ads1115_found"));
  if (ccs811_found) mqtt_publish(status,F("ccs811_found"));
  if (pir_found) mqtt_publish(status,F("pir_found"));
  if (as3935_found) {
    mqtt_publish(status, F("as3935_found"));
    publish_as3935();
  }
#if defined(ARDUINO_ARCH_ESP8266)
  mqtt_publish(F("status/ChipID"),(uint32_t)ESP.getChipId());
  mqtt_publish(F("status/ResetReason"),ESP.getResetReason().c_str());
  mqtt_publish(F("status/SdkVersion"),ESP.getSdkVersion());
  mqtt_publish(F("status/SketchSize"),ESP.getSketchSize());
  mqtt_publish(F("status/CycleCount"),ESP.getCycleCount());
#endif
  mqtt_publish(F("status/CompileTime"), compile_date);

#if LWIP_IPV6 && defined(ARDUINO_ARCH_ESP8266)
for (auto a : addrList) {
  a.isV6() ? a.isLocal() ? linkLocal = a.toString() : ipv6 = a.toString() : ipv4 = a.toString();

  Log.verbose(F("IF=%s index=%d legacy=%d IPv4=%d local=%d hostname='%s' addr= %s"),
                a.ifname().c_str(),
                a.ifnumber(),
                a.isLegacy(),
                a.addr().isV4(),
                a.addr().isLocal(),
                a.ifhostname(),
                a.addr().toString().c_str());


}

mqtt_publish(F("status/ipv4"), ipv4.c_str());
mqtt_publish(F("status/ipv6ll"), linkLocal.c_str());
mqtt_publish(F("status/ipv6"), ipv6.c_str());
#else
  mqtt_publish(F("status/ipv6"),F("disabled"));
#endif
}

void loop_publish_voltage(){
#if defined(ARDUINO_ARCH_ESP8266)
  mqtt_publish("voltage", (float)(ESP.getVcc() / 1000.0));
  mqtt_influx("voltage", (float)(ESP.getVcc() / 1000.0));
#endif
}

void loop_publish_tsl2561() {
  if (tsl2561_found) {
    sensors_event_t event;

    if (tsl2561.begin()) {
      uint16_t br, ir;
      tsl2561.getLuminosity(&br, &ir);
      uint32_t lux = tsl2561.calculateLux(br, ir);
      mqtt_publish("light",lux);
      mqtt_influx("light",lux);
    } else {
      Log.verbose("loop_publish_tsl2561: Sensor not initialized");
    }
  }
}

void publish_bme280() {
  if (bme280_found) {
    mqtt_publish(F("bme280/tempadjust"),
      (float)bme280.getTemperatureCompensation());
  }
}

void publish_bme680() {
  if (bme680_found) {
    mqtt_publish(F("bme680/tempadjust"),
      (float)bme680TempAdjust);
  }
}

void publish_si7021() {
  if (si7021_found) {
    mqtt_publish(F("si7021/tempadjust"),
      (float)si7021TempAdjust);
  }
}

void loop_publish_bme280() {
  if (bme280_found) {
    mqtt_publish("temperature", bme280.readTemperature());
    mqtt_publish("airpressure", bme280.readPressure() / 100.0F);
    mqtt_publish("humidity", bme280.readHumidity());
    mqtt_influx("temperature", bme280.readTemperature());
    mqtt_influx("airpressure", bme280.readPressure() / 100.0F);
    mqtt_influx("humidity", bme280.readHumidity());
  }
}

#if defined(BME680Bosch)
void checkIaqSensorStatus(void) {
  bme680_found = true;
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      Log.error(F("BSEC error code %d"), iaqSensor.status);
      bme680_found = false;
    } else {
      Log.warning(F("BSEC warning code %d"), iaqSensor.status);
    }
  }

  if (iaqSensor.status != BME680_OK) {
    if (iaqSensor.status < BME680_OK) {
      Log.error(F("BME680 error code %d"), iaqSensor.bme680Status);
      bme680_found = false;
    } else {
      Log.warning(F("BME680 warning code %d"), iaqSensor.bme680Status);
    }
  }

}

void loop_publish_bme680() {
  if (bme680_found) {
    if (iaqSensor.run()) {
      mqtt_publish(F("temperature"), (float)iaqSensor.temperature);
      mqtt_influx(F("temperature"), (float)iaqSensor.temperature);
      mqtt_publish(F("humidity"), (float)iaqSensor.humidity);
      mqtt_influx(F("humidity"), (float)iaqSensor.humidity);
      mqtt_publish(F("airpressure"), iaqSensor.pressure / 100.0F);
      mqtt_influx(F("airpressure"), iaqSensor.pressure / 100.0F);
      mqtt_influx(F("gasResistance"),(float)iaqSensor.gasResistance);
      mqtt_influx(F("compGasValue"),(float)iaqSensor.compGasValue);
      if (iaqSensor.runInStatus == 1 &&
          iaqSensor.stabStatus == 1) {
        mqtt_publish(F("CO2"), iaqSensor.co2Equivalent);
        mqtt_influx(F("CO2"), iaqSensor.co2Equivalent);
        mqtt_publish(F("airquality"), iaqSensor.iaq);
        mqtt_influx(F("airquality"), iaqSensor.iaq);
        mqtt_influx(F("gasPercentage"),iaqSensor.gasPercentage);
        Log.verbose(F("  co2Ac=%d iaqAc=%d compGasAccuracy=%d gasPercentageAcccuracy=%d"),
          iaqSensor.co2Accuracy,iaqSensor.iaqAccuracy,
          iaqSensor.compGasAccuracy,
          iaqSensor.gasPercentageAcccuracy);
      }  else {
        Log.notice(F("bme60 runIn = %F stab = %F"),
                  iaqSensor.runInStatus,
                  iaqSensor.stabStatus);
        Log.verbose(F("  gasResistance=%F compGasValue=%F gasPercentage=%F compGasAccuracy=%d gasPercentageAcccuracy=%d"),
          iaqSensor.gasResistance,
          iaqSensor.compGasValue,
          iaqSensor.gasPercentage,
          iaqSensor.compGasAccuracy,
          iaqSensor.gasPercentageAcccuracy);
        Log.verbose(F("  rawtemp=%F rawHum=%F"),
          iaqSensor.rawTemperature,
          iaqSensor.rawHumidity
        );
        Log.verbose(F("  temp=%F hum=%F press=%F co2=%F iaq=%F"),
          iaqSensor.temperature,
          iaqSensor.humidity,
          iaqSensor.pressure / 100.0F,
          iaqSensor.co2Equivalent,
          iaqSensor.iaq);
        Log.verbose(F("  co2Ac=%d iaqAc=%d"),
          iaqSensor.co2Accuracy,iaqSensor.iaqAccuracy
        );
      }
    } else {
      checkIaqSensorStatus();
    }
  }


}
#else // Adafruit
void loop_publish_bme680() {
  if (bme680_found && bme680.performReading()) {
    mqtt_publish("temperature", (float) (bme680.temperature + bme680TempAdjust));
    mqtt_publish("airpressure", bme680.pressure / 100.0F);
    mqtt_publish("humidity", (float)bme680.humidity);
    mqtt_publish("airquality", bme680.gas_resistance / 1000.0F);

    mqtt_influx("temperature", (float)bme680.temperature);
    mqtt_influx("airpressure", bme680.pressure / 100.0F);
    mqtt_influx("humidity", (float)bme680.humidity);
    mqtt_influx("airquality", bme680.gas_resistance / 1000.0F);

  }
}
#endif


void loop_publish_veml6070() {
  if (veml_found) {
    mqtt_publish("UV", veml.readUV());
    mqtt_influx("UV", veml.readUV());
  }
}

void loop_publish_gy49() {
  if (gy49_found) {
    mqtt_publish(F("light"),(float)gy49.getLux());
    mqtt_influx(F("light"),(float)gy49.getLux());
  }
}

int co2ampel(int co2value) {
  if (Bco2alert) {
    if (co2value <=1000) {
      setled(0,255,0);
    } else if (co2value <= 2000) {
      setled(255,255,0);
    } else {
      setled(255,0,0);
    }

  }
  return co2value;
}

void loop_publish_ccs811() {
  if (ccs811_found) {
    if (ccs.available()) {
      if (!ccs.readData()) {
        mqtt_publish("CO2", co2ampel(ccs.geteCO2()));
        mqtt_publish("TVOC", ccs.getTVOC());
        mqtt_influx("CO2",co2ampel(ccs.geteCO2()));
        mqtt_influx("TVOC",ccs.getTVOC());
      } else {
        Log.verbose(F("Error reading data from CCS811"));
      }
    } else {
      Log.verbose(F("No data available at CCS811"));
    }
  }
}


void loop_publish_si7021() {
  if (si7021_found) {
    mqtt_publish("temperature", si7021.readTemperature() + si7021TempAdjust);
    mqtt_publish("humidity", si7021.readHumidity());
    mqtt_influx("temperature", si7021.readTemperature() + si7021TempAdjust);
    mqtt_influx("humidity", si7021.readHumidity());
  }
}

void loop_publish_tcs34725() {
  if (tcs_found) {
    uint16_t r,g,b,c;
    tcs.getRawData(&r,&g,&b,&c);

    Log.verbose(F("TCS Color raw: R=%d G=%d B=%d C=%d"),r,g,b,c);
    uint16_t minval = min(min(r,g),b);
    uint16_t maxval = max(max(r,g),b);

    int d1 = abs(r-g);
    int d2 = abs(r-b);
    int d3 = abs(g-b);

    int maxdist = max(max(d1,d2),d3);
    int mindist = min(min(d1,d2),d3);

    if (maxdist > 40) {
      r = map(r,minval,maxval,0,255);
      g = map(g,minval,maxval,0,255);
      b = map(b,minval,maxval,0,255);
    }

    setled(r,g,b);
    Log.verbose(F("TCS Color adj: R=%d G=%d B=%d C=%d"),r,g,b,c);
  }
}

static unsigned long lastReportTime = 0;
static bool lastReportState = false;

void loop_publish_motion(bool m) {
  // report motion (true or false)
  // remember when last report was and report only once a minute


//  if (lastReportTime + 60l*1000l > millis())
//    return;

  lastReportTime = millis();
  lastReportState = m;
  mqtt_influx("motion", m);
  mqtt_publish("motion", (int) m);
}

int loop_get_lox_distance() {
  if (lox_found) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure);
    if (measure.RangeStatus != 4) {
      // mqtt_influx(F("distance"), measure.RangeMilliMeter);
      return measure.RangeMilliMeter;
    }
  }
  return -1;
}

void lights_on(int dist) {
  bool x = false;

  if (dist == 1406)
    return;

 if ((dist > 0) && (dist < 500))
    x = true;

  if (x == light_on)
    return;

  Log.verbose(F("Lights on? %T %d mm"),x,dist);

  light_on = x;

  if (x) {
    led.setPixelColor(0, led_current_color);
    led.show();
    loop_publish_motion(true);
  } else {
    led_current_color = led.getPixelColor(0);
    led.clear();
    led.show();
    loop_publish_motion(false);
  }

  if (u8x8_found) {
    if (x) {
      u8x8.setContrast(255);
    } else {
      u8x8.setContrast(display_brightness);
    }
  }
}

// read PIR if it exists
void loop_pir() {
  if (pir_found) {
    int pir_old = pirState;
    pirState = digitalRead(pirInput);
    // Low -> Low
    if (pir_old == LOW && pirState == LOW)
      return;
    // Low -> High
    if (pir_old == LOW && pirState == HIGH) {
      loop_publish_motion(true);
    }

    // High -> High
    if (pir_old == HIGH && pirState == HIGH)
      return;
    // High -> Low
    if (pir_old == HIGH && pirState == LOW) {
      loop_publish_motion(false);
    }
  }
}

static int noiseCount=0;
static int totalDisturberCount=0;
static int loopcount = 0;
void loop_as3935() {
  byte watchDogVal = as3935.readWatchdogThreshold();
  if (digitalRead(as3935_input) == HIGH) {
    lightningStatus intReason = (lightningStatus) as3935.readInterruptReg();
    if (intReason == NOISE_TO_HIGH) {
      Log.verbose(F("as3935: noise %d"),++noiseCount);
      byte noiseFloor = as3935.readNoiseLevel();
      if (noiseCount > 2 && noiseFloor < 7) {
        as3935.setNoiseLevel(++noiseFloor);
        Log.verbose(F("as3935: noiseFloor %d"),noiseFloor);
      }
    }
    if (intReason == DISTURBER_DETECT) {
      Log.verbose(F("as3935: disturber %d"),++totalDisturberCount);
      if (totalDisturberCount > 2) {
        if (watchDogVal < 10) {
          as3935.watchdogThreshold(++watchDogVal);
          Log.verbose(F("as3935: watchDogVal %d"),watchDogVal);
        } else {
          as3935.maskDisturber(true);
          Log.verbose(F("as3935: Disturbers masked"));
        }
      }
    }
    if (intReason == LIGHTNING) {
      byte distance = as3935.distanceToStorm();
      uint32_t energy = as3935.lightningEnergy();
      Log.notice(F("as3935: Lightning detected in %d km, energy %u"),distance,energy);
      mqtt_publish(F("lightning/distance"), (uint32_t)distance);
      mqtt_publish(F("lightning/energy"), energy);
      mqtt_influx(F("lightning/distance"), (uint32_t)distance);
      mqtt_influx(F("lightning/energy"), energy);

    }
  }
  if (++loopcount > 10000) {
    loopcount = 0;
    if (totalDisturberCount > 2 && watchDogVal > 2) {
      --totalDisturberCount;
      --watchDogVal;
      as3935.maskDisturber(false);
      as3935.watchdogThreshold(watchDogVal);
      Log.verbose(F("as3935: WatchDogVal %d"),watchDogVal);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  client.loop();

  if (!client.connected()) {
    now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (mqtt_reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  client.loop();

#if defined(ARDUINO_ARCH_ESP32)
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 10);
#endif

  if (as3935_found) {
    loop_as3935();
  }

  // read sensors and publish values
  if (color_watch)
    loop_publish_tcs34725();
  client.loop();

  if ((millis() - last_transmission) > (transmission_delay * 1000)) {
  // Voltage
    loop_publish_voltage();
    client.loop();
    loop_publish_tsl2561();
    client.loop();
    loop_publish_bme280();
    client.loop();
    loop_publish_bme680();
    client.loop();
    loop_publish_veml6070();
    client.loop();
    loop_publish_gy49();
    client.loop();
    loop_publish_si7021();
    client.loop();
    loop_publish_ccs811();
    client.loop();
    last_transmission = millis();
  }
  client.loop();
  if (pir_found)
    loop_pir();
  client.loop();

  if (lox_found) {
    int distance = loop_get_lox_distance();
    lights_on(distance);
  }
  client.loop();

  if (u8x8_found &&
      light_on &&
      (((millis() - last_display) > (1000*30)) ||
       (display_what == DISPLAY_DISTANCE) ||
       display_refresh
     )
    ) {
    char s[10];
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
    display_refresh = false;
    client.loop();
    switch(display_what) {
      case DISPLAY_TEMPHUM:
        if (bme280_found) {
          char h[10];
          snprintf(s, 9, "%.1f C", bme280.readTemperature());
          snprintf(h, 9, "%.1f %%", bme280.readHumidity());
          u8x8.clearDisplay();
          u8x8.draw2x2String(1, 1, s);
          u8x8.draw2x2String(1, 4, h);
        }
        break;
      case DISPLAY_TEMPERATURE:
        if (bme280_found) {
          snprintf(s, 9, "%.1f C", bme280.readTemperature());
          u8x8.clearDisplay();
          u8x8.draw2x2String(1, 3, s);
        }
        break;
      case DISPLAY_HUMIDITY:
        if (bme280_found) {
          snprintf(s, 9, "%.1f %%", bme280.readHumidity());
          u8x8.clearDisplay();
          u8x8.draw2x2String(1, 3, s);
        }
        break;
      case DISPLAY_AIRPRESSURE:
        if (bme280_found) {
          snprintf(s, 9, "%d hPa", (int)(bme280.readPressure() / 100));
          u8x8.clearDisplay();
          u8x8.draw2x2String(1, 3, s);
        }
        break;
      case DISPLAY_CO2:
          if (ccs811_found && ccs.available()) {
            snprintf(s, 9, "%dppm", ccs.geteCO2());
            u8x8.clearDisplay();
            u8x8.draw2x2String(1, 3, s);
          }
          break;
      case DISPLAY_DISTANCE:
        if (lox_found) {
          int dist = loop_get_lox_distance();
          if (dist >= 0) {
            snprintf(s,9,"%d mm",dist);
          } else {
            snprintf(s,9,"---- mm");
          }
          u8x8.clearDisplay();
          u8x8.draw2x2String(1, 3, s);
        }
        break;
    }
    u8x8.setContrast(display_brightness);
    last_display = millis();
  }
  client.loop();

}
