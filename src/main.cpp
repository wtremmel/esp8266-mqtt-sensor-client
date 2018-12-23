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
// #define ESP10 // Büro Wolfgang / Schifferstadt / Flur 1.OG
// #define ESP11 // Lolin32 Lite
// #define ESP12 // Schlafzimmer
// #define ESP13 // Lichterkette / Uhr
// #define ESP14 // Display
// #define ESP15 // Keller
// ESP16 Erdgeschoss Flur

#include <Arduino.h>
#include <ArduinoLog.h>

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
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

#else
#pragma message ("unknow build enviroment")
#endif

// Board dependencies
#if BOARD == heltec
#define I2CSDA 4  //D2 gruen
#define I2CSCL 15  //D1 gelb
#endif

#include <PubSubClient.h>
#include <ArduinoJson.h>


// Sensor Libraries
#include <Wire.h>
#include "Adafruit_Si7021.h"
#include "Adafruit_BME280.h"
#include "Adafruit_TSL2561_U.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_VEML6070.h"
#include <U8x8lib.h>

// Global defines
#define NEOPIXEL 14 //D5
#define NROFLEDS 10


// Global Objects
Adafruit_Si7021 si7021;
Adafruit_BME280 bme280;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);
Adafruit_VEML6070 veml = Adafruit_VEML6070();
Adafruit_NeoPixel led = Adafruit_NeoPixel(NROFLEDS, NEOPIXEL, NEO_GRB + NEO_KHZ800);
WiFiClient espClient;
PubSubClient client;
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

unsigned transmission_delay = 60; // seconds

// Strings for dynamic config
String Smyname, Spass, Sssid, Smqttserver, Ssite, Sroom, Smqttuser, Smqttpass;
unsigned int Imqttport;
bool Bflipped;


// Flags for sensors found
bool si7021_found = false;
bool bme280_found = false;
bool tsl2561_found= false;
bool voltage_found= true;
bool u8x8_found = false;
bool veml_found = false;
bool rtc_init_done = false;
bool rtc_alarm_raised = false;

// Flags for display
#define DISPLAY_OFF 0
#define DISPLAY_TEMPERATURE 1
#define DISPLAY_HUMIDITY 2
#define DISPLAY_AIRPRESSURE 3
#define DISPLAY_LUX 4
#define DISPLAY_STRING 5

unsigned int display_what = DISPLAY_TEMPERATURE;

// Timer variables
unsigned long now;
unsigned long last_transmission = 0;
unsigned long last_display = 0;


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
  Log.verbose("Sroom = %s",Sroom.c_str());
  Log.verbose("Sssid = %s",Sssid.c_str());
  Log.verbose("Spass = %s",Spass.c_str());
  Log.verbose("Smqttuser = %s",Smqttuser.c_str());
  Log.verbose("Smqttpass = %s",Smqttpass.c_str());
  Log.verbose("Imqttport = %d",Imqttport);
  Log.verbose(F("Bflipped = %t"),Bflipped);

}

void write_config () {
  StaticJsonBuffer<512> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["myname"] = Smyname;
  root["flipped"] = Bflipped;
  JsonObject& network = root.createNestedObject("network");
  network["pass"] = Spass;
  network["ssid"] = Sssid;
  JsonObject& mqtt = root.createNestedObject("mqtt");
  mqtt["server"] = Smqttserver;
  mqtt["user"] = Smqttuser;
  mqtt["pass"] = Smqttpass;
  mqtt["port"] = Imqttport;
  JsonObject& location = root.createNestedObject("location");
  location["site"] = Ssite;
  location["room"] = Sroom;

  Log.notice(F("Writing new config file"));
  root.prettyPrintTo(Serial);

  SPIFFS.begin();
  File f = SPIFFS.open("/config.json","w");
  if (!f) {
    Log.error(F("Open of config file for writing failed"));
  } else {
    if (root.printTo(f) == 0) {
      Log.error(F("Writing object into file failed"));
    } else {
      Log.notice(F("Written new config. Now reboot"));
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

  if (in[0] == "led") {
    if (wordcounter == 3) {
      // led r g b
      setled(in[1].toInt(),in[2].toInt(),in[3].toInt());
    } else if (wordcounter == 4) {
      setled(in[1].toInt(),in[2].toInt(),in[3].toInt(),in[4].toInt());
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

  if (in[0] == "display" && wordcounter >= 1) {
    last_display = 0;
    if (in[1] == "humidity") {
      display_what = DISPLAY_HUMIDITY;
    } else if (in[1] == "airpressure") {
      display_what = DISPLAY_AIRPRESSURE;
    } else if (in[1] == F("temperature")) {
      display_what = DISPLAY_TEMPERATURE;
    } else if (in[1] == "off") {
      u8x8.clearDisplay();
      display_what = DISPLAY_OFF;
    } else if (in[1] == F("flip")) {
      u8x8.clearDisplay();
      u8x8.setFlipMode(wordcounter > 1);
      Bflipped = (wordcounter > 1);
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



  Log.verbose("Attempting MQTT connection...%d...",client.state());

  // Attempt to connect
  if (client.connect(Smyname.c_str(),Smqttuser.c_str(),Smqttpass.c_str(),mytopic,0,0,"stopped")) {
    Log.verbose("MQTT connected");

    client.publish(mytopic, "started");
    delay(10);
    // ... and resubscribe to my name
    client.subscribe(Smyname.c_str());
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

  Log.verbose("MQTT Publish message [%s]:%s",topic,msg);

  char mytopic[50];
  snprintf(mytopic, 50, "/%s/%s/%s", Ssite.c_str(), Sroom.c_str(),topic);
  client.publish(mytopic, msg);
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

void mqtt_publish(char *topic, float value) {
  char buf[15];
  snprintf(buf,14,"%.3f",value);
  mqtt_publish(topic, buf);
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
// 0x3c Display
// 0x40 SI7021
// 0x48 4*AD converter
// 0x4a GY49 or MAX44009 Light Sensor
// 0x50 PCF8583P
// 0x57 ATMEL732
// 0x68 DS3231 Clock
// 0x76 BME280
// 0x77 BME680 (also BMP180)


  Log.notice("Scanning i2c bus");
  Wire.begin(I2CSDA, I2CSCL);

  // Try for DISPLAY
  pinMode(16,OUTPUT);
  digitalWrite(16,LOW);
  delay(100);
  digitalWrite(16,HIGH);


  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Log.trace("I2C device found at address 0x%x",address);

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
        if (u8x8_found) {
          Log.notice("U8xu found? %T",u8x8_found);
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

      if (address == 0x76 || address == 0x77) {
        // BME280
        bme280_found = bme280.begin(address);
        Log.notice("BME280 found? %T at 0x%x",bme280_found,address);
      }
    }
  }
  Log.notice("End scanning i2c bus");
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
   Bflipped = root["flipped"];
   Spass = root["network"]["pass"].as<String>();
   Sssid = root["network"]["ssid"].as<String>();
   Smqttserver = root["mqtt"]["server"].as<String>();
   Ssite = root["location"]["site"].as<String>();
   Sroom = root["location"]["room"].as<String>();
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
#if defined(ARDUINO_ARCH_ESP8266)
  WiFi.hostname(Smyname);
#endif
  WiFi.begin(Sssid.c_str(), Spass.c_str());

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    retries++;
    Log.error("Wifi.status() = %d",WiFi.status());
  }
  String myIP = String(WiFi.localIP().toString());
  String myMask = String(WiFi.subnetMask().toString());
  Log.verbose("Wifi connected as %s/%s",myIP.c_str(),myMask.c_str());
}

void setup_mqtt() {
  client.setClient(espClient);
  client.setServer(Smqttserver.c_str(), Imqttport);
  client.setCallback(mqtt_callback);

}


// ESP32 Specific Setup routines
#if defined(ARDUINO_ARCH_ESP32)
void callback_esp32_gap (esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {

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
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);

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
  setup_i2c();
#ifdef ARDUINO_ARCH_ESP32
  setup_esp32();
#endif
  setled(255, 128, 0);
  setup_wifi();
  setup_mqtt();
  setled(0, 255, 0);
  delay(1000);
  setled(0,0,0);
}

void loop_publish_voltage(){
#if defined(ARDUINO_ARCH_ESP8266)
  mqtt_publish("voltage", (float)(ESP.getVcc() / 1000.0));
#endif
}

void loop_publish_tsl2561() {
  if (tsl2561_found) {
    sensors_event_t event;

    if (tsl2561.begin() && tsl2561.getEvent(&event)) {
      mqtt_publish("light",event.light);
    } else {
      Log.verbose("loop_publish_tsl2561: Sensor overloaded");
    }
  }
}

void loop_publish_bme280() {
  if (bme280_found) {
    mqtt_publish("temperature", bme280.readTemperature());
    mqtt_publish("airpressure", bme280.readPressure() / 100.0F);
    mqtt_publish("humidity", bme280.readHumidity());

  }
}



void loop() {
  // put your main code here, to run repeatedly:
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

  // read sensors and publish values

  if ((millis() - last_transmission) > (transmission_delay * 1000)) {
  // Voltage
    loop_publish_voltage();
    loop_publish_tsl2561();
    loop_publish_bme280();

    last_transmission = millis();
  }

  if (u8x8_found && ((millis() - last_display) > (1000*30))) {
    char s[10];
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
    switch(display_what) {
      case DISPLAY_TEMPERATURE:
        if (bme280_found) {
          snprintf(s, 9, "%.1f C", bme280.readTemperature());
        }
        u8x8.clearDisplay();
        u8x8.draw2x2String(1, 3, s);
        break;
      case DISPLAY_HUMIDITY:
        if (bme280_found) {
          snprintf(s, 9, "%.1f %%", bme280.readHumidity());
        }
        u8x8.clearDisplay();
        u8x8.draw2x2String(1, 3, s);
        break;
      case DISPLAY_AIRPRESSURE:
        if (bme280_found) {
          snprintf(s, 9, "%d hPa", (int)(bme280.readPressure() / 100));
        }
        u8x8.clearDisplay();
        u8x8.draw2x2String(1, 3, s);
        break;

    }

    last_display = millis();
  }
}
