/***************************************************************************
  BME280 Multi Output Thingy
  Reads an BME280 using ESP8266 and provides the results via Serial/USB,
    an internal HTTP-Server, MQTT (with TLS) and HTTP-GET to a Volkszähler

  This script requires the Adafruit BME280-Library. This library is
  written by Limor Fried & Kevin Townsend for Adafruit Industries.

  This script requires the PubSubClient-Library. This library is
  written by Nicholas O'Leary

  This script is written by Florian Knodt - www.adlerweb.info
 ***************************************************************************/
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFiUdp.h>


Adafruit_BME280 bme;

/*extern "C" {
  #include "user_interface.h" //os_timer
}*/

//--------------------------------------------------------------------------
// Constant
//--------------------------------------------------------------------------
const char* device_name = "Wohnzimmer";

// WiFi Settings
const char* wifi_ssid = "<ssid>";
const char* wifi_pass = "geheim";

// MQTT Settings
const char* mqtt_server = "<mqtt_brocker>";
const unsigned int mqtt_port = 1883;
const char* mqtt_user = "username";
const char* mqtt_pass = "geheim";

const char* mqtt_root = "klima/";

//periodic status reports
//const unsigned int stats_interval = 60; // Update statistics and measure every 60 seconds

//-------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------
//#define USE_DEEPSLEEP
// ESP32 Deep Sleep example from http://educ8s.tv/esp32-deep-sleep-tutorial
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

//#define SEALEVELPRESSURE_HPA (1013.25)
#define ALTITUDE 95 //Altitude of your location (m above sea level)

#define USE_MQTT
#define CONFIG_MQTT_TOPIC_GET "/get"
#define CONFIG_MQTT_TOPIC_GET_TEMP "/temperature"
#define CONFIG_MQTT_TOPIC_GET_DEW "/dewpoint"
#define CONFIG_MQTT_TOPIC_GET_HUM "/humidity_abs"
#define CONFIG_MQTT_TOPIC_GET_HUMR "/humidity"
#define CONFIG_MQTT_TOPIC_GET_PRES "/pressure"
#define CONFIG_MQTT_TOPIC_GET_PRESR "/pressure_rel"
#define CONFIG_MQTT_TOPIC_SET "/set"
#define CONFIG_MQTT_TOPIC_SET_RESET "/reset"
#define CONFIG_MQTT_TOPIC_SET_UPDATE "/update"
#define CONFIG_MQTT_TOPIC_SET_PING "/ping"
#define CONFIG_MQTT_TOPIC_SET_PONG "/pong"
#define CONFIG_MQTT_TOPIC_STATUS "/status"
#define CONFIG_MQTT_TOPIC_STATUS_ONLINE "/online"
#define CONFIG_MQTT_TOPIC_STATUS_HARDWARE "/hardware"
#define CONFIG_MQTT_TOPIC_STATUS_VERSION "/version"
#define CONFIG_MQTT_TOPIC_STATUS_INTERVAL "/statsinterval"
#define CONFIG_MQTT_TOPIC_STATUS_IP "/ip"
#define CONFIG_MQTT_TOPIC_STATUS_MAC "/mac"
#define CONFIG_MQTT_TOPIC_STATUS_UPTIME "/uptime"
#define CONFIG_MQTT_TOPIC_STATUS_SIGNAL "/rssi"

#define MQTT_PRJ_HARDWARE "ESP32-bme280"
#define MQTT_PRJ_VERSION "0.0.1"

const float cToKOffset = 273.15;
float absoluteHumidity(float temperature, float humidity);
float saturationVaporPressure(float temperature);
float dewPoint(float temperature, float humidity);


//-------------------------------------------------------------------------
// Timer
//-------------------------------------------------------------------------
unsigned long startTime;
bool sendStats = true;

void timerCallback(void *arg) {
  sendStats = true;
}

//-------------------------------------------------------------------------
// MQTT Code
//-------------------------------------------------------------------------
#ifdef USE_MQTT

class PubSubClientWrapper : public PubSubClient{
  private:
  public:
    PubSubClientWrapper(Client& espc);
    bool publish(StringSumHelper topic, String str);
    bool publish(StringSumHelper topic, unsigned int num);
    bool publish(const char* topic, String str);
    bool publish(const char* topic, unsigned int num);

    bool publish(StringSumHelper topic, String str, bool retain);
    bool publish(StringSumHelper topic, unsigned int num, bool retain);
    bool publish(const char* topic, String str, bool retain);
    bool publish(const char* topic, unsigned int num, bool retain);
};

PubSubClientWrapper::PubSubClientWrapper(Client& espc) : PubSubClient(espc){

}

bool PubSubClientWrapper::publish(StringSumHelper topic, String str) {
  return publish(topic.c_str(), str);
}

bool PubSubClientWrapper::publish(StringSumHelper topic, unsigned int num) {
  return publish(topic.c_str(), num);
}

bool PubSubClientWrapper::publish(const char* topic, String str) {
  return publish(topic, str, false);
}

bool PubSubClientWrapper::publish(const char* topic, unsigned int num) {
  return publish(topic, num, false);
}

bool PubSubClientWrapper::publish(StringSumHelper topic, String str, bool retain) {
  return publish(topic.c_str(), str, retain);
}

bool PubSubClientWrapper::publish(StringSumHelper topic, unsigned int num, bool retain) {
  return publish(topic.c_str(), num, retain);
}

bool PubSubClientWrapper::publish(const char* topic, String str, bool retain) {
  char buf[128];

  if(str.length() >= 128) return false;

  str.toCharArray(buf, 128);
  return PubSubClient::publish(topic, buf, retain);
}

bool PubSubClientWrapper::publish(const char* topic, unsigned int num, bool retain) {
  char buf[6];

  dtostrf(num, 0, 0, buf);
  return PubSubClient::publish(topic, buf, retain);
}


WiFiClient mqtt_client;
#ifdef USE_MQTT
  PubSubClientWrapper client(mqtt_client);
#endif

uint8_t rssiToPercentage(int32_t rssi) {
  //@author Marvin Roger - https://github.com/marvinroger/homie-esp8266/blob/ad876b2cd0aaddc7bc30f1c76bfc22cd815730d9/src/Homie/Utils/Helpers.cpp#L12
  uint8_t quality;
  if (rssi <= -100) {
    quality = 0;
  } else if (rssi >= -50) {
    quality = 100;
  } else {
    quality = 2 * (rssi + 100);
  }

  return quality;
}

void ipToString(const IPAddress& ip, char * str) {
  //@author Marvin Roger - https://github.com/marvinroger/homie-esp8266/blob/ad876b2cd0aaddc7bc30f1c76bfc22cd815730d9/src/Homie/Utils/Helpers.cpp#L82
  snprintf(str, 16, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
}

void _mqttConnect(){

  // Do not connect if already connected
  if (client.connected()) return;

  client.setServer(mqtt_server, mqtt_port);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP32-client", mqtt_user, mqtt_pass )) {

      Serial.println("connected");

    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }

}

#endif

//-------------------------------------------------------------------------
// WiFi Code
//-------------------------------------------------------------------------
void wifi_setup() {
  delay(10);

  if(WiFi.status() == WL_CONNECTED) return;

  //We start by connecting to a WiFi network
  Serial.println();
  Serial.print(F("Connecting to "));
  Serial.println(wifi_ssid);

  WiFi.mode(WIFI_STA); // Disable the built-in WiFi access point.
  WiFi.begin(wifi_ssid, wifi_pass);

  // Check the Connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println(F("WiFi connected"));
}

//--------------------------------------------------------------------------
// BME280
//--------------------------------------------------------------------------

void bmeSetup(){

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);

  if(!status){
    Serial.println(F("Ops! BME280 could not be found!"));
    Serial.println(F("Please check your connections."));
    Serial.println();
    Serial.println(F("Troubleshooting Guide"));
    Serial.println(F("*************************************************************"));
    Serial.println(F("1. Let's check the basics: Are the VCC and GND pins connected correctly? If the BME280 is getting really hot, then the wires are crossed."));
    Serial.println();
    Serial.println(F("2. Are you using the I2C mode? Did you connect the SDI pin from your BME280 to the SDA line from the Arduino?"));
    Serial.println();
    Serial.println(F("3. And did you connect the SCK pin from the BME280 to the SCL line from your Arduino?"));
    Serial.println();
    Serial.println(F("4. Are you using the alternative I2C Address(0x76)? Did you remember to connect the SDO pin to GND?"));
    Serial.println();
    Serial.println(F("5. If you are using the default I2C Address (0x77), did you remember to leave the SDO pin unconnected?"));
    Serial.println();
    Serial.println(F("6. Are you using the SPI mode? Did you connect the Chip Select (CS) pin to the pin 10 of your Arduino (or to wherever pin you choosed)?"));
    Serial.println();
    Serial.println(F("7. Did you connect the SDI pin from the BME280 to the MOSI pin from your Arduino?"));
    Serial.println();
    Serial.println(F("8. Did you connect the SDO pin from the BME280 to the MISO pin from your Arduino?"));
    Serial.println();
    Serial.println(F("9. And finally, did you connect the SCK pin from the BME280 to the SCK pin from your Arduino?"));
    Serial.println();

    while(1);
  }

  Serial.println("Detect BME280 Sensor");

  Serial.println();
}

//-------------------------------------------------------------------------
// Send States
//-------------------------------------------------------------------------

void stateOnBoot(){
  #ifdef USE_MQTT
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_STATUS_HARDWARE), MQTT_PRJ_HARDWARE, true);
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_STATUS_VERSION), MQTT_PRJ_VERSION, true);
    char buf[5];
    sprintf(buf, "%d", TIME_TO_SLEEP);
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_STATUS_INTERVAL), buf, true);
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_STATUS_MAC), WiFi.macAddress(), true);
  #endif

  Serial.println(F("=== State ==="));
  Serial.print(F("Devicename: "));
  Serial.println(device_name);
  Serial.print(F("MacAdress: "));
  Serial.println(WiFi.macAddress());
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());
  Serial.println();
  printHassSettings();
}

void printHassSettings(){
  Serial.println(F("=== Hass Settings ==="));

  // Temperatur
  Serial.println();
  Serial.print(F("- name: "));
  Serial.print(device_name);
  Serial.println(F(" Temperatur"));
  Serial.println(F("  platform: mqtt"));
  Serial.print(F("  state_topic: "));
  Serial.println((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_GET + CONFIG_MQTT_TOPIC_GET_TEMP);
  Serial.println(F("  unit_of_measurement: '°C'"));

  // Taupunkt
  Serial.println();
  Serial.print(F("- name: "));
  Serial.print(device_name);
  Serial.println(F(" Taupunkt"));
  Serial.println(F("  platform: mqtt"));
  Serial.print(F("  state_topic: "));
  Serial.println((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_GET + CONFIG_MQTT_TOPIC_GET_DEW);
  Serial.println(F("  unit_of_measurement: '°C'"));

  // humidity_abs
  Serial.println();
  Serial.print(F("- name: "));
  Serial.print(device_name);
  Serial.println(F(" absolute Luftfeuchtigkeit"));
  Serial.println(F("  platform: mqtt"));
  Serial.print(F("  state_topic: "));
  Serial.println((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_GET + CONFIG_MQTT_TOPIC_GET_HUM);
  Serial.println(F("  unit_of_measurement: 'g/m^3'"));

  // humidity_rel
  Serial.println();
  Serial.print(F("- name: "));
  Serial.print(device_name);
  Serial.println(F(" relative Luftfeuchtigkeit"));
  Serial.println(F("  platform: mqtt"));
  Serial.print(F("  state_topic: "));
  Serial.println((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_GET + CONFIG_MQTT_TOPIC_GET_HUMR);
  Serial.println(F("  unit_of_measurement: '%'"));

  // pressure
  Serial.println();
  Serial.print(F("- name: "));
  Serial.print(device_name);
  Serial.println(F(" Luftdruck"));
  Serial.println(F("  platform: mqtt"));
  Serial.print(F("  state_topic: "));
  Serial.println((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_GET + CONFIG_MQTT_TOPIC_GET_PRES);
  Serial.println(F("  unit_of_measurement: hPA"));

  // pressure_rel
  Serial.println();
  Serial.print(F("- name: "));
  Serial.print(device_name);
  Serial.println(F(" relativer Luftdruck"));
  Serial.println(F("  platform: mqtt"));
  Serial.print(F("  state_topic: "));
  Serial.println((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_GET + CONFIG_MQTT_TOPIC_GET_PRESR);
  Serial.println(F("  unit_of_measurement: hPA"));

  // Group
  Serial.println();
  Serial.print(device_name);
  Serial.println(F("-klima:"));
  Serial.print(F("  name: Temperatur "));
  Serial.println(device_name);
  Serial.println(F("  entities:"));
  Serial.print(F("    - sensor."));
  Serial.print(device_name);
  Serial.println(F("_temperatur"));
  Serial.print(F("    - sensor."));
  Serial.print(device_name);
  Serial.println(F("_taupunkt"));
  Serial.print(F("    - sensor."));
  Serial.print(device_name);
  Serial.println(F("_relative_luftfeuchtigkeit"));
  Serial.print(F("    - sensor."));
  Serial.print(device_name);
  Serial.println(F("_absolute_luftfeuchtigkeit"));
  Serial.print(F("    - sensor."));
  Serial.print(device_name);
  Serial.println(F("_luftdruck"));
  Serial.print(F("    - sensor."));
  Serial.print(device_name);
  Serial.println(F("_relativer_luftdruck"));
  Serial.println();
}

void sendStatsInterval(void) {
  char buf[16]; //v4 only atm
  ipToString(WiFi.localIP(), buf);
  #ifdef USE_MQTT
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_STATUS_IP), buf);
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_STATUS_UPTIME), (uint32_t)(millis()/1000));
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_STATUS_SIGNAL), rssiToPercentage(WiFi.RSSI()));
  #endif

  float temperature = bme.readTemperature();
  float humidity_r = bme.readHumidity();
  float humidity = absoluteHumidity(temperature, humidity_r);
  float pressure = bme.readPressure() / 100.0F;
  float pressure_r = bme.seaLevelForAltitude(ALTITUDE, pressure_r);
  float dew = dewPoint(temperature, humidity_r);

  Serial.print(F("T: "));
  Serial.print((String)temperature);
  Serial.print(F(" *C\nDP: "));
  Serial.print((String)dew);
  Serial.print(F(" *C\nH: "));
  Serial.print((String)humidity_r);
  Serial.print(F(" %\nAH: "));
  Serial.print((String)humidity);
  Serial.print(F(" g/m3\nRP: "));
  Serial.print((String)pressure_r);
  Serial.print(F(" hPa\nP: "));
  Serial.print((String)pressure);
  Serial.println(F(" hPa"));
  Serial.flush();

  yield();

  #ifdef USE_MQTT
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_GET + CONFIG_MQTT_TOPIC_GET_TEMP), (String)temperature);
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_GET + CONFIG_MQTT_TOPIC_GET_DEW), (String)dew);
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_GET + CONFIG_MQTT_TOPIC_GET_HUM), (String)humidity);
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_GET + CONFIG_MQTT_TOPIC_GET_HUMR), (String)humidity_r);
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_GET + CONFIG_MQTT_TOPIC_GET_PRES), (String)pressure);
    client.publish(((String)mqtt_root + (String)device_name + CONFIG_MQTT_TOPIC_GET + CONFIG_MQTT_TOPIC_GET_PRESR), (String)pressure_r);
  #endif

  #ifdef USE_VOLKSZAEHLER
    sendVz(vz_uuid_temp, temperature);
    sendVz(vz_uuid_dew, dew);
    sendVz(vz_uuid_hum, humidity);
    sendVz(vz_uuid_hum_r, humidity_r);
    sendVz(vz_uuid_pres, pressure);
    sendVz(vz_uuid_pres_r, pressure_r);
  #endif

  Serial.println("Send Data!");
}

//--------------------------------------------------------------------------
// Main Code
//--------------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("ESP32 Klima mit BMP280, MQTT und InfluxDB");

  bmeSetup();


  // Setup the WiFi connection
  wifi_setup();

  // Setup mqtt connection
  #ifdef USE_MQTT
    _mqttConnect();
  #endif

  if(bootCount == 0){
    // Send boot informations
    stateOnBoot();
    bootCount = bootCount+1;
  }

  #ifdef USE_DEEPSLEEP
    delay(3000);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
    Serial.println("Going to sleep as normal now.");
    esp_deep_sleep_start();
  #endif
}


void loop() {
  #ifndef USE_DEEPSLEEP

    if (millis() - startTime >= TIME_TO_SLEEP * 1000) {
        // 5 seconds have elapsed. ... do something interesting ...
        startTime = millis();
        sendStats=true;
     }

    #ifdef USE_MQTT
      if (!client.connected()) {
        _mqttConnect();
      }
      client.loop();
    #endif

    if(sendStats) {
      sendStatsInterval();
      sendStats=false;
    }

  #endif
}


//-----------------------------------------------------------------------------
// Do Math
//-----------------------------------------------------------------------------
// Relative to absolute humidity
// Based on https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
float absoluteHumidity(float temperature, float humidity) {
  return (13.2471*pow(EULER,17.67*temperature/(temperature+243.5))*humidity/(cToKOffset+temperature));
}

// Calculate saturation vapor pressure
// Based on dew.js, Copyright 2011 Wolfgang Kuehn, Apache License 2.0
float saturationVaporPressure(float temperature) {
  if(temperature < 173 || temperature > 678) return -112; //Temperature out of range

  float svp = 0;
  if(temperature <= cToKOffset) {
    /**
      * -100-0°C -> Saturation vapor pressure over ice
      * ITS-90 Formulations by Bob Hardy published in
      * "The Proceedings of the Third International
      * Symposium on Humidity & Moisture",
      * Teddington, London, England, April 1998
      */

    svp = exp(-5.8666426e3/temperature + 2.232870244e1 + (1.39387003e-2 + (-3.4262402e-5 + (2.7040955e-8*temperature)) * temperature) * temperature + 6.7063522e-1 * log(temperature));
  }else{
    /**
      * 0°C-400°C -> Saturation vapor pressure over water
      * IAPWS Industrial Formulation 1997
      * for the Thermodynamic Properties of Water and Steam
      * by IAPWS (International Association for the Properties
      * of Water and Steam), Erlangen, Germany, September 1997.
      * Equation 30 in Section 8.1 "The Saturation-Pressure
      * Equation (Basic Equation)"
      */

    const float th = temperature + -0.23855557567849 / (temperature - 0.65017534844798e3);
    const float a  = (th + 0.11670521452767e4) * th + -0.72421316703206e6;
    const float b  = (-0.17073846940092e2 * th + 0.12020824702470e5) * th + -0.32325550322333e7;
    const float c  = (0.14915108613530e2 * th + -0.48232657361591e4) * th + 0.40511340542057e6;

    svp = 2 * c / (-b + sqrt(b * b - 4 * a * c));
    svp *= svp;
    svp *= svp;
    svp *= 1e6;
  }

  yield();

  return svp;
}


// Calculate dew point in °C
// Based on dew.js, Copyright 2011 Wolfgang Kuehn, Apache License 2.0
float dewPoint(float temperature, float humidity)
{
  temperature += cToKOffset; //Celsius to Kelvin

  if(humidity < 0 || humidity > 100) return -111; //Invalid humidity
  if(temperature < 173 || temperature > 678) return -112; //Temperature out of range

  humidity = humidity / 100 * saturationVaporPressure(temperature);

  byte mc = 10;

  float xNew;
  float dx;
  float z;

  do {
    dx = temperature / 1000;
    z = saturationVaporPressure(temperature);
    xNew = temperature + dx * (humidity - z) / (saturationVaporPressure(temperature + dx) - z);
    if (abs((xNew - temperature) / xNew) < 0.0001) {
        return xNew - cToKOffset;
    }
    temperature = xNew;
    mc--;
  } while(mc > 0);

  return -113; //Solver did not get a close result
}
