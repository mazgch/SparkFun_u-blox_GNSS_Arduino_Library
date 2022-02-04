/*
  Use ESP32 WiFi to get SPARTN data from PointPerfect (broker) as a Client
  By: u-blox AG / Michael Ammann
  Date: January 27th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to obtain SPARTN data from a PointPerfect Broker over WiFi
  and push it over I2C to a ZED-F9x.
  It's confusing, but the Arduino is acting as a 'client' to the PointPerfect SSR correction service.

  You will need to have a valid u-blox Thingstream account and have a PointPerfect Thing and payed plan. 
  Thingstream offers SSR corrections to SPARTN cabalble RTK receivers such as the u-blox ZED-F9 series 
  in continental Europ and US. There Network is planned to be expanded to ther regions over next years. 
  To see sign up go to https://portal.thingstream.io/app/location-services/things

  This is a proof of concept to show how to connect via MQTT to get SPARTN SSR correction. 
  Using WiFi for a rover is generally a bad idea because of limited WiFi range in the field. 
  You may use this exmaple in combination with a cell phone with hotspot mode enabled. 

  For more information about MQTT, SPARTN and PointPerfect Correction Services 
  please see: https://www.u-blox.com/en/product/pointperfect
  
  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/16481
  RTK Surveyor: https://www.sparkfun.com/products/18443
  RTK Express: https://www.sparkfun.com/products/18442
  
  Recommended Hardware:
  MicroMod GNSS Carrier Board: https://www.sparkfun.com/products/17722 
  ESP32 Micromod https://www.sparkfun.com/products/16781

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a ESP32 Thing Plus
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/
#include <WiFi.h>
#include <WiFiClientSecure.h>  // https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFiClientSecure
#include <ArduinoMqttClient.h> // https://github.com/arduino-libraries/ArduinoMqttClient 
#include <Arduino_JSON.h>       // https://github.com/bblanchon/ArduinoJson
//The ESP32 core has a built in base64 library but not every platform does
//We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif
#include "secrets.h"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library
   
// ########################################################################################
// move to SparkFun_u-blox_GNSS_Arduino_Library/src/u-blox_config_keys.h  880 / 786
// ########################################################################################

//CFG-PMP: Point-To-MultiPoint for L-band Receiver (NEO-D9S)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_PMP_CENTER_FREQUENCY     = 0x40b10011;
const uint32_t UBLOX_CFG_PMP_SEARCH_WINDOW        = 0x30b10012;
const uint32_t UBLOX_CFG_PMP_USE_SERVICE_ID       = 0x10b10016;
const uint32_t UBLOX_CFG_PMP_SERVICE_ID           = 0x30b10017;
const uint32_t UBLOX_CFG_PMP_DATA_RATE            = 0x30b10013;
const uint32_t UBLOX_CFG_PMP_USE_DESCRAMBLER      = 0x10b10014;
const uint32_t UBLOX_CFG_PMP_DESCRAMBLER_INIT     = 0x30b10015;
const uint32_t UBLOX_CFG_PMP_USE_PRESCRAMBLING    = 0x10b10019;
const uint32_t UBLOX_CFG_PMP_UNIQUE_WORD          = 0x50b1001a;

const uint32_t UBLOX_CFG_SPARTN_USE_SOURCE        = 0x20a70001;

//Additional CFG_MSGOUT keys for the NEO-D9S
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_PMP_I2C   = 0x2091031d; 
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART1 = 0x2091031e; 
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART2 = 0x2091031f; 
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_PMP_USB   = 0x20910320; 
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_PMP_SPI   = 0x20910321; 
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_PMP_I2C   = 0x20910322; 
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_PMP_UART1 = 0x20910323; 
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_PMP_UART2 = 0x20910324; 
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_PMP_USB   = 0x20910325; 
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_PMP_SPI   = 0x20910326;

#if 0
// ########################################################################################
// SparkFun_u-blox_GNSS_Arduino_Library.cpp::760
// ########################################################################################

//Returns true if I2C device ack's
bool SFE_UBLOX_GNSS::isConnected(uint16_t maxWait)
{
  if (commType == COMM_TYPE_I2C)
  {
    _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
    if (_i2cPort->endTransmission() != 0)
      return false; //Sensor did not ack
  }

  //Poll MON-VER to see whether we get a meaningful response
  packetCfg.cls = UBX_CLASS_MON;
  packetCfg.id = UBX_MON_VER;
  packetCfg.len = 0;
  packetCfg.startingSpot = 40; //Start at first "extended software information" string
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED);
  // Query navigation rate to see whether we get a meaningful response
  //return (getNavigationFrequencyInternal(maxWait)); // NEO-D9S does not support this :(
}

// ########################################################################################
// SparkFun_u-blox_GNSS_Arduino_Library.h::910
// ########################################################################################

uint8_t setVal64(uint32_t keyID, uint64_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = 250);             //Sets the 32-bit value at a given group/id/size location
    
// ########################################################################################
// SparkFun_u-blox_GNSS_Arduino_Library.cpp::7324
// ########################################################################################

//Given a key, set a 64-bit value
//This function takes a full 64-bit key
//Default layer is all: RAM+BBR+Flash
//Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::setVal64(uint32_t key, uint64_t value, uint8_t layer, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4 + 4 + 8; //4 byte header, 4 byte key ID, 8 bytes of value
  packetCfg.startingSpot = 0;

  //Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  payloadCfg[0] = 0;     //Message Version - set to 0
  payloadCfg[1] = layer; //By default we ask for the BBR layer

  //Load key into outgoing payload
  payloadCfg[4] = key >> 8 * 0; //Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;

  //Load user's value
  payloadCfg[8]  = value >> 8 * 0; //Value LSB
  payloadCfg[9]  = value >> 8 * 1;
  payloadCfg[10] = value >> 8 * 2;
  payloadCfg[11] = value >> 8 * 3;
  payloadCfg[12] = value >> 8 * 4;
  payloadCfg[13] = value >> 8 * 5;
  payloadCfg[14] = value >> 8 * 6;
  payloadCfg[15] = value >> 8 * 7;

  //Send VALSET command with this key and value
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}
#endif

//Global variables
long last_ms = 0;
WiFiClientSecure wifiClient = WiFiClientSecure();
MqttClient mqttClient(wifiClient);
SFE_UBLOX_GNSS myGNSS;
SFE_UBLOX_GNSS myLBand;
bool gnssOk = false;
bool lbandOk = false;
bool wifiOk = false;
bool wifiConnected = false;
bool mqttOk = false;
uint32_t myLBandFreq = 0;
const char* myIpTopic = NULL;

#define OK(ok) (ok ? F("  ->  ok") : F("  ->  ERROR!"))

void mqttMessageHandler(int messageSize) {
  Serial.print(F("MQTT: "));
  String topic = mqttClient.messageTopic();
  Serial.print(topic);
  int total = 0; 
  while (mqttClient.available()) {
    uint8_t buf[512]; // Most incoming data is around 500 bytes but may be larger
    int cnt = 0;
    while (mqttClient.available())
    {
      buf[cnt++] = mqttClient.read();
      if (cnt == sizeof(buf)) 
        break;
    }
    if (gnssOk) 
      gnssOk = myGNSS.pushRawData(buf, cnt, false);
    total += cnt;
  }
  Serial.print(F(" received "));
  Serial.print(total);
  Serial.print(F(" bytes, send to GNSS "));
  Serial.println(OK(((total > 0) && gnssOk))); 
}

bool checkLband() {
  // TODO: This is a hack we capture the LBAND here using direct Wire 0x43 access and just 
  // forward all its data to the GNSS. Instead it would be better if the SFE_UBLOX_GNSS 
  // would capture the RXM-PMP message and we could attach a callback to it. 
  Wire.beginTransmission(0x43);
  Wire.write(0xFD);
  int ok = (0 == Wire.endTransmission(false));
  if (ok) ok = (2 == Wire.requestFrom(0x43, 2));
  if (ok) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t cnt = ((uint16_t)msb << 8) + lsb;
    if (cnt) {
      Serial.print(F("L-Band: received "));
      Serial.print(cnt);
      Serial.print(F(" bytes, sent to GNSS "));
      int total = 0; 
      while (ok && cnt) {
        uint8_t buf[32];
        int blk = (cnt>sizeof(buf)) ? sizeof(buf) : cnt;
        ok = (blk == Wire.requestFrom(0x43, blk));
        int i = 0;
        while (i < blk) {
          buf[i++] = Wire.read();
        }
        if (gnssOk) 
          gnssOk = myGNSS.pushRawData(buf, blk, false);
        total += blk;
        cnt -= blk;
      }
      Serial.println(OK(gnssOk));  
    }
  }
  return ok;
}
bool setupWiFi() {
  wifiClient.setCACert(AWS_CERT_CA);
  wifiClient.setCertificate(AWS_CERT_CRT);
  wifiClient.setPrivateKey(AWS_CERT_PRIVATE);
  mqttClient.onMessage(mqttMessageHandler);
  int ok = WiFi.begin(ssid, password);
  Serial.print(F("WiFI: configured and connecting to "));
  Serial.println(ssid);
  wifiConnected = false;
  return ok; 
}

bool detectGNSS() {
  bool ok = myGNSS.begin(); //Connect to the Ublox module using Wire port
  if (ok) {
    ok = myGNSS.setI2COutput(COM_TYPE_UBX); //Turn off NMEA noise
    if (ok) ok = myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_SPARTN); // Be sure SPARTN input is enabled.
    if (ok) ok = myGNSS.setNavigationFrequency(1); //Set output in Hz.
    if (ok) ok = myGNSS.setAutoPVT(true); //Tell the GNSS to "send" each solution
    Serial.print(F("GNSS: detected receiver, configuration"));
    Serial.println(OK(ok));
  }
  return ok;
}

bool detectLBand() {
  int ok = myLBand.begin(Wire, 0x43 /* NEO-D9S I2C address */ );
  if (ok)
  {
    myLBandFreq = 0;
    ok = myLBand.setVal16(UBLOX_CFG_PMP_SEARCH_WINDOW,      2200); 
    if (ok) ok = myLBand.setVal8(UBLOX_CFG_PMP_USE_SERVICE_ID,      0);           // Default 1 
    if (ok) ok = myLBand.setVal16(UBLOX_CFG_PMP_SERVICE_ID,         21845);       // Default 50851
    if (ok) ok = myLBand.setVal16(UBLOX_CFG_PMP_DATA_RATE,          2400); 
    if (ok) ok = myLBand.setVal8(UBLOX_CFG_PMP_USE_DESCRAMBLER,     1); 
    if (ok) ok = myLBand.setVal16(UBLOX_CFG_PMP_DESCRAMBLER_INIT,   26969);       // Default 23560
    if (ok) ok = myLBand.setVal8(UBLOX_CFG_PMP_USE_PRESCRAMBLING,   0); 
    if (ok) ok = myLBand.setVal64(UBLOX_CFG_PMP_UNIQUE_WORD,        16238547128276412563ull); 
    if (ok) ok = myLBand.setVal(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_I2C,   1);
    if (ok) ok = myLBand.setVal(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART1, 1);
    if (ok) ok = myLBand.setVal(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART2, 1);
    if (ok) ok = myLBand.setVal32(UBLOX_CFG_UART1_BAUDRATE,           38400); // match baudrate with ZED default
    if (ok) ok = myLBand.setVal32(UBLOX_CFG_UART2_BAUDRATE,           38400); // match baudrate with ZED default
    Serial.print(F("L-Band: detected receiver, configuration "));
    Serial.println(OK(ok));
  }
  return ok;
}

void updatePointPerfectSources(long lon, long lat) {
  int32_t lBandFreq = myLBandFreq;
  const char* ipTopic = myIpTopic; 
  float fLat = lat * 1e-7;
  float fLon = lon * 1e-7;;   
  for (int i = 0; i < sizeof(REGION_LIST)/sizeof(*REGION_LIST); i ++) {
    if ((fLat >= REGION_LIST[i].lat1 && fLat <= REGION_LIST[i].lat2) && 
        (fLon >= REGION_LIST[i].lon1 && fLon <= REGION_LIST[i].lon2)) {
      if (REGION_LIST[i].lBandFreq > 0) lBandFreq = REGION_LIST[i].lBandFreq;
      if (REGION_LIST[i].ipTopic)       ipTopic = REGION_LIST[i].ipTopic;
    }
  }
  // Upate the ferquecy in the LBand Rx
  if ((lbandOk && (lBandFreq != myLBandFreq)) || (mqttOk && (ipTopic != myIpTopic))) {
    Serial.print(F("GNSS: coverage changed "));
    Serial.print(fLat, 4);
    Serial.print(F(" "));
    Serial.println(fLon, 4);
  }
  if (lbandOk && (lBandFreq != myLBandFreq)) {
    lbandOk = myLBand.setVal32(UBLOX_CFG_PMP_CENTER_FREQUENCY, lBandFreq); 
    if (lbandOk) {
      myLBand.softwareResetGNSSOnly(); // do a restart
      myLBandFreq = lBandFreq;
    }
    Serial.print(F("L-Band: new freq "));
    Serial.print(lBandFreq);
    Serial.println(OK(lbandOk));
  }
  // connect to mqtt IP Stream
  if (mqttOk && (ipTopic != myIpTopic)) {
    if (myIpTopic) mqttClient.unsubscribe(myIpTopic);
    mqttOk = mqttClient.subscribe(ipTopic);
    Serial.print(F("MQTT: subscribed to (new) IP topic "));
    Serial.print(ipTopic);
    Serial.println(OK(mqttOk)); 
    if (mqttOk) myIpTopic = ipTopic;
    if (mqttOk && gnssOk) {
      int /*TODO why is an ERROR returned ?*/ gnssOk = myGNSS.setVal8(UBLOX_CFG_SPARTN_USE_SOURCE, 0); // 0 = IP, 1 = LBAND
      Serial.print(F("GNSS: correction source IP"));
      Serial.println(OK(gnssOk));
    }
    Serial.print(F("MQTT: subscribed to IP topic "));
    Serial.print(ipTopic);
    Serial.println(OK(mqttOk));
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("PointPerfect testing"));
  Wire.begin(); // Start I2C
  last_ms = millis();
  
  if (!setupWiFi())
    Serial.println(F("WIFI: setup failed"));
  lbandOk = detectLBand();
  if (!lbandOk) Serial.println(F("L-Band: NEO-D9S check wiring"));
  gnssOk = detectGNSS();
  if (!gnssOk) Serial.println(F("GNSS: ZED-F9 check wiring"));
}

void loop()
{
  long now_ms = millis(); 
  if ((now_ms - last_ms) > 1000) {
    // detection every 1 second
    if (!lbandOk) lbandOk = detectLBand();
    else lbandOk = checkLband();
    if (!gnssOk)  gnssOk = detectGNSS();
    int connected = (WiFi.status() == WL_CONNECTED);
    if (connected != wifiConnected) {
      wifiConnected = connected;
      if (connected) {
        Serial.print(F("WiFi: connected with IP "));
        Serial.println(WiFi.localIP());
        mqttOk = false;
      }
      else {
        Serial.print(F("WiFi: disconnected"));
        mqttClient.stop();
        mqttOk = false;
        if (gnssOk) {
          int /*TODO why is an ERROR returned ?*/ gnssOk = myGNSS.setVal8(UBLOX_CFG_SPARTN_USE_SOURCE, 1); // 0 = IP, 1 = LBAND
          Serial.print(F("GNSS: correction source LBAND"));
          Serial.println(OK(gnssOk));
        }
      }
    }
    if (!mqttOk && wifiConnected) {
      mqttOk = mqttClient.connect(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
      Serial.print(F("MQTT: connection to broker "));
      Serial.print(AWS_IOT_ENDPOINT);
      Serial.println(OK(mqttOk));
      if(mqttOk) {
        const char* keyTopic = lbandOk ? MQTT_TOPIC_KEY_LBAND : MQTT_TOPIC_KEY_IP;
        mqttOk = mqttClient.subscribe(keyTopic);
        Serial.print(F("MQTT: subscribed to KEY topic "));
        Serial.print(keyTopic);
        Serial.println(OK(mqttOk));
        if (mqttOk) mqttOk = mqttClient.subscribe(MQTT_TOPIC_MGA);
        Serial.print(F("MQTT: subscribed to MGA topic "));
        Serial.print(MQTT_TOPIC_MGA);
        Serial.println(OK(mqttOk));
        myLBandFreq = 0;
        myIpTopic = NULL;
     }
    }
    last_ms = now_ms;
  }
  if (mqttOk)
    mqttClient.poll();
  if (gnssOk && myGNSS.getPVT() && (myGNSS.getInvalidLlh() == false)) {
    updatePointPerfectSources(myGNSS.getLongitude(), myGNSS.getLatitude());
  }
  delay(10);
}
